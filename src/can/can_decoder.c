#include "services/can_decoder.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "services/can_service.h"

K_MUTEX_DEFINE(decoder_mutex);

static struct vehicle_signal_config decoder_signals[CAN_DECODER_MAX_SIGNALS];
static size_t decoder_signal_count;
static struct vehicle_state current_state;
static bool initialized;

static bool signal_config_is_valid(const struct vehicle_signal_config *signal)
{
    uint32_t max_can_id = signal->extended_id ? 0x1FFFFFFFU : 0x7FFU;

    return (unsigned int)signal->signal < VEHICLE_SIGNAL_COUNT &&
           signal->can_id <= max_can_id &&
           signal->start_bit < 8U &&
           signal->bit_length > 0U &&
           signal->bit_length <= 64U &&
           (unsigned int)signal->endianness <=
               VEHICLE_SIGNAL_BIG_ENDIAN &&
           (unsigned int)signal->format <= VEHICLE_SIGNAL_SIGNED;
}

static int extract_raw_value(
    const struct vehicle_signal_config *signal,
    const struct can_frame *frame,
    uint64_t *raw_value
)
{
    size_t payload_bytes = can_dlc_to_bytes(frame->dlc);

    if (payload_bytes > sizeof(frame->data)) {
        payload_bytes = sizeof(frame->data);
    }

    size_t payload_bits = payload_bytes * 8U;
    size_t first_bit = (size_t)signal->start_byte * 8U + signal->start_bit;

    if (first_bit + signal->bit_length > payload_bits) {
        return -EMSGSIZE;
    }

    uint64_t raw = 0U;

    if (signal->endianness == VEHICLE_SIGNAL_LITTLE_ENDIAN) {
        for (uint8_t i = 0U; i < signal->bit_length; ++i) {
            size_t bit_index = first_bit + i;
            uint8_t bit = (frame->data[bit_index / 8U] >>
                           (bit_index % 8U)) & 1U;

            raw |= (uint64_t)bit << i;
        }
    } else {
        for (uint8_t i = 0U; i < signal->bit_length; ++i) {
            size_t bit_index = first_bit + i;
            uint8_t bit = (frame->data[bit_index / 8U] >>
                           (7U - (bit_index % 8U))) & 1U;

            raw = (raw << 1U) | bit;
        }
    }

    *raw_value = raw;
    return 0;
}

static float raw_to_number(
    const struct vehicle_signal_config *signal,
    uint64_t raw
)
{
    if (signal->format == VEHICLE_SIGNAL_UNSIGNED) {
        return (float)raw;
    }

    uint64_t sign_bit = UINT64_C(1) << (signal->bit_length - 1U);

    if ((raw & sign_bit) == 0U) {
        return (float)raw;
    }

    uint64_t mask = signal->bit_length == 64U
                        ? UINT64_MAX
                        : (UINT64_C(1) << signal->bit_length) - 1U;
    uint64_t magnitude = ((~raw) + 1U) & mask;

    return -(float)magnitude;
}

static void update_vehicle_state(
    enum vehicle_signal_id signal,
    float physical_value,
    int64_t updated_at_ms
)
{
    switch (signal) {
    case VEHICLE_SIGNAL_SPEED:
        current_state.speed_kph = physical_value;
        current_state.speed_updated_at_ms = updated_at_ms;
        break;
    case VEHICLE_SIGNAL_ENGINE_RPM:
        current_state.engine_rpm = physical_value;
        current_state.engine_rpm_updated_at_ms = updated_at_ms;
        break;
    case VEHICLE_SIGNAL_THROTTLE_POSITION:
        current_state.throttle_position_pct = physical_value;
        current_state.throttle_position_updated_at_ms = updated_at_ms;
        break;
    default:
        return;
    }

    current_state.valid_signals |= 1U << signal;
    ++current_state.update_count;
}

static void decoder_rx_handler(const struct can_frame *frame, void *user_data)
{
    ARG_UNUSED(user_data);
    (void)can_decoder_decode_frame(frame);
}

int can_decoder_init(const struct vehicle_config *config)
{
    if (config == NULL || config->signals == NULL ||
        config->signal_count == 0U) {
        return -EINVAL;
    }

    if (config->signal_count > CAN_DECODER_MAX_SIGNALS) {
        return -E2BIG;
    }

    for (size_t i = 0U; i < config->signal_count; ++i) {
        if (!signal_config_is_valid(&config->signals[i])) {
            return -EINVAL;
        }
    }

    k_mutex_lock(&decoder_mutex, K_FOREVER);

    memcpy(decoder_signals, config->signals,
           config->signal_count * sizeof(decoder_signals[0]));
    decoder_signal_count = config->signal_count;
    memset(&current_state, 0, sizeof(current_state));
    initialized = true;

    k_mutex_unlock(&decoder_mutex);

    can_service_set_rx_handler(decoder_rx_handler, NULL);
    return 0;
}

int can_decoder_decode_frame(const struct can_frame *frame)
{
    if (frame == NULL) {
        return -EINVAL;
    }

    if ((frame->flags & CAN_FRAME_RTR) != 0U) {
        return 0;
    }

    k_mutex_lock(&decoder_mutex, K_FOREVER);

    if (!initialized) {
        k_mutex_unlock(&decoder_mutex);
        return -EACCES;
    }

    int updated = 0;
    int error = 0;
    int64_t updated_at_ms = k_uptime_get();

    for (size_t i = 0U; i < decoder_signal_count; ++i) {
        const struct vehicle_signal_config *signal = &decoder_signals[i];
        bool frame_is_extended = (frame->flags & CAN_FRAME_IDE) != 0U;

        if (frame->id != signal->can_id ||
            frame_is_extended != signal->extended_id) {
            continue;
        }

        uint64_t raw;
        int ret = extract_raw_value(signal, frame, &raw);

        if (ret != 0) {
            error = ret;
            continue;
        }

        float physical = raw_to_number(signal, raw) * signal->scale +
                         signal->offset;

        update_vehicle_state(signal->signal, physical, updated_at_ms);
        ++updated;
    }

    k_mutex_unlock(&decoder_mutex);

    return updated > 0 ? updated : error;
}

int can_decoder_get_vehicle_state(struct vehicle_state *state)
{
    if (state == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&decoder_mutex, K_FOREVER);

    if (!initialized) {
        k_mutex_unlock(&decoder_mutex);
        return -EACCES;
    }

    *state = current_state;

    k_mutex_unlock(&decoder_mutex);
    return 0;
}

bool can_decoder_is_initialized(void)
{
    k_mutex_lock(&decoder_mutex, K_FOREVER);
    bool result = initialized;
    k_mutex_unlock(&decoder_mutex);

    return result;
}
