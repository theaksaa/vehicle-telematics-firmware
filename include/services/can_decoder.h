#ifndef CAN_DECODER_H
#define CAN_DECODER_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

#include "services/vehicle_config.h"

#define CAN_DECODER_MAX_SIGNALS 16U

enum vehicle_state_validity {
    VEHICLE_STATE_SPEED_VALID = (1U << VEHICLE_SIGNAL_SPEED),
    VEHICLE_STATE_ENGINE_RPM_VALID = (1U << VEHICLE_SIGNAL_ENGINE_RPM),
    VEHICLE_STATE_THROTTLE_POSITION_VALID =
        (1U << VEHICLE_SIGNAL_THROTTLE_POSITION)
};

struct vehicle_state {
    float speed_kph;
    float engine_rpm;
    float throttle_position_pct;
    int64_t speed_updated_at_ms;
    int64_t engine_rpm_updated_at_ms;
    int64_t throttle_position_updated_at_ms;
    uint32_t valid_signals;
    uint32_t update_count;
};

/*
 * Initializes the decoder, clears its state, and attaches it to can_service.
 * can_service_init() must be called first. The configuration is copied.
 */
int can_decoder_init(const struct vehicle_config *config);

/* Decode one frame. Returns the number of updated signals, or a negative errno. */
int can_decoder_decode_frame(const struct can_frame *frame);

/* Obtain a thread-safe snapshot of the latest decoded values. */
int can_decoder_get_vehicle_state(struct vehicle_state *state);

bool can_decoder_is_initialized(void);

#endif
