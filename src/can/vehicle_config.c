#include "services/vehicle_config.h"

/*
 * Candidate VW PQ-platform layout used by the bench simulator:
 *   0x1A0, bits 17..31: vehicle speed in 0.01 km/h
 *   0x280, bytes 2..3: engine speed in 0.25 rpm
 *   0x280, byte 5: throttle position in 0.4 percent
 * Verify these signals against captures from the target vehicle before use.
 */
static const struct vehicle_signal_config default_signals[] = {
    {
        .signal = VEHICLE_SIGNAL_SPEED,
        .can_id = 0x1A0,
        .extended_id = false,
        .start_byte = 2,
        .start_bit = 1,
        .bit_length = 15,
        .endianness = VEHICLE_SIGNAL_LITTLE_ENDIAN,
        .format = VEHICLE_SIGNAL_UNSIGNED,
        .scale = 0.01f,
        .offset = 0.0f,
    },
    {
        .signal = VEHICLE_SIGNAL_ENGINE_RPM,
        .can_id = 0x280,
        .extended_id = false,
        .start_byte = 2,
        .start_bit = 0,
        .bit_length = 16,
        .endianness = VEHICLE_SIGNAL_LITTLE_ENDIAN,
        .format = VEHICLE_SIGNAL_UNSIGNED,
        .scale = 0.25f,
        .offset = 0.0f,
    },
    {
        .signal = VEHICLE_SIGNAL_THROTTLE_POSITION,
        .can_id = 0x280,
        .extended_id = false,
        .start_byte = 5,
        .start_bit = 0,
        .bit_length = 8,
        .endianness = VEHICLE_SIGNAL_LITTLE_ENDIAN,
        .format = VEHICLE_SIGNAL_UNSIGNED,
        .scale = 0.4f,
        .offset = 0.0f,
    },
};

const struct vehicle_config vehicle_config_default = {
    .signals = default_signals,
    .signal_count = sizeof(default_signals) / sizeof(default_signals[0]),
};
