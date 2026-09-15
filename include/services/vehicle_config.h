#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum vehicle_signal_id {
    VEHICLE_SIGNAL_SPEED,
    VEHICLE_SIGNAL_ENGINE_RPM,
    VEHICLE_SIGNAL_THROTTLE_POSITION,
    VEHICLE_SIGNAL_COUNT
};

enum vehicle_signal_endianness {
    VEHICLE_SIGNAL_LITTLE_ENDIAN,
    VEHICLE_SIGNAL_BIG_ENDIAN
};

enum vehicle_signal_format {
    VEHICLE_SIGNAL_UNSIGNED,
    VEHICLE_SIGNAL_SIGNED
};

/*
 * Signal bit numbering:
 *
 * - start_byte is a zero-based payload byte index.
 * - For little-endian signals, start_bit 0 is the least-significant bit of
 *   start_byte. Successive bits move toward bit 7 and then the next byte.
 * - For big-endian signals, start_bit 0 is the most-significant bit of
 *   start_byte. Successive bits move toward bit 0 and then the next byte.
 *
 * The physical value is raw_value * scale + offset.
 */
struct vehicle_signal_config {
    enum vehicle_signal_id signal;
    uint32_t can_id;
    bool extended_id;
    uint8_t start_byte;
    uint8_t start_bit;
    uint8_t bit_length;
    enum vehicle_signal_endianness endianness;
    enum vehicle_signal_format format;
    float scale;
    float offset;
};

struct vehicle_config {
    const struct vehicle_signal_config *signals;
    size_t signal_count;
};

/*
 * Example packed frame definition. Replace these IDs and bit positions with
 * the values from the target vehicle's CAN database (DBC).
 */
extern const struct vehicle_config vehicle_config_default;

#endif
