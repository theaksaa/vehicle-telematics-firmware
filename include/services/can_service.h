#ifndef CAN_SERVICE_H
#define CAN_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

enum can_service_mode {
    CAN_SERVICE_MODE_NORMAL,
    CAN_SERVICE_MODE_LISTEN_ONLY,
    CAN_SERVICE_MODE_LOOPBACK
};

typedef void (*can_service_rx_handler_t)(
    const struct can_frame *frame,
    void *user_data
);

struct can_service_stats {
    bool initialized;
    bool running;

    enum can_service_mode mode;

    uint32_t rx_frames;
    uint32_t rx_dropped;

    uint32_t tx_frames;
    uint32_t tx_errors;

    enum can_state state;

    uint8_t tx_error_count;
    uint8_t rx_error_count;
};

int can_service_init(void);

int can_service_start(enum can_service_mode mode);

int can_service_stop(void);

bool can_service_is_running(void);

int can_service_send(const struct can_frame *frame);

void can_service_set_rx_handler(
    can_service_rx_handler_t handler,
    void *user_data
);

int can_service_get_last_frame(struct can_frame *frame);

int can_service_get_stats(struct can_service_stats *stats);

#endif