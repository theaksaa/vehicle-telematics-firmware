#include "services/can_service.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(can_service, LOG_LEVEL_INF);

#define CAN_SERVICE_RX_QUEUE_SIZE       64
#define CAN_SERVICE_THREAD_STACK_SIZE   1536
#define CAN_SERVICE_THREAD_PRIORITY     5
#define CAN_SERVICE_TX_TIMEOUT          K_MSEC(100)

#if !DT_HAS_CHOSEN(zephyr_canbus)
#error "Devicetree chosen node 'zephyr,canbus' is required"
#endif

static const struct device *const can_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

K_MSGQ_DEFINE(
    rx_queue,
    sizeof(struct can_frame),
    CAN_SERVICE_RX_QUEUE_SIZE,
    4
);

K_MUTEX_DEFINE(data_mutex);

static atomic_t initialized;
static atomic_t running;

static atomic_t rx_frames;
static atomic_t rx_dropped;

static atomic_t tx_frames;
static atomic_t tx_errors;

static int standard_filter_id = -1;
static int extended_filter_id = -1;

static enum can_service_mode current_mode = CAN_SERVICE_MODE_NORMAL;

static struct can_frame last_frame;
static bool last_frame_valid;

static can_service_rx_handler_t rx_handler;
static void *rx_handler_user_data;


/*
 * Called from CAN driver interrupt context.
 *
 * Do as little work here as possible.
 * The frame is copied to a message queue and processed later
 * by can_service_thread().
 */
static void can_rx_callback(
    const struct device *dev,
    struct can_frame *frame,
    void *user_data
)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    atomic_inc(&rx_frames);

    if (k_msgq_put(&rx_queue, frame, K_NO_WAIT) != 0) {
        atomic_inc(&rx_dropped);
    }
}


static void can_service_thread(
    void *p1,
    void *p2,
    void *p3
)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct can_frame frame;

    while (true) {
        k_msgq_get(&rx_queue, &frame, K_FOREVER);

        can_service_rx_handler_t handler;
        void *handler_user_data;

        k_mutex_lock(&data_mutex, K_FOREVER);

        last_frame = frame;
        last_frame_valid = true;

        handler = rx_handler;
        handler_user_data = rx_handler_user_data;

        k_mutex_unlock(&data_mutex);

        /*
         * This runs in thread context, NOT interrupt context.
         *
         * Later, can_decoder can register itself here.
         */
        if (handler != NULL) {
            handler(&frame, handler_user_data);
        }
    }
}


K_THREAD_DEFINE(
    can_service_thread_id,
    CAN_SERVICE_THREAD_STACK_SIZE,
    can_service_thread,
    NULL,
    NULL,
    NULL,
    CAN_SERVICE_THREAD_PRIORITY,
    0,
    0
);


static can_mode_t to_zephyr_mode(enum can_service_mode mode)
{
    switch (mode) {
    case CAN_SERVICE_MODE_NORMAL:
        return CAN_MODE_NORMAL;

    case CAN_SERVICE_MODE_LISTEN_ONLY:
        return CAN_MODE_LISTENONLY;

    case CAN_SERVICE_MODE_LOOPBACK:
        return CAN_MODE_LOOPBACK;

    default:
        return CAN_MODE_NORMAL;
    }
}


int can_service_init(void)
{
    if (atomic_get(&initialized)) {
        return 0;
    }

    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN device is not ready");
        return -ENODEV;
    }

    /*
     * Receive every standard 11-bit CAN frame.
     */
    const struct can_filter standard_filter = {
        .id = 0,
        .mask = 0,
        .flags = 0
    };

    standard_filter_id = can_add_rx_filter(
        can_dev,
        can_rx_callback,
        NULL,
        &standard_filter
    );

    if (standard_filter_id < 0) {
        LOG_ERR(
            "Failed to add standard CAN filter: %d",
            standard_filter_id
        );

        return standard_filter_id;
    }

    /*
     * Receive every extended 29-bit CAN frame.
     */
    const struct can_filter extended_filter = {
        .id = 0,
        .mask = 0,
        .flags = CAN_FILTER_IDE
    };

    extended_filter_id = can_add_rx_filter(
        can_dev,
        can_rx_callback,
        NULL,
        &extended_filter
    );

    if (extended_filter_id < 0) {
        LOG_ERR(
            "Failed to add extended CAN filter: %d",
            extended_filter_id
        );

        can_remove_rx_filter(
            can_dev,
            standard_filter_id
        );

        standard_filter_id = -1;

        return extended_filter_id;
    }

    atomic_set(&rx_frames, 0);
    atomic_set(&rx_dropped, 0);
    atomic_set(&tx_frames, 0);
    atomic_set(&tx_errors, 0);

    k_mutex_lock(&data_mutex, K_FOREVER);

    last_frame_valid = false;
    rx_handler = NULL;
    rx_handler_user_data = NULL;

    k_mutex_unlock(&data_mutex);

    atomic_set(&initialized, 1);

    LOG_INF("CAN service initialized");

    return 0;
}


int can_service_start(enum can_service_mode mode)
{
    int ret;

    if (!atomic_get(&initialized)) {
        ret = can_service_init();

        if (ret != 0) {
            return ret;
        }
    }

    if (atomic_get(&running)) {
        if (current_mode == mode) {
            return 0;
        }

        LOG_ERR("CAN service is already running");
        return -EBUSY;
    }

    can_mode_t zephyr_mode = to_zephyr_mode(mode);

    ret = can_set_mode(can_dev, zephyr_mode);

    if (ret != 0) {
        LOG_ERR(
            "Failed to set CAN mode: %d",
            ret
        );

        return ret;
    }

    ret = can_start(can_dev);

    if (ret != 0 && ret != -EALREADY) {
        LOG_ERR(
            "Failed to start CAN controller: %d",
            ret
        );

        return ret;
    }

    current_mode = mode;

    atomic_set(&running, 1);

    LOG_INF(
        "CAN service started in mode %d",
        mode
    );

    return 0;
}


int can_service_stop(void)
{
    if (!atomic_get(&initialized)) {
        return -EACCES;
    }

    if (!atomic_get(&running)) {
        return 0;
    }

    int ret = can_stop(can_dev);

    if (ret != 0 && ret != -EALREADY) {
        LOG_ERR(
            "Failed to stop CAN controller: %d",
            ret
        );

        return ret;
    }

    atomic_set(&running, 0);

    /*
     * Do not process stale frames after the service is
     * started again.
     */
    k_msgq_purge(&rx_queue);

    LOG_INF("CAN service stopped");

    return 0;
}


bool can_service_is_running(void)
{
    return atomic_get(&running) != 0;
}


int can_service_send(const struct can_frame *frame)
{
    if (frame == NULL) {
        return -EINVAL;
    }

    if (!atomic_get(&running)) {
        return -ENETDOWN;
    }

    /*
     * Listen-only is what we will normally use in the vehicle.
     * Transmission must never happen in this mode.
     */
    if (current_mode == CAN_SERVICE_MODE_LISTEN_ONLY) {
        return -EPERM;
    }

    int ret = can_send(
        can_dev,
        frame,
        CAN_SERVICE_TX_TIMEOUT,
        NULL,
        NULL
    );

    if (ret != 0) {
        atomic_inc(&tx_errors);

        LOG_ERR(
            "Failed to send CAN frame: %d",
            ret
        );

        return ret;
    }

    atomic_inc(&tx_frames);

    return 0;
}


void can_service_set_rx_handler(
    can_service_rx_handler_t handler,
    void *user_data
)
{
    k_mutex_lock(&data_mutex, K_FOREVER);

    rx_handler = handler;
    rx_handler_user_data = user_data;

    k_mutex_unlock(&data_mutex);
}


int can_service_get_last_frame(struct can_frame *frame)
{
    if (frame == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&data_mutex, K_FOREVER);

    if (!last_frame_valid) {
        k_mutex_unlock(&data_mutex);
        return -ENODATA;
    }

    *frame = last_frame;

    k_mutex_unlock(&data_mutex);

    return 0;
}


int can_service_get_stats(struct can_service_stats *stats)
{
    if (stats == NULL) {
        return -EINVAL;
    }

    if (!atomic_get(&initialized)) {
        return -EACCES;
    }

    struct can_bus_err_cnt error_count;
    enum can_state state;

    int ret = can_get_state(
        can_dev,
        &state,
        &error_count
    );

    if (ret != 0) {
        LOG_ERR(
            "Failed to get CAN state: %d",
            ret
        );

        return ret;
    }

    stats->initialized =
        atomic_get(&initialized) != 0;

    stats->running =
        atomic_get(&running) != 0;

    stats->mode = current_mode;

    stats->rx_frames =
        (uint32_t)atomic_get(&rx_frames);

    stats->rx_dropped =
        (uint32_t)atomic_get(&rx_dropped);

    stats->tx_frames =
        (uint32_t)atomic_get(&tx_frames);

    stats->tx_errors =
        (uint32_t)atomic_get(&tx_errors);

    stats->state = state;

    stats->tx_error_count =
        error_count.tx_err_cnt;

    stats->rx_error_count =
        error_count.rx_err_cnt;

    return 0;
}