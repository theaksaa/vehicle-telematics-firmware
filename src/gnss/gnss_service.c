#include "services/gnss_service.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define GNSS_DEVICE DEVICE_DT_GET(DT_ALIAS(gnss))
#define USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec gps_pwr =
    GPIO_DT_SPEC_GET(USER_NODE, gps_pwr_gpios);

static const struct gpio_dt_spec gps_wup =
    GPIO_DT_SPEC_GET(USER_NODE, gps_wup_gpios);

K_MUTEX_DEFINE(gnss_mutex);

static struct gnss_data latest_data;
static bool data_received;
static int64_t last_update_ms;

static struct gnss_satellite latest_satellites[GNSS_SERVICE_MAX_SATELLITES];
static uint16_t latest_satellite_count;

static void gnss_data_callback(
    const struct device *dev,
    const struct gnss_data *data)
{
    k_mutex_lock(&gnss_mutex, K_FOREVER);

    latest_data = *data;
    data_received = true;
    last_update_ms = k_uptime_get();

    k_mutex_unlock(&gnss_mutex);
}

GNSS_DATA_CALLBACK_DEFINE(GNSS_DEVICE, gnss_data_callback);

static void gnss_satellites_callback(
    const struct device *dev,
    const struct gnss_satellite *satellites,
    uint16_t size)
{
    k_mutex_lock(&gnss_mutex, K_FOREVER);

    latest_satellite_count =
        MIN(size, ARRAY_SIZE(latest_satellites));

    memcpy(
        latest_satellites,
        satellites,
        latest_satellite_count * sizeof(struct gnss_satellite)
    );

    k_mutex_unlock(&gnss_mutex);
}

GNSS_SATELLITES_CALLBACK_DEFINE(
    GNSS_DEVICE,
    gnss_satellites_callback
);

int gnss_service_init(void)
{
    if (!device_is_ready(GNSS_DEVICE)) {
        return -ENODEV;
    }

    if (!gpio_is_ready_dt(&gps_pwr) ||
        !gpio_is_ready_dt(&gps_wup)) {
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(
        &gps_pwr,
        GPIO_OUTPUT_INACTIVE
    );

    if (ret < 0) {
        return ret;
    }

    return gpio_pin_configure_dt(
        &gps_wup,
        GPIO_INPUT
    );
}

bool gnss_service_is_ready(void)
{
    return device_is_ready(GNSS_DEVICE);
}

bool gnss_service_get_data(struct gnss_data *data)
{
    k_mutex_lock(&gnss_mutex, K_FOREVER);

    bool available = data_received;

    if (available) {
        *data = latest_data;
    }

    k_mutex_unlock(&gnss_mutex);

    return available;
}

uint16_t gnss_service_get_satellites(
    struct gnss_satellite *satellites,
    uint16_t capacity)
{
    k_mutex_lock(&gnss_mutex, K_FOREVER);

    uint16_t count =
        MIN(capacity, latest_satellite_count);

    memcpy(
        satellites,
        latest_satellites,
        count * sizeof(struct gnss_satellite)
    );

    k_mutex_unlock(&gnss_mutex);

    return count;
}

int64_t gnss_service_last_update(void)
{
    k_mutex_lock(&gnss_mutex, K_FOREVER);

    int64_t value =
        data_received ? last_update_ms : -1;

    k_mutex_unlock(&gnss_mutex);

    return value;
}

int gnss_service_get_awake(bool *awake)
{
    int value = gpio_pin_get_dt(&gps_wup);

    if (value < 0) {
        return value;
    }

    *awake = value != 0;

    return 0;
}

static int pulse_power_control(void)
{
    int ret = gpio_pin_set_dt(&gps_pwr, 1);

    if (ret < 0) {
        return ret;
    }

    k_msleep(100);

    return gpio_pin_set_dt(&gps_pwr, 0);
}

int gnss_service_wake(void)
{
    bool awake;
    int ret = gnss_service_get_awake(&awake);

    if (ret < 0) {
        return ret;
    }

    if (awake) {
        return 0;
    }

    for (int attempt = 0; attempt < 3; attempt++) {
        ret = pulse_power_control();

        if (ret < 0) {
            return ret;
        }

        for (int i = 0; i < 10; i++) {
            k_msleep(100);

            ret = gnss_service_get_awake(&awake);

            if (ret < 0) {
                return ret;
            }

            if (awake) {
                return 0;
            }
        }
    }

    return -ETIMEDOUT;
}

int gnss_service_sleep(void)
{
    bool awake;
    int ret = gnss_service_get_awake(&awake);

    if (ret < 0) {
        return ret;
    }

    if (!awake) {
        return 0;
    }

    ret = pulse_power_control();

    if (ret < 0) {
        return ret;
    }

    for (int i = 0; i < 15; i++) {
        k_msleep(100);

        ret = gnss_service_get_awake(&awake);

        if (ret < 0) {
            return ret;
        }

        if (!awake) {
            return 0;
        }
    }

    return -ETIMEDOUT;
}