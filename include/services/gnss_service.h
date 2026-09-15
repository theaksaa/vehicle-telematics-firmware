#ifndef GNSS_SERVICE_H
#define GNSS_SERVICE_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/gnss.h>

#define GNSS_SERVICE_MAX_SATELLITES 32

int gnss_service_init(void);

bool gnss_service_is_ready(void);

bool gnss_service_get_data(struct gnss_data *data);

uint16_t gnss_service_get_satellites(
    struct gnss_satellite *satellites,
    uint16_t capacity
);

int64_t gnss_service_last_update(void);

int gnss_service_get_awake(bool *awake);

int gnss_service_wake(void);

int gnss_service_sleep(void);

#if defined(CONFIG_TELEMATICS_SIMULATION)
void gnss_service_set_simulation_enabled(bool enabled);

bool gnss_service_simulation_is_enabled(void);

int gnss_service_inject_data(const struct gnss_data *data);

int gnss_service_inject_satellites(
    const struct gnss_satellite *satellites,
    uint16_t count
);
#endif

#endif
