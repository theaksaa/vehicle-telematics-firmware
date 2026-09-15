#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "services/gnss_service.h"
#include "services/can_decoder.h"
#include "services/can_service.h"
#include "services/vehicle_config.h"

int main(void)
{
    int ret = gnss_service_init();

    if (ret < 0) {
        printk("GNSS service initialization failed: %d\n", ret);
    }

    ret = can_service_init();

    if (ret != 0) {
        printk("CAN service init failed: %d\n", ret);
        return ret;
    }

    ret = can_decoder_init(&vehicle_config_default);

    if (ret != 0) {
        printk("CAN decoder init failed: %d\n", ret);
        return ret;
    }

#if defined(CONFIG_TELEMATICS_SIMULATION)
    printk("WARNING: bench simulation enabled; CAN is in normal mode\n");
    ret = can_service_start(CAN_SERVICE_MODE_NORMAL);
#else
    ret = can_service_start(CAN_SERVICE_MODE_LISTEN_ONLY);
#endif

    if (ret != 0) {
        printk("CAN service start failed: %d\n", ret);
        return ret;
    }

    printk("Vehicle telematics firmware started\n");

    return 0;
}
