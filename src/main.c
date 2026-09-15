#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "gnss/gnss_service.h"

int main(void)
{
    int ret = gnss_service_init();

    if (ret < 0) {
        printk("GNSS service initialization failed: %d\n", ret);
    }

    printk("Vehicle telematics firmware started\n");

    return 0;
}