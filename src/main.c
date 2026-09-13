#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
    printk("Vehicle telematics firmware started\n");

    return 0;
}