#include <zephyr/kernel.h>

int main(void)
{
    printk("Hello Zephyr on RPi Pico!\n");

    while (1) {
        k_sleep(K_SECONDS(1));
    }
    return 0;
}