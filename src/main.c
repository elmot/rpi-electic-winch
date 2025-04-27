#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>


#if !DT_NODE_HAS_STATUS(DT_ALIAS(pwm_motor0), okay)
#error "pwm_motor0 device is not enabled"
#endif


static const struct pwm_dt_spec pwm_motor0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0));

int main(void)
{
    int ret;

    if (!device_is_ready(pwm_motor0.dev)) {
        printk("PWM device %s is not ready\n", pwm_motor0.dev->name);
        return 0;
    }

    printk("PWM device %s is ready\n", pwm_motor0.dev->name);

    while (1) {
        // Set PWM duty cycle for motor0 (GPIO16) - Example: 50%
        ret = pwm_set_dt(&pwm_motor0, PWM_USEC(1000), PWM_USEC(500)); // 50% duty cycle
        if (ret < 0) {
            printk("Error setting PWM duty cycle for motor0: %d\n", ret);
        }

        k_sleep(K_MSEC(1000));
    }

    return 0;
}