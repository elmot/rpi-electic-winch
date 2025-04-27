#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>


#if !DT_NODE_HAS_STATUS(DT_ALIAS(pwm_motor0), okay)
#error "pwm_motor0 device is not enabled"
#endif

#if !DT_NODE_HAS_STATUS(DT_ALIAS(pwm_motor1), okay)
#error "pwm_motor1 device is not enabled"
#endif



static const struct pwm_dt_spec pwm_motor0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0));
static const struct pwm_dt_spec pwm_motor1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1));

int main(void)
{
    int ret;

    if (!device_is_ready(pwm_motor0.dev)) {
        printk("PWM device %s is not ready\n", pwm_motor0.dev->name);
        return 0;
    }
    if (!device_is_ready(pwm_motor1.dev)) {
        printk("PWM device %s is not ready\n", pwm_motor1.dev->name);
        return 1;
    }

    printk("PWM devices %s and %s are ready\n", pwm_motor0.dev->name, pwm_motor1.dev->name);

    while (1) {
        // Set PWM duty cycle for motor0 (GPIO16) - Example: 50%
        ret = pwm_set_dt(&pwm_motor0, PWM_USEC(1000), PWM_USEC(500)); // 50% duty cycle
        if (ret < 0) {
            printk("Error setting PWM duty cycle for motor0: %d\n", ret);
        }

        // Set PWM duty cycle for motor1 (GPIO17) - Example: 25%
        ret = pwm_set_dt(&pwm_motor1, PWM_USEC(1000), PWM_USEC(250)); // 25% duty cycle
        if (ret < 0) {
            printk("Error setting PWM duty cycle for motor1: %d\n", ret);
        }

        k_sleep(K_MSEC(1000));
    }

    return 0;
}