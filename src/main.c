#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "zephyr/usb/usb_device.h"
#include <zephyr/sys/printk.h>

#include "zephyr/drivers/sensor.h"

#if !DT_NODE_HAS_STATUS(DT_ALIAS(pwm_motor0), okay)
#error "pwm_motor0 device is not enabled"
#endif

#if !DT_NODE_HAS_STATUS(DT_ALIAS(pwm_motor1), okay)
#error "pwm_motor1 device is not enabled"
#endif

#if !DT_NODE_HAS_STATUS(DT_ALIAS(as5600_sensor), okay)
    #error "as5600_sensor device alias not found or not enabled"
    #endif


static const struct pwm_dt_spec pwm_motor0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0));
static const struct pwm_dt_spec pwm_motor1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1));
static const struct device *const as5600_dev = DEVICE_DT_GET(DT_ALIAS(as5600_sensor));

int main(void)
{
    int ret;
    usb_enable(NULL);

    k_msleep(10000);

    if (!device_is_ready(pwm_motor0.dev)) {
        printk("PWM device %s is not ready\n", pwm_motor0.dev->name);
        return 1;
    }
    if (!device_is_ready(pwm_motor1.dev)) {
        printk("PWM device %s is not ready\n", pwm_motor1.dev->name);
        return 2;
    }

    if (!device_is_ready(as5600_dev)) {
        printk("AS5600 sensor device not ready.\n");
        return 3;
    }

    printk("PWM devices %s and %s are ready\n", pwm_motor0.dev->name, pwm_motor1.dev->name);

    while (1) {
        // Set PWM duty cycle for motor0 (GPIO16) - Example: 50%
        // ret = pwm_set_dt(&pwm_motor0, PWM_USEC(1000), PWM_USEC(500)); // 50% duty cycle
        if (ret < 0) {
            printk("Error setting PWM duty cycle for motor0: %d\n", ret);
        }

        // Set PWM duty cycle for motor1 (GPIO17) - Example: 25%
        // ret = pwm_set_dt(&pwm_motor1, PWM_USEC(1000), PWM_USEC(250)); // 25% duty cycle
        if (ret < 0) {
            printk("Error setting PWM duty cycle for motor1: %d\n", ret);
        }
        printk("PWM duties set\n");

        static struct sensor_value angle_val;
        ret = sensor_sample_fetch(as5600_dev);

        if (ret < 0) {
            printk("Failed to fetch sample from AS5600: %d\n", ret);
        } else {
            // Get the angle reading (SENSOR_CHAN_ROTATION)
            ret = sensor_channel_get(as5600_dev, SENSOR_CHAN_ROTATION, &angle_val);
            if (ret < 0) {
                printk("Failed to get angle data from AS5600: %d\n", ret);
            } else {
                // The angle is returned in degrees * 100
                // Use sensor_value_to_double or integer arithmetic as needed
                int angle_degrees = sensor_value_to_milli(&angle_val);
                // printk("AS5600 Angle: %d.%02d degrees\n", angle_val.val1, angle_val.val2 / 10000); // Integer printing
                printk("AS5600 Angle: %d degrees\n", angle_degrees); // Floating point printing (requires CONFIG_NEWLIB_LIBC or similar)

            }
        }

        k_sleep(K_MSEC(1000));
    }

    return 0;
}