#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "zephyr/usb/usb_device.h"
#include <zephyr/sys/printk.h>

#include "zephyr/drivers/sensor.h"

#include "params.h"
//todo algorithms
//todo safe start

//todo PHASE II battery watching
//todo PHASE III sensor status
//todo PHASE IV motor current alarm

#if !DT_NODE_HAS_STATUS(DT_ALIAS(pwm_motor0), okay)
#error "pwm_motor0 device is not enabled"
#endif

#if !DT_NODE_HAS_STATUS(DT_ALIAS(pwm_motor1), okay)
#error "pwm_motor1 device is not enabled"
#endif

// <<< Add check for pwm_led alias >>>
#if !DT_NODE_HAS_STATUS(DT_ALIAS(pwm_led), okay)
#error "pwm_led device alias not found or not enabled"
#endif

#if !DT_NODE_HAS_STATUS(DT_ALIAS(as5600_sensor), okay)
    #error "as5600_sensor device alias not found or not enabled"
#endif

enum led_status_type
{
    OFF, ON, DIM, ALARM
};

atomic_t led_status = ATOMIC_INIT(OFF); // Initialize to OFF

static const struct pwm_dt_spec pwm_motor0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0));
static const struct pwm_dt_spec pwm_motor1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1));
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led));

static const struct device* const as5600_dev = DEVICE_DT_GET(DT_ALIAS(as5600_sensor));

#define LED_STACK_SIZE 512
#define LED_PRIORITY 5
K_THREAD_STACK_DEFINE(led_stack_area, LED_STACK_SIZE);
struct k_thread led_thread_data;

void startup_device(const struct device* dev)
{
    while (!device_is_ready(dev)) {
        atomic_set(&led_status, ALARM);
        printk("Device %s is not ready\n", dev->name);
        k_sleep(K_MSEC(1000));
    }
    // Set back to OFF only if no other part set it to ALARM meanwhile
    atomic_set(&led_status, OFF);
}

/** Set motor PWM
 *
 * @param duty PWM duty, [-100..100]
 */
void motor_pwm(int duty)
{
    static int old_duty = 10000000;
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;
    if (old_duty == duty) return;
    old_duty = duty;
    int ret;
    enum led_status_type new_status;
    if (duty > MIN_PWM_PERCENT) {
        ret =  pwm_set_dt(&pwm_motor0, PWM_USEC(40), PWM_USEC(40 * duty / 100));
        ret |= pwm_set_dt(&pwm_motor1, PWM_USEC(40), PWM_USEC(0));
        new_status = duty >=99 ? ON : DIM;
    } else if (duty < -MIN_PWM_PERCENT) {
        ret = pwm_set_dt(&pwm_motor0, PWM_USEC(40), PWM_USEC(0));
        ret |= pwm_set_dt(&pwm_motor1, PWM_USEC(40), PWM_USEC(-40 * duty / 100));
        new_status = duty <=-99 ? ON : DIM;
    } else {
        ret = pwm_set_dt(&pwm_motor0, PWM_USEC(40), PWM_USEC(0));
        ret |= pwm_set_dt(&pwm_motor1, PWM_USEC(40), PWM_USEC(0));
        new_status = OFF;
    }
    if (ret) {
        printk("Failed to set motor PWM: %d\n", ret);
        atomic_set(&led_status, ALARM);
    } else {
        printk("Motor PWM: %d\n", duty);
        atomic_set(&led_status, new_status);
    }
}

_Noreturn void alarm(const char* fmt, ...)
{
    motor_pwm(0);
    atomic_set(&led_status, ALARM);
    while (1) {
        va_list ap;
        va_start(ap, fmt);
        vprintk(fmt, ap);
        va_end(ap);
        k_sleep(K_SECONDS(1));
    }
}


_Noreturn void led_task_entry(__unused void* p1,__unused void* p2,__unused void* p3)
{
    static bool phase = false;
    while (true) {
        if (phase) {
            if(led_status!=OFF) pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(40));
        } else {
            switch (led_status) {
                 case ON:
                     pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(40));
                     break;
                 case DIM:
                     pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(10));
                     break;
                 default:
                     pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(0));
                     break;
            }
        }
        if (led_status == ALARM) {
            k_sleep(K_MSEC(100));
        } else {
            k_sleep(K_MSEC(500));
        }
            phase = !phase;
    }
}

_Noreturn int main(void)
{
    usb_enable(NULL);

    startup_device(pwm_motor0.dev);
    startup_device(pwm_motor1.dev);
    startup_device(pwm_led.dev);
    startup_device(as5600_dev);

    // <<< Create and start the LED task >>>
    k_tid_t led_tid = k_thread_create(&led_thread_data, led_stack_area,
                                      K_THREAD_STACK_SIZEOF(led_stack_area),
                                      led_task_entry, NULL, NULL, NULL,
                                      LED_PRIORITY, 0, K_NO_WAIT);
    if (!led_tid) {
        alarm("Failed to create LED thread!\n");
    } else {
         k_thread_name_set(led_tid, "led_task");
    }


    //todo remove test
    motor_pwm(100);
    k_sleep(K_MSEC(4000));
    motor_pwm(30);
    k_sleep(K_MSEC(4000));
    motor_pwm(-30);
    k_sleep(K_MSEC(4000));
    motor_pwm(-100);
    k_sleep(K_MSEC(4000));
    motor_pwm(10);


    while (1) {

        static struct sensor_value angle_val;
        int ret = sensor_sample_fetch(as5600_dev);

        if (ret != 0) {
            printk("Failed to fetch sample from AS5600: %d\n", ret);
            atomic_set(&led_status, ALARM);
        } else {
            ret = sensor_channel_get(as5600_dev, SENSOR_CHAN_ROTATION, &angle_val);
            if (ret < 0) {
                printk("Failed to get angle data from AS5600: %d\n", ret);
                atomic_set(&led_status, ALARM);
            } else {
                int angle_degrees = angle_val.val1;
                printk("AS5600 Angle: %d degrees\n", angle_degrees);
            }
        }

        k_sleep(K_MSEC(1000));
    }

}

