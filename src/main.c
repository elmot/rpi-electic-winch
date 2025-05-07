#include "main.h"
#include <zephyr/sys/printk.h>

#include "zephyr/drivers/i2c.h"
#include "zephyr/settings/settings.h"

//todo algorithms
//todo safe start

//todo improve PHASE I battery watching
//todo improve PHASE II motor current alarm

struct params_t params = {
    .max_angle_degree = 40,
    .min_pwm_percent = 10,
    .center_angle_degree = -1,
    .dead_angle_degree = 5
};

atomic_t led_status = ATOMIC_INIT(OFF);

const struct pwm_dt_spec pwm_motor0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor0)); // NOLINT(*-interfaces-global-init)
const struct pwm_dt_spec pwm_motor1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_motor1)); // NOLINT(*-interfaces-global-init)
const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led)); // NOLINT(*-interfaces-global-init)

K_THREAD_STACK_DEFINE(led_stack_area, TASK_STACK_SIZE);
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

static int old_motor_duty = 10000000;
atomic_t motor_suspended = ATOMIC_INIT(false);

/** Set motor PWM
 *
 * @param duty PWM duty, [-100..100]
 * @param forced ignore motor suspend flag for sake of setup
 */
void motor_pwm(int duty, bool forced)
{
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;
    if (motor_suspended && !forced) {
        old_motor_duty = duty;
        return;
    }
    if (old_motor_duty == duty && !forced) return;
    old_motor_duty = duty;
    int ret;
    enum led_status_type new_status;
    if (duty > params.min_pwm_percent) {
        ret = pwm_set_dt(&pwm_motor0, PWM_USEC(40), PWM_USEC(40 * duty / 100));
        ret |= pwm_set_dt(&pwm_motor1, PWM_USEC(40), PWM_USEC(0));
        new_status = duty >= 99 ? ON : DIM;
    } else if (duty < -params.min_pwm_percent) {
        ret = pwm_set_dt(&pwm_motor0, PWM_USEC(40), PWM_USEC(0));
        ret |= pwm_set_dt(&pwm_motor1, PWM_USEC(40), PWM_USEC(-40 * duty / 100));
        new_status = duty <= -99 ? ON : DIM;
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

void motor_pause(bool suspend)
{
    atomic_set(&motor_suspended, suspend);
    if (suspend) {
        motor_pwm(0, true);
        atomic_set(&led_status, SUSPEND);
    } else {
        atomic_set(&led_status, OFF);
        if (old_motor_duty >= -100 && old_motor_duty <= 100)
            motor_pwm(old_motor_duty, true);
    }
}


_Noreturn void alarm(const char* fmt, ...)
{
    motor_pwm(0, true);
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
            if (led_status != OFF) pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(40));
        } else {
            switch (led_status) {
            case ON:
                pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(40));
                break;
            case DIM:
            case SUSPEND:
                pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(10));
                break;
            default:
                pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(0));
            }
        }
        switch (led_status) {
        case ALARM:
            k_sleep(K_MSEC(100));
            break;
        case SUSPEND:
            k_sleep(K_SECONDS(1));
            break;
        default:
            k_sleep(K_MSEC(500));
        }
        phase = !phase;
    }
}


_Noreturn int main(void)
{
    startup_device(pwm_motor0.dev);
    startup_device(pwm_motor1.dev);
    startup_device(pwm_led.dev);
    // Create and start the LED task
    k_tid_t led_tid = k_thread_create(&led_thread_data, led_stack_area,
                                      K_THREAD_STACK_SIZEOF(led_stack_area),
                                      led_task_entry, NULL, NULL, NULL,
                                      TASK_PRIORITY, 0, K_NO_WAIT);
    if (!led_tid) {
        alarm("Failed to create LED thread!\n");
    } else {
        k_thread_name_set(led_tid, "led_task");
    }

    loadParameters();
    startSensorThread();
    //todo remove test

    atomic_set(&led_status, ALARM);
    k_sleep(K_SECONDS(3));

    motor_pwm(100, false);
    k_sleep(K_MSEC(4000));
    motor_pwm(30, false);
    k_sleep(K_MSEC(4000));
    motor_pwm(-30, false);
    k_sleep(K_MSEC(4000));
    motor_pwm(-100, false);
    k_sleep(K_MSEC(4000));
    motor_pwm(10, false);


    while (1) {
        printk("AS5600 Angle: %d degrees\n", (int)sampled_angle_degree);
        k_sleep(K_MSEC(1000));
    }
}
