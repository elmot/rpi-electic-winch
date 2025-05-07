#include "main.h"

#include <stdlib.h>
#include <zephyr/sys/printk.h>

#include "zephyr/drivers/i2c.h"
#include "zephyr/settings/settings.h"

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

atomic_t motor_suspended = ATOMIC_INIT(false);
atomic_t report_motor_pwm = ATOMIC_INIT(0);

void motor_pwm(int duty, bool forced)
{
    static int old_motor_duty = 1000000;
    if (duty > 100) duty = 100;
    if (duty < -100) duty = -100;
    if (motor_suspended && !forced) {
        return;
    }
    if (old_motor_duty == duty && !forced) return;
    old_motor_duty = duty;
    int ret;
    if (duty >= params.min_pwm_percent) {
        ret = pwm_set_dt(&pwm_motor0, PWM_USEC(40), PWM_USEC(40 * duty / 100));
        ret |= pwm_set_dt(&pwm_motor1, PWM_USEC(40), PWM_USEC(0));
    } else if (duty <= -params.min_pwm_percent) {
        ret = pwm_set_dt(&pwm_motor0, PWM_USEC(40), PWM_USEC(0));
        ret |= pwm_set_dt(&pwm_motor1, PWM_USEC(40), PWM_USEC(-40 * duty / 100));
    } else {
        ret = pwm_set_dt(&pwm_motor0, PWM_USEC(40), PWM_USEC(0));
        ret |= pwm_set_dt(&pwm_motor1, PWM_USEC(40), PWM_USEC(0));
    }
    if (ret) {
        printk("Failed to set motor PWM: %d\n", ret);
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

int effectiveAngle()
{
    int angle = atomic_get(&sampled_angle_degree);
    angle = (360 + angle - params.center_angle_degree) % 360;
    if (angle > (360 - params.max_angle_degree)) {
        return angle - 360;
    }
    if (angle > 180 ) {
        return -params.max_angle_degree;
    }
    if (angle > params.max_angle_degree) {
        return params.max_angle_degree;
    }
    return angle;
}

_Noreturn void led_task_entry(__unused void* p1,__unused void* p2,__unused void* p3)
{
    static bool phase = false;
    for (int i = 1; i <= 10; i++) {
        pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(i * 10));
        k_sleep(K_MSEC(20));
    }
    for (int i = 1; i <= 5; i++) {
        pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(40));
        k_sleep(K_MSEC(150));
        pwm_set_dt(&pwm_led, PWM_USEC(40), PWM_USEC(0));
        k_sleep(K_MSEC(50));
    }
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
        int angle = (int)atomic_get(&sampled_angle_degree);
        printk("Angle sampled: %d, corrected: %d. PWM: %d\n",
               angle, effectiveAngle(),
               (int)atomic_get(&report_motor_pwm));
    }
}

_Noreturn void motor_loop()
{
    int desired_pwm = 0;
    static int current_pwm = 0;
    while (true) {
        if (atomic_get(&motor_suspended)) {
            k_sleep(K_MSEC(300));
            continue;
        }
        k_sleep(K_MSEC(2));
        int angle = atomic_get(&sampled_angle_degree);
        if (angle < 0) {
            atomic_set(&led_status, SUSPEND);
            motor_pwm(0, true);
            continue;
        }
        int effective_angle = effectiveAngle();
        const int abs_angle = abs(effective_angle) - params.dead_angle_degree;
        if (abs_angle <= 0) {
            desired_pwm = 0;
            atomic_set(&led_status, OFF);
            motor_pwm(0, true);
            continue;
        }
        desired_pwm = params.min_pwm_percent +
            (100 - params.min_pwm_percent) * abs_angle / (params.max_angle_degree - params.dead_angle_degree);
        if (effective_angle < 0) desired_pwm = -desired_pwm;
        if (desired_pwm >= 0) {
            if (current_pwm < 0) current_pwm = 0;
            if (current_pwm < desired_pwm) {
                current_pwm++;
            } else { current_pwm = desired_pwm; }
        } else {
            if (current_pwm > 0) current_pwm = 0;
            if (current_pwm > desired_pwm) {
                current_pwm--;
            } else { current_pwm = desired_pwm; }
        }
        switch (current_pwm) {
        case 0: atomic_set(&led_status, OFF);
            break;
        case 100:
        case 99:
        case -99:
        case -100: atomic_set(&led_status, ON);
            break;
        default: atomic_set(&led_status, DIM);
            break;
        }
        motor_pwm(current_pwm, false);
        atomic_set(&report_motor_pwm, current_pwm);
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

    motor_loop();
}
