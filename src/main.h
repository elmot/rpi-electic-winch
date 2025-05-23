#ifndef MAIN_H
#define MAIN_H

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

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

#define TASK_STACK_SIZE 1512
#define TASK_PRIORITY 5

enum led_status_type
{
    OFF, ON, DIM, ALARM, SUSPEND
};

void motor_pause(bool suspend);

extern const struct pwm_dt_spec pwm_motor0;
extern const struct pwm_dt_spec pwm_motor1;
extern const struct pwm_dt_spec pwm_led;

extern atomic_t led_status;

/** Set motor PWM
 *
 * @param duty PWM duty, [-100..100]
 * @param forced ignore motor suspend flag for sake of setup
 */
void motor_pwm(int duty, bool forced);

__packed struct params_t
{
    uint16_t min_pwm_percent;
    uint16_t dead_angle_degree;
    int16_t center_angle_degree;
    uint16_t max_angle_degree;
};

extern struct params_t params;
extern atomic_t sampled_angle_degree;

void loadParameters();
void startSensorThread();
void startup_device(const struct device* dev);

_Noreturn void alarm(const char* fmt, ...);

#define STORAGE_PARTITION	storage_partition
#define STORAGE_PARTITION_ID	FIXED_PARTITION_ID(STORAGE_PARTITION)

#endif /* MAIN_H */
