#ifndef MAIN_H
#define MAIN_H

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "zephyr/drivers/sensor.h"

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
    OFF, ON, DIM, ALARM, SUSPEND
};

void motor_pause(bool suspend);

extern const struct pwm_dt_spec pwm_motor0;
extern const struct pwm_dt_spec pwm_motor1;
extern const struct pwm_dt_spec pwm_led;

extern const struct device* const as5600_dev;

extern atomic_t led_status;

__packed struct params_t
{
    uint16_t min_pwm_percent;
    uint16_t dead_angle_degree;
    int16_t center_angle_degree;
    uint16_t max_angle_degree;
};

extern struct params_t params;

void loadParameters();

_Noreturn void alarm(const char* fmt, ...);

#define STORAGE_PARTITION	storage_partition
#define STORAGE_PARTITION_ID	FIXED_PARTITION_ID(STORAGE_PARTITION)

#endif /* MAIN_H */
