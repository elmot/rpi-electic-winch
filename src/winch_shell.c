#include <stdlib.h>
#include <hardware/resets.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>

#include "main.h"
#include "../../../../sdks/zephyrproject/zephyr/subsys/shell/shell_ops.h"


static int cmd_reboot(const struct shell* sh, size_t argc, char** argv)
{
    shell_print(sh, "Rebooting...\n");
    k_sleep(K_SECONDS(1));
    sys_reboot(SYS_REBOOT_COLD);
}

static int cmd_joystick(const struct shell* sh, size_t argc, char** argv)
{
    int repeat;
    switch (argc) {
        case 1:
            repeat = 1;
            break;
        case 2:
            repeat = atoi(argv[1]);
            break;
        default:
            shell_print(sh, "Usage: joystick [repeat]\n");
            return 1;
    }
    shell_print(sh, "Reading joystick:\n");
    for (int i = 0; i < repeat; i++) {
        static struct sensor_value angle_val;
        int ret = sensor_sample_fetch(as5600_dev);

        if (ret != 0) {
            shell_print(sh, "Failed to fetch sample from AS5600: %d", ret);
        } else {
            ret = sensor_channel_get(as5600_dev, SENSOR_CHAN_ROTATION, &angle_val);
            if (ret < 0) {
                shell_print(sh, "Failed to get angle data from AS5600: %d", ret);
            } else {
                int angle_degrees = angle_val.val1;
                
                shell_print(sh, "%d: %d degrees", i + 1, angle_degrees);
            }
        }
        if (i < repeat - 1) {
            k_sleep(K_SECONDS(1));
        }
    }
    return 0;
}

static int cmd_pause(const struct shell* sh, __unused size_t argc, __unused char** argv)
{
    shell_print(sh, "Motor control paused");
    atomic_set(&motor_suspended, true);
    return 0;
}

static int cmd_resume(const struct shell* sh,__unused size_t argc,__unused char** argv)
{
    shell_print(sh, "Motor control resumed");
    atomic_set(&motor_suspended, false);
    return 0;
}

static int cmd_params(const struct shell* sh,__unused size_t argc,__unused char** argv)
{
    shell_print(sh, "Motor control resumed");
    atomic_set(&motor_suspended, false);
    return 0;
}

static int cmd_set_min_pwm(const struct shell* sh, size_t argc, char** argv)
{
    //TODO
    return 0;
}

static int cmd_set_max_pwm(const struct shell* sh, size_t argc, char** argv)
{
    //TODO
    return 0;
}

static int cmd_set_dead_angle(const struct shell* sh, size_t argc, char** argv)
{
    //TODO
    return 0;
}

static int cmd_center_joystick(const struct shell* sh, size_t argc, char** argv)
{
    //TODO
    return 0;
}


SHELL_CMD_REGISTER(reboot, NULL, "Reboot MCU", cmd_reboot);
SHELL_CMD_REGISTER(pause, NULL, "Suspend normal operation to setup parameters", cmd_pause);
SHELL_CMD_REGISTER(resume, NULL, "Resume normal operation", cmd_resume);
SHELL_CMD_REGISTER(params, NULL, "Print current params", cmd_params);
SHELL_CMD_ARG_REGISTER(joystick, NULL, "Read joystick angle", cmd_joystick, 0, 1);
SHELL_CMD_ARG_REGISTER(center_joystick, NULL, "Set joystick central position [0..359]", cmd_center_joystick, 0, 1);
SHELL_CMD_ARG_REGISTER(set_min_pwm, NULL, "Set minimal pwm [0..99]", cmd_set_min_pwm, 0, 1);
SHELL_CMD_ARG_REGISTER(set_max_pwm, NULL, "Set max pwm [1..100]", cmd_set_max_pwm, 0, 1);
SHELL_CMD_ARG_REGISTER(set_dead_angle, NULL, "Set joystick dead angle", cmd_set_dead_angle, 0, 1);
