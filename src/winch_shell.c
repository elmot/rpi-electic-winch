#include <stdlib.h>
#include <hardware/resets.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>

#include "main.h"
#include "zephyr/fs/nvs.h"
#include "zephyr/settings/settings.h"
#ifdef CONFIG_SOC_SERIES_RP2XXX
#include <pico/bootrom.h>
#endif

#define PARAMETERS_SIZE sizeof(struct params_t)

int params_handle_get(const char* key, char* val, int val_len_max)
{
    if (key != NULL && *key != 0) {
        return -EINVAL;
    }
    if (val_len_max < PARAMETERS_SIZE) {
        return -ENOMEM;
    }
    memcpy(val, &params, PARAMETERS_SIZE);
    return PARAMETERS_SIZE;
}

/**< Set value handler of settings items identified by keyword names.
 *
 * Parameters:
 *  - key[in] the name with skipped part that was used as name in
 *    handler registration
 *  - len[in] the size of the data found in the backend.
 *  - read_cb[in] function provided to read the data from the backend.
 *  - cb_arg[in] arguments for the read function provided by the
 *    backend.
 *
 * Return: 0 on success, non-zero on failure.
 */
int params_handle_set(const char* key, size_t len, settings_read_cb read_cb,
                      void* cb_arg)
{
    if (key != NULL && *key != 0) {
        return -EINVAL;
    }
    if (len < PARAMETERS_SIZE) {
        return -ENOMEM;
    }
    const int res = read_cb(cb_arg, &params, PARAMETERS_SIZE);
    if (res == PARAMETERS_SIZE) return 0;
    return res;
}

SETTINGS_STATIC_HANDLER_DEFINE(winch, "parameters", params_handle_get, params_handle_set, NULL, NULL);

void loadParameters()
{
    int res = settings_subsys_init();
    if (res) {
        alarm("Failed to initialize settings subsystem: %d", res);
    }
    settings_load();
}

static int cmd_reboot(const struct shell* sh, size_t argc, char** argv)
{
    shell_print(sh, "Rebooting...");
    k_sleep(K_SECONDS(1));
    sys_reboot(SYS_REBOOT_COLD);
}

static int parse_int(const struct shell* sh, size_t argc, char** argv, int max_value, const char* usage)
{
    if (argc != 2) {
        shell_print(sh, "%s", usage);
        return -1;
    }
    char* end_ptr;
    int val = strtol(argv[1], &end_ptr, 10);
    if (*end_ptr != '\0' || val > max_value) {
        shell_print(sh, "%s", usage);
        return -1;
    }
    return val;
}

static int cmd_joystick(const struct shell* sh, size_t argc, char** argv)
{
    int repeat;
    if (argc == 1) {
        repeat = 1;
    } else {
        repeat = parse_int(sh, argc, argv, 1000, "Usage: joystick [repeat]\n");
        if (repeat < 0) return 1;
    }
    shell_print(sh, "Reading joystick:\n");
    for (int i = 0; i < repeat; i++) {
        shell_print(sh, "%d: %d degrees", i + 1, (int)sampled_angle_degree);
        if (i < repeat - 1) {
            k_sleep(K_SECONDS(1));
        }
    }
    return 0;
}

static int cmd_params(const struct shell* sh,__unused size_t argc,__unused char** argv)
{
    shell_print(sh, "Angle center %d; dead %d; max %d",
                params.center_angle_degree,
                params.dead_angle_degree,
                params.max_angle_degree);
    shell_print(sh, "PWM min: %d%%", params.min_pwm_percent);
    return 0;
}

static int cmd_pause(const struct shell* sh, __unused size_t argc, __unused char** argv)
{
    shell_print(sh, "Motor control paused");
    motor_pause(true);
    cmd_params(sh, 0,NULL);
    return 0;
}

static int cmd_resume(const struct shell* sh,__unused size_t argc,__unused char** argv)
{
    shell_print(sh, "Motor control resumed");
    motor_pause(false);
    cmd_params(sh, 0,NULL);
    return 0;
}

static int save_params(const struct shell* sh)
{
    int result = settings_save_one("parameters", &params, PARAMETERS_SIZE);
    if (result)
        shell_print(sh, "Failed to save settings: %d", result);
    result = settings_commit();
    if (result) printk("Failed to commit: %d", result);
    return result;
}

static int cmd_set_min_pwm(const struct shell* sh, size_t argc, char** argv)
{
    int val = parse_int(sh, argc, argv, 99, "Usage: set_min_pwm [percent]\n");
    if (val < 0) return 1;
    val = val % 100;
    params.min_pwm_percent = val;
    cmd_pause(sh, 0,NULL);
    k_sleep(K_MSEC(200));
    shell_print(sh, "Testing motor forward...");
    motor_pwm(params.min_pwm_percent, true);
    k_sleep(K_MSEC(1500));
    motor_pwm(0, true);
    k_sleep(K_MSEC(500));
    shell_print(sh, "Testing motor reverse...");
    motor_pwm(-params.min_pwm_percent, true);
    k_sleep(K_MSEC(1500));
    motor_pwm(0, true);

    cmd_params(sh, 0,NULL);
    return save_params(sh);
}

static int cmd_set_dead_angle(const struct shell* sh, size_t argc, char** argv)
{
    int val = parse_int(sh, argc, argv, 179, "Usage: set_dead_angle [degree]\n");
    if (val < 0) return 1;
    params.dead_angle_degree = val % 180;
    cmd_params(sh, 0,NULL);
    return save_params(sh);
}

static int cmd_set_max_angle(const struct shell* sh, size_t argc, char** argv)
{
    int val = parse_int(sh, argc, argv, 179, "Usage: set_dead_angle [degree]\n");
    if (val < 0) return 1;
    params.max_angle_degree = val % 180;
    cmd_params(sh, 0,NULL);
    return save_params(sh);
}

static int cmd_center_joystick(const struct shell* sh, size_t argc, char** argv)
{
    int val;
    if (argc == 1) {
        val = sampled_angle_degree;
        shell_print(sh, "Reading joystick position: %d", val);
    } else {
        val = parse_int(sh, argc, argv, 359, "Usage: center_joystick [angle]\n");
        if (val < 0) return 1;
    }

    params.center_angle_degree = val % 360;
    cmd_params(sh, 0,NULL);
    return save_params(sh);
}


SHELL_CMD_REGISTER(reboot, NULL, "Reboot MCU", cmd_reboot); // NOLINT(*-branch-clone)

#ifdef CONFIG_SOC_SERIES_RP2XXX
static int cmd_bootloader(const struct shell* sh, size_t argc, char** argv)
{
#ifdef CONFIG_BOARD_RPI_PICO
#define BOOTLOADER_LED_MASK (1 << 25)
#else
#define BOOTLOADER_LED_MASK (0)
#endif
    shell_print(sh, "Starting bootloader...");
    k_sleep(K_SECONDS(1));
    reset_usb_boot(BOOTLOADER_LED_MASK, 0);
}

SHELL_CMD_REGISTER(bootloader, NULL, "Reboot MCU to bootloader", cmd_bootloader); // NOLINT(*-branch-clone)
#endif

SHELL_CMD_REGISTER(pause, NULL, "Suspend normal operation to setup parameters", cmd_pause); // NOLINT(*-branch-clone)
SHELL_CMD_REGISTER(resume, NULL, "Resume normal operation", cmd_resume); // NOLINT(*-branch-clone)
SHELL_CMD_REGISTER(params, NULL, "Print current params", cmd_params); // NOLINT(*-branch-clone)
SHELL_CMD_ARG_REGISTER(joystick, NULL, "Read joystick angle", cmd_joystick, 0, 1); // NOLINT(*-branch-clone)

SHELL_STATIC_SUBCMD_SET_CREATE(sub_winch_position, // NOLINT(*-branch-clone)
                               SHELL_CMD(center, NULL, "Set joystick central position [0..359]", cmd_center_joystick),
                               SHELL_CMD(pwm_min, NULL, "Set minimal pwm [0..99]", cmd_set_min_pwm),
                               SHELL_CMD(dead_angle, NULL, "Set joystick dead angle", cmd_set_dead_angle),
                               SHELL_CMD(max_angle, NULL, "Set joystick max angle", cmd_set_max_angle),
                               SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(set, &sub_winch_position, "Set parameters", NULL); // NOLINT(*-branch-clone)
