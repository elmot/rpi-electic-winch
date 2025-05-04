#include <zephyr/shell/shell.h>

static int cmd_print(const struct shell* sh, size_t argc, char** argv)
{
    shell_print(sh, "Print command executed");
    return 0;
}

SHELL_CMD_REGISTER(print, NULL, "Print command", cmd_print);
