#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

static int cmd_app_status(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "Vehicle telematics firmware");
    shell_print(sh, "Status: OK");
    shell_print(sh, "Uptime: %lld s",
                (long long)(k_uptime_get() / 1000));

    return 0;
}

static int cmd_app_ping(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "pong");

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_app,
    SHELL_CMD(status, NULL, "Show firmware status.", cmd_app_status),
    SHELL_CMD(ping, NULL, "Test shell communication.", cmd_app_ping),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(
    app,
    &sub_app,
    "Vehicle telematics application commands.",
    NULL
);