#include "services/can_decoder.h"

#include <errno.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#define VEHICLE_WATCH_DEFAULT_PERIOD_MS 1000U
#define VEHICLE_WATCH_MIN_PERIOD_MS      100U
#define VEHICLE_WATCH_MAX_PERIOD_MS      60000U
#define VEHICLE_SIGNAL_STALE_AFTER_MS    2000

static const char *freshness_suffix(int64_t age_ms)
{
    return age_ms > VEHICLE_SIGNAL_STALE_AFTER_MS ? ", stale" : "";
}

static int64_t signal_age_ms(int64_t now_ms, int64_t updated_at_ms)
{
    return now_ms >= updated_at_ms ? now_ms - updated_at_ms : 0;
}

static int print_vehicle_state(const struct shell *sh)
{
    struct vehicle_state state;
    int ret = can_decoder_get_vehicle_state(&state);

    if (ret != 0) {
        shell_error(sh, "Vehicle state unavailable (%d)", ret);
        return ret;
    }

    int64_t now_ms = k_uptime_get();

    if ((state.valid_signals & VEHICLE_STATE_SPEED_VALID) != 0U) {
        int64_t age_ms = signal_age_ms(now_ms, state.speed_updated_at_ms);

        shell_print(sh, "Speed: %.2f km/h (%lld ms ago%s)",
                    (double)state.speed_kph, (long long)age_ms,
                    freshness_suffix(age_ms));
    } else {
        shell_print(sh, "Speed: not received");
    }

    if ((state.valid_signals & VEHICLE_STATE_ENGINE_RPM_VALID) != 0U) {
        int64_t age_ms = signal_age_ms(now_ms, state.engine_rpm_updated_at_ms);

        shell_print(sh, "RPM: %.0f (%lld ms ago%s)",
                    (double)state.engine_rpm, (long long)age_ms,
                    freshness_suffix(age_ms));
    } else {
        shell_print(sh, "RPM: not received");
    }

    if ((state.valid_signals &
         VEHICLE_STATE_THROTTLE_POSITION_VALID) != 0U) {
        int64_t age_ms = signal_age_ms(
            now_ms, state.throttle_position_updated_at_ms);

        shell_print(sh, "Throttle: %.1f %% (%lld ms ago%s)",
                    (double)state.throttle_position_pct,
                    (long long)age_ms, freshness_suffix(age_ms));
    } else {
        shell_print(sh, "Throttle: not received");
    }

    shell_print(sh, "Signal updates: %u", state.update_count);
    return 0;
}

static int cmd_vehicle_status(
    const struct shell *sh,
    size_t argc,
    char **argv
)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    return print_vehicle_state(sh);
}

static int parse_watch_period(
    const struct shell *sh,
    size_t argc,
    char **argv,
    uint32_t *period_ms
)
{
    *period_ms = VEHICLE_WATCH_DEFAULT_PERIOD_MS;

    if (argc < 2U) {
        return 0;
    }

    char *end;
    unsigned long value = strtoul(argv[1], &end, 10);

    if (*argv[1] == '\0' || *end != '\0' ||
        value < VEHICLE_WATCH_MIN_PERIOD_MS ||
        value > VEHICLE_WATCH_MAX_PERIOD_MS) {
        shell_error(sh, "Period must be between %u and %u ms",
                    VEHICLE_WATCH_MIN_PERIOD_MS,
                    VEHICLE_WATCH_MAX_PERIOD_MS);
        return -EINVAL;
    }

    *period_ms = (uint32_t)value;
    return 0;
}

static int cmd_vehicle_watch(
    const struct shell *sh,
    size_t argc,
    char **argv
)
{
    uint32_t period_ms;
    int ret = parse_watch_period(sh, argc, argv, &period_ms);

    if (ret != 0) {
        return ret;
    }

    shell_print(sh, "Watching every %u ms; press Ctrl-C or Enter to stop",
                period_ms);

    uint8_t input[2];

    while (true) {
        ret = print_vehicle_state(sh);

        if (ret != 0) {
            return ret;
        }

        ret = shell_readline(sh, input, sizeof(input), K_MSEC(period_ms));

        if (ret == -ETIMEDOUT) {
            continue;
        }

        if (ret != -ECANCELED && ret < 0) {
            shell_error(sh, "Watch input failed (%d)", ret);
            return ret;
        }

        shell_print(sh, "Vehicle watch stopped");
        return 0;
    }
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_vehicle,
    SHELL_CMD(status, NULL, "Show the latest decoded vehicle state.",
              cmd_vehicle_status),
    SHELL_CMD_ARG(watch, NULL,
                  "Continuously show state: watch [period_ms]",
                  cmd_vehicle_watch, 1, 1),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(
    vehicle,
    &sub_vehicle,
    "Decoded vehicle state commands.",
    NULL
);
