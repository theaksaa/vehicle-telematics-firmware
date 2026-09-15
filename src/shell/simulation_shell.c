#include "services/gnss_service.h"

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#define DEFAULT_SATELLITE_COUNT 10U

static int parse_double(
    const struct shell *sh,
    const char *text,
    const char *name,
    double minimum,
    double maximum,
    double *value)
{
    char *end;

    errno = 0;
    *value = strtod(text, &end);

    if (errno != 0 || text[0] == '\0' || *end != '\0' ||
        *value != *value || *value < minimum || *value > maximum) {
        shell_error(sh, "%s must be between %.2f and %.2f",
                    name, minimum, maximum);
        return -EINVAL;
    }

    return 0;
}

static int parse_satellite_count(
    const struct shell *sh,
    const char *text,
    uint16_t *count)
{
    char *end;
    unsigned long value;

    errno = 0;
    value = strtoul(text, &end, 10);

    if (errno != 0 || text[0] == '\0' || *end != '\0' ||
        value > GNSS_SERVICE_MAX_SATELLITES) {
        shell_error(sh, "satellites must be between 0 and %u",
                    GNSS_SERVICE_MAX_SATELLITES);
        return -EINVAL;
    }

    *count = (uint16_t)value;
    return 0;
}

static int cmd_sim_start(
    const struct shell *sh,
    size_t argc,
    char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    gnss_service_set_simulation_enabled(true);
    shell_print(sh, "GNSS simulation started; hardware fixes are paused");
    return 0;
}

static int cmd_sim_stop(
    const struct shell *sh,
    size_t argc,
    char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    gnss_service_set_simulation_enabled(false);
    shell_print(sh, "GNSS simulation stopped; hardware fixes resumed");
    return 0;
}

static int cmd_sim_status(
    const struct shell *sh,
    size_t argc,
    char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "GNSS simulation: %s",
                gnss_service_simulation_is_enabled() ? "running" : "stopped");
    shell_warn(sh, "Bench build: CAN controller is in normal (active) mode");
    return 0;
}

static void inject_satellites(uint16_t count)
{
    struct gnss_satellite satellites[GNSS_SERVICE_MAX_SATELLITES] = {0};

    for (uint16_t i = 0; i < count; ++i) {
        satellites[i].prn = (uint8_t)(i + 1U);
        satellites[i].snr = (uint8_t)(35U + (i % 10U));
        satellites[i].elevation = (uint8_t)(20U + ((i * 7U) % 65U));
        satellites[i].azimuth = (uint16_t)((i * 37U) % 360U);
        satellites[i].system = GNSS_SYSTEM_GPS;
        satellites[i].is_tracked = 1U;
    }

    (void)gnss_service_inject_satellites(satellites, count);
}

static int cmd_sim_gps(
    const struct shell *sh,
    size_t argc,
    char **argv)
{
    double latitude;
    double longitude;
    double altitude_m;
    double speed_kph;
    double bearing_deg;
    uint16_t satellite_count = DEFAULT_SATELLITE_COUNT;
    int ret;

    if (!gnss_service_simulation_is_enabled()) {
        shell_error(sh, "Run 'sim start' first");
        return -EACCES;
    }

    ret = parse_double(sh, argv[1], "latitude", -90.0, 90.0, &latitude);
    ret = ret != 0 ? ret : parse_double(
        sh, argv[2], "longitude", -180.0, 180.0, &longitude);
    ret = ret != 0 ? ret : parse_double(
        sh, argv[3], "altitude", -1000.0, 20000.0, &altitude_m);
    ret = ret != 0 ? ret : parse_double(
        sh, argv[4], "speed", 0.0, 1000.0, &speed_kph);
    ret = ret != 0 ? ret : parse_double(
        sh, argv[5], "bearing", 0.0, 360.0, &bearing_deg);

    if (ret != 0) {
        return ret;
    }

    if (argc == 7U) {
        ret = parse_satellite_count(sh, argv[6], &satellite_count);
        if (ret != 0) {
            return ret;
        }
    }

    struct gnss_data data = {0};

    data.nav_data.latitude = (int64_t)(latitude * 1000000000.0);
    data.nav_data.longitude = (int64_t)(longitude * 1000000000.0);
    data.nav_data.altitude = (int32_t)(altitude_m * 1000.0);
    data.nav_data.speed = (uint32_t)(speed_kph * (1000000.0 / 3600.0));
    data.nav_data.bearing = (uint32_t)(bearing_deg * 1000.0);
    data.info.satellites_cnt = satellite_count;
    data.info.hdop = 900U;
    data.info.fix_status = GNSS_FIX_STATUS_GNSS_FIX;
    data.info.fix_quality = GNSS_FIX_QUALITY_GNSS_SPS;

    ret = gnss_service_inject_data(&data);
    if (ret == 0) {
        inject_satellites(satellite_count);
    }

    return ret;
}

static int cmd_sim_nofix(
    const struct shell *sh,
    size_t argc,
    char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!gnss_service_simulation_is_enabled()) {
        shell_error(sh, "Run 'sim start' first");
        return -EACCES;
    }

    struct gnss_data data = {0};

    data.info.fix_status = GNSS_FIX_STATUS_NO_FIX;
    data.info.fix_quality = GNSS_FIX_QUALITY_INVALID;

    int ret = gnss_service_inject_data(&data);

    if (ret == 0) {
        (void)gnss_service_inject_satellites(NULL, 0U);
    }

    return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_sim,
    SHELL_CMD(start, NULL, "Start GNSS simulation.", cmd_sim_start),
    SHELL_CMD(stop, NULL, "Stop simulation and resume real GNSS.", cmd_sim_stop),
    SHELL_CMD(status, NULL, "Show simulation status.", cmd_sim_status),
    SHELL_CMD_ARG(gps, NULL,
                  "Inject: gps <lat> <lon> <alt_m> <speed_kph> <bearing_deg> [sats]",
                  cmd_sim_gps, 6, 1),
    SHELL_CMD(nofix, NULL, "Inject loss of GNSS fix.", cmd_sim_nofix),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sim, &sub_sim, "Bench simulation commands.", NULL);
