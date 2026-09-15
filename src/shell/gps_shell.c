#include "gnss/gnss_service.h"

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

static const char *fix_status_name(enum gnss_fix_status status)
{
    switch (status) {
    case GNSS_FIX_STATUS_NO_FIX:
        return "no fix";

    case GNSS_FIX_STATUS_GNSS_FIX:
        return "GNSS fix";

    case GNSS_FIX_STATUS_DGNSS_FIX:
        return "DGNSS fix";

    case GNSS_FIX_STATUS_ESTIMATED_FIX:
        return "estimated fix";

    default:
        return "unknown";
    }
}

static const char *system_name(enum gnss_system system)
{
    switch (system) {
    case GNSS_SYSTEM_GPS:
        return "GPS";

    case GNSS_SYSTEM_GLONASS:
        return "GLONASS";

    case GNSS_SYSTEM_GALILEO:
        return "Galileo";

    case GNSS_SYSTEM_BEIDOU:
        return "BeiDou";

    case GNSS_SYSTEM_QZSS:
        return "QZSS";

    case GNSS_SYSTEM_SBAS:
        return "SBAS";

    default:
        return "Other";
    }
}

static void print_coordinate(
    const struct shell *sh,
    const char *name,
    int64_t value)
{
    bool negative = value < 0;
    uint64_t absolute =
        negative ? (uint64_t)-value : (uint64_t)value;

    shell_print(
        sh,
        "%s: %s%llu.%09llu",
        name,
        negative ? "-" : "",
        absolute / 1000000000ULL,
        absolute % 1000000000ULL
    );
}

static int cmd_gps_status(
    const struct shell *sh,
    size_t argc,
    char **argv)
{
    shell_print(
        sh,
        "Driver: %s",
        gnss_service_is_ready() ? "ready" : "not ready"
    );

    bool awake;

    if (gnss_service_get_awake(&awake) == 0) {
        shell_print(
            sh,
            "Power: %s",
            awake ? "awake" : "sleeping"
        );
    }

    struct gnss_data data;

    if (!gnss_service_get_data(&data)) {
        shell_print(sh, "Data: not received");
        return 0;
    }

    shell_print(sh, "Data: receiving");
    shell_print(
        sh,
        "Fix: %s",
        fix_status_name(data.info.fix_status)
    );

    shell_print(
        sh,
        "Satellites tracked: %u",
        data.info.satellites_cnt
    );

    shell_print(
        sh,
        "HDOP: %u.%03u",
        data.info.hdop / 1000,
        data.info.hdop % 1000
    );

    int64_t last_update =
        gnss_service_last_update();

    shell_print(
        sh,
        "Last update: %lld ms ago",
        k_uptime_get() - last_update
    );

    return 0;
}

static int cmd_gps_fix(
    const struct shell *sh,
    size_t argc,
    char **argv)
{
    struct gnss_data data;

    if (!gnss_service_get_data(&data)) {
        shell_error(sh, "No GNSS data received yet");
        return 0;
    }

    shell_print(
        sh,
        "Fix: %s",
        fix_status_name(data.info.fix_status)
    );

    if (data.info.fix_status ==
        GNSS_FIX_STATUS_NO_FIX) {
        return 0;
    }

    print_coordinate(
        sh,
        "Latitude",
        data.nav_data.latitude
    );

    print_coordinate(
        sh,
        "Longitude",
        data.nav_data.longitude
    );

    shell_print(
        sh,
        "Altitude: %d.%03d m",
        data.nav_data.altitude / 1000,
        abs(data.nav_data.altitude % 1000)
    );

    shell_print(
        sh,
        "Speed: %u.%03u m/s",
        data.nav_data.speed / 1000,
        data.nav_data.speed % 1000
    );

    shell_print(
        sh,
        "Bearing: %u.%03u deg",
        data.nav_data.bearing / 1000,
        data.nav_data.bearing % 1000
    );

    shell_print(
        sh,
        "UTC: %02u:%02u:%02u.%03u",
        data.utc.hour,
        data.utc.minute,
        data.utc.millisecond / 1000,
        data.utc.millisecond % 1000
    );

    return 0;
}

static int cmd_gps_satellites(
    const struct shell *sh,
    size_t argc,
    char **argv)
{
    struct gnss_satellite satellites[
        GNSS_SERVICE_MAX_SATELLITES
    ];

    uint16_t count =
        gnss_service_get_satellites(
            satellites,
            ARRAY_SIZE(satellites)
        );

    if (count == 0) {
        shell_print(
            sh,
            "No satellite information received yet"
        );

        return 0;
    }

    shell_print(
        sh,
        "Satellites: %u",
        count
    );

    for (uint16_t i = 0; i < count; i++) {
        shell_print(
            sh,
            "%2u: %-8s PRN=%3u SNR=%2u dB "
            "EL=%2u AZ=%3u tracked=%s",
            i + 1,
            system_name(satellites[i].system),
            satellites[i].prn,
            satellites[i].snr,
            satellites[i].elevation,
            satellites[i].azimuth,
            satellites[i].is_tracked ? "yes" : "no"
        );
    }

    return 0;
}

static int cmd_gps_wake(
    const struct shell *sh,
    size_t argc,
    char **argv)
{
    int ret = gnss_service_wake();

    if (ret < 0) {
        shell_error(
            sh,
            "Failed to wake GPS (%d)",
            ret
        );

        return ret;
    }

    shell_print(sh, "GPS is awake");

    return 0;
}

static int cmd_gps_sleep(
    const struct shell *sh,
    size_t argc,
    char **argv)
{
    int ret = gnss_service_sleep();

    if (ret < 0) {
        shell_error(
            sh,
            "Failed to put GPS to sleep (%d)",
            ret
        );

        return ret;
    }

    shell_print(sh, "GPS is sleeping");

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_gps,

    SHELL_CMD(
        status,
        NULL,
        "Show GPS status.",
        cmd_gps_status
    ),

    SHELL_CMD(
        fix,
        NULL,
        "Show current position and navigation data.",
        cmd_gps_fix
    ),

    SHELL_CMD(
        satellites,
        NULL,
        "Show visible satellite information.",
        cmd_gps_satellites
    ),

    SHELL_CMD(
        wake,
        NULL,
        "Wake the GPS module.",
        cmd_gps_wake
    ),

    SHELL_CMD(
        sleep,
        NULL,
        "Put the GPS module into hibernate.",
        cmd_gps_sleep
    ),

    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(
    gps,
    &sub_gps,
    "GPS diagnostic commands.",
    NULL
);