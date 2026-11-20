
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// RTC values received from the Infineon controller
int rtc_hour = 0;
int rtc_minute = 0;
int rtc_second = 0;

int rtc_day = 0;
int rtc_month = 0;
int rtc_year = 0;

bool rtc_valid = false;

bool telemetry_parse_rtc(const char *line)
{
    int h, m, s, d, mo, y;

    if (sscanf(
        line,
        "TIME:%d:%d:%d,%d/%d/%d",
        &h, &m, &s,
        &d, &mo, &y
    ) != 6)
    {
        return false;
    }

    if (h < 0 || h > 23 ||
        m < 0 || m > 59 ||
        s < 0 || s > 59 ||
        d < 1 || d > 31 ||
        mo < 1 || mo > 12 ||
        y < 0 || y > 9999)
    {
        return false;
    }

    rtc_hour = h;
    rtc_minute = m;
    rtc_second = s;

    rtc_day = d;
    rtc_month = mo;
    rtc_year = y;

    rtc_valid = true;

    return true;
}

static bool get_float(
    const char *line,
    const char *label,
    float *value)
{
    const char *position = strstr(line, label);

    if (position == NULL)
        return false;

    position = strchr(position, ':');

    if (position == NULL)
        return false;

    position++;

    char *end;
    float result = strtof(position, &end);

    if (end == position)
        return false;

    *value = result;
    return true;
}

static bool get_integer(
    const char *line,
    const char *label,
    int *value)
{
    const char *position = strstr(line, label);

    if (position == NULL)
        return false;

    position = strchr(position, ':');

    if (position == NULL)
        return false;

    position++;

    char *end;
    long result = strtol(position, &end, 10);

    if (end == position)
        return false;

    *value = (int)result;
    return true;
}

static bool get_uint32(
    const char *line,
    const char *label,
    uint32_t *value,
    int base)
{
    const char *position = strstr(line, label);

    if (position == NULL)
        return false;

    position = strchr(position, ':');

    if (position == NULL)
        return false;

    position++;

    char *end;
    unsigned long result = strtoul(position, &end, base);

    if (end == position)
        return false;

    *value = (uint32_t)result;
    return true;
}

static bool get_current_amps(
    const char *line,
    const char *label,
    float *value)
{
    int current_ma;

    if (!get_integer(line, label, &current_ma))
        return false;

    *value = (float)current_ma / 1000.0f;
    return true;
}



bool telemetry_parse_line(
    const char *line,
    float *rpm,
    float *requested_rpm,
    float *iu,
    float *iv,
    float *iw,
    unsigned int *position,
    uint32_t *time_ms,
    int *state,
    int *enabled,
    uint32_t *faults)
{
    if (line == NULL)
        return false;

    // Ignore RTC messages
    if (strncmp(line, "TIME:", 5) == 0)
        return false;

    // Only process the new telemetry format
    if (strstr(line, "[Telemetry]") == NULL)
        return false;

    bool valid = false;

    // Actual speed is already transmitted in RPM
    if (get_float(line, "Speed", rpm))
        valid = true;

    // Requested speed in RPM
    get_float(line, "ReqRPM", requested_rpm);

    // Phase currents arrive in milliamps and are stored in amps
    get_current_amps(line, "Iu", iu);
    get_current_amps(line, "Iv", iv);
    get_current_amps(line, "Iw", iw);

    // Hall sensor state
    int hall_value;

    if (get_integer(line, "Hall", &hall_value) &&
        hall_value >= 0 && hall_value <= 7)
    {
        *position = (unsigned int)hall_value;
    }

    // PSoC timestamp in milliseconds
    get_uint32(line, "Time", time_ms, 10);

    // Motor state and enable status
    get_integer(line, "State", state);
    get_integer(line, "En", enabled);

    // Fault flags are hexadecimal
    get_uint32(line, "Faults", faults, 0);

    return valid;
}
