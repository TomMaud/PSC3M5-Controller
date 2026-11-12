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

    long result = strtol(position, &end, 0);

    if (end == position)
        return false;

    *value = (int)result;
    return true;
}

bool telemetry_parse_line(
    const char *line,
    float *rpm,
    float *requested_rpm,
    float *iu,
    float *iv,
    float *iw,
    float *vu,
    float *vv,
    float *vw,
    int *state,
    int *enabled,
    uint32_t *faults)
{
    if (telemetry_parse_rtc(line))
{
    return false;
}
    bool valid = false;

    float value;

    // Speed in RPM: find the second Speed field if present.
    const char *first_speed = strstr(line, "Speed:");
    const char *second_speed = NULL;

    if (first_speed != NULL)
        second_speed = strstr(first_speed + 6, "Speed:");

    if (second_speed != NULL)
    {
        char *end;
        value = strtof(strchr(second_speed, ':') + 1, &end);

        if (end != strchr(second_speed, ':') + 1)
        {
            *rpm = value;
            valid = true;
        }
    }
    else if (first_speed != NULL)
    {
        // Only rad/s is present: convert to RPM.
        char *end;
        const char *start = strchr(first_speed, ':') + 1;
        value = strtof(start, &end);

        if (end != start)
        {
            *rpm = value * 60.0f / 6.28318530718f;
            valid = true;
        }
    }
    // Requested motor speed in RPM
    if (get_float(line, "ReqRPM", requested_rpm))
    {
        valid = true;
}

    get_float(line, "I_U", iu);
    get_float(line, "I_V", iv);
    get_float(line, "I_W", iw);

    get_float(line, "V_U", vu);
    get_float(line, "V_V", vv);
    get_float(line, "V_W", vw);

    get_integer(line, "State", state);
    get_integer(line, "En", enabled);

    int fault_value;

    if (get_integer(line, "Faults", &fault_value)){
        *faults = (uint32_t)fault_value;
    }
    return valid;
}

