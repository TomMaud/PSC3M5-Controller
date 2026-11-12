#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>


// Motor telemetry parser
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
    uint32_t *faults
);

// RTC values
extern int rtc_hour;
extern int rtc_minute;
extern int rtc_second;

extern int rtc_day;
extern int rtc_month;
extern int rtc_year;

extern bool rtc_valid;

// RTC parser
bool telemetry_parse_rtc(const char *line);

#endif // TELEMETRY_H