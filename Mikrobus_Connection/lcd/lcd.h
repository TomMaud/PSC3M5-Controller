#ifndef LCD_H
#define LCD_H
#include <stdint.h>
#include <stdbool.h>

void lcd_init(void);
void lcd_draw_dashboard(
    float rpm,
    float requested_rpm,
    float iu, float iv, float iw,
    float vu, float vv, float vw,
    int state,
    int enabled,
    bool rtc_valid,
    int rtc_hour,
    int rtc_minute,
    int rtc_second,
    uint32_t faults,
    bool connected);



#endif // LCD_H