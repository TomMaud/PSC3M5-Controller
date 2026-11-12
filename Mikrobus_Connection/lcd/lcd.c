#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

#define SCREEN_WIDTH  240
#define SCREEN_HEIGHT 135

// Landscape: MADCTL = MX | MV | ML (0x70)
// The panel's RAM offsets swap round when the axes swap.
#define LCD_MADCTL 0x70
#define X_OFFSET   40
#define Y_OFFSET   53


#define PIN_DIN   19
#define PIN_CLK   18
#define PIN_CS    20
#define PIN_DC    16
#define PIN_RESET 17
#define PIN_BL    21

#define BLACK   0x0000
#define WHITE   0xFFFF
#define GREEN   0x07E0
#define RED     0xF800
#define CYAN    0x07FF
#define YELLOW  0xFFE0
#define GREY    0x7BEF
#define DARK    0x1082

static void lcd_command(uint8_t cmd)
{
    gpio_put(PIN_CS, 0);
    gpio_put(PIN_DC, 0);

    spi_write_blocking(spi0, &cmd, 1);

    gpio_put(PIN_CS, 1);
}

static void lcd_data(uint8_t data)
{
    gpio_put(PIN_CS, 0);
    gpio_put(PIN_DC, 1);

    spi_write_blocking(spi0, &data, 1);

    gpio_put(PIN_CS, 1);
}

static void lcd_data_buffer(const uint8_t *data, size_t length)
{
    gpio_put(PIN_CS, 0);
    gpio_put(PIN_DC, 1);

    spi_write_blocking(spi0, data, length);

    gpio_put(PIN_CS, 1);
}

static void lcd_set_window(
    uint16_t x0,
    uint16_t y0,
    uint16_t x1,
    uint16_t y1)
{
    x0 += X_OFFSET;
    x1 += X_OFFSET;

    y0 += Y_OFFSET;
    y1 += Y_OFFSET;

    lcd_command(0x2A);
    lcd_data(x0 >> 8);
    lcd_data(x0 & 0xFF);
    lcd_data(x1 >> 8);
    lcd_data(x1 & 0xFF);

    lcd_command(0x2B);
    lcd_data(y0 >> 8);
    lcd_data(y0 & 0xFF);
    lcd_data(y1 >> 8);
    lcd_data(y1 & 0xFF);

    lcd_command(0x2C);
}

static void lcd_fill_rect(
    int x,
    int y,
    int width,
    int height,
    uint16_t colour)
{
    if (x < 0 || y < 0 ||
        x + width > SCREEN_WIDTH ||
        y + height > SCREEN_HEIGHT ||
        width <= 0 || height <= 0)
    {
        return;
    }

    lcd_set_window(
        x,
        y,
        x + width - 1,
        y + height - 1
    );

    uint8_t row[SCREEN_WIDTH * 2];

    for (int i = 0; i < width; i++)
    {
        row[i * 2] = colour >> 8;
        row[i * 2 + 1] = colour & 0xFF;
    }

    gpio_put(PIN_CS, 0);
    gpio_put(PIN_DC, 1);

    for (int i = 0; i < height; i++)
    {
        spi_write_blocking(spi0, row, width * 2);
    }

    gpio_put(PIN_CS, 1);
}

static void lcd_fill(uint16_t colour)
{
    lcd_fill_rect(
        0,
        0,
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
        colour
    );
}

static void lcd_line(int x, int y, int width, uint16_t colour)
{
    lcd_fill_rect(x, y, width, 1, colour);
}

/*
 * 5x7 font.
 * Each byte represents one vertical column.
 * Bit 0 is the top pixel.
 */
static const uint8_t *font_get(char c)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};

    static const uint8_t digits[10][5] = {
        {0x3E,0x51,0x49,0x45,0x3E}, // 0
        {0x00,0x42,0x7F,0x40,0x00}, // 1
        {0x42,0x61,0x51,0x49,0x46}, // 2
        {0x21,0x41,0x45,0x4B,0x31}, // 3
        {0x18,0x14,0x12,0x7F,0x10}, // 4
        {0x27,0x45,0x45,0x45,0x39}, // 5
        {0x3C,0x4A,0x49,0x49,0x30}, // 6
        {0x01,0x71,0x09,0x05,0x03}, // 7
        {0x36,0x49,0x49,0x49,0x36}, // 8
        {0x06,0x49,0x49,0x29,0x1E}  // 9
    };

    static const uint8_t letters[26][5] = {
        {0x7E,0x11,0x11,0x11,0x7E}, // A
        {0x7F,0x49,0x49,0x49,0x36}, // B
        {0x3E,0x41,0x41,0x41,0x22}, // C
        {0x7F,0x41,0x41,0x22,0x1C}, // D
        {0x7F,0x49,0x49,0x49,0x41}, // E
        {0x7F,0x09,0x09,0x09,0x01}, // F
        {0x3E,0x41,0x49,0x49,0x7A}, // G
        {0x7F,0x08,0x08,0x08,0x7F}, // H
        {0x00,0x41,0x7F,0x41,0x00}, // I
        {0x20,0x40,0x41,0x3F,0x01}, // J
        {0x7F,0x08,0x14,0x22,0x41}, // K
        {0x7F,0x40,0x40,0x40,0x40}, // L
        {0x7F,0x02,0x0C,0x02,0x7F}, // M
        {0x7F,0x04,0x08,0x10,0x7F}, // N
        {0x3E,0x41,0x41,0x41,0x3E}, // O
        {0x7F,0x09,0x09,0x09,0x06}, // P
        {0x3E,0x41,0x51,0x21,0x5E}, // Q
        {0x7F,0x09,0x19,0x29,0x46}, // R
        {0x46,0x49,0x49,0x49,0x31}, // S
        {0x01,0x01,0x7F,0x01,0x01}, // T
        {0x3F,0x40,0x40,0x40,0x3F}, // U
        {0x1F,0x20,0x40,0x20,0x1F}, // V
        {0x3F,0x40,0x38,0x40,0x3F}, // W
        {0x63,0x14,0x08,0x14,0x63}, // X
        {0x07,0x08,0x70,0x08,0x07}, // Y
        {0x61,0x51,0x49,0x45,0x43}  // Z
    };

    if (c >= '0' && c <= '9')
        return digits[c - '0'];

    if (c >= 'A' && c <= 'Z')
        return letters[c - 'A'];

    static const uint8_t dot[5]   = {0,0,0x60,0x60,0};
    static const uint8_t colon[5] = {0,0x36,0x36,0,0};
    static const uint8_t dash[5]  = {0x08,0x08,0x08,0x08,0};
    static const uint8_t slash[5] = {0x20,0x10,0x08,0x04,0x02};
    static const uint8_t plus[5]  = {0x08,0x08,0x3E,0x08,0x08};
    static const uint8_t equal[5] = {0x14,0x14,0x14,0x14,0x14};

    switch (c)
    {
        case '.': return dot;
        case ':': return colon;
        case '-': return dash;
        case '/': return slash;
        case '+': return plus;
        case '=': return equal;
        default: return blank;
    }
}

static void lcd_draw_char(
    int x,
    int y,
    char c,
    uint16_t foreground,
    uint16_t background,
    int scale)
{
    if (scale < 1)
        return;

    c = (char)toupper((unsigned char)c);

    const uint8_t *glyph = font_get(c);

    uint8_t pixels[15 * 21 * 2];

    int width = 5 * scale;
    int height = 7 * scale;

    int position = 0;

    for (int py = 0; py < 7; py++)
    {
        for (int sy = 0; sy < scale; sy++)
        {
            for (int px = 0; px < 5; px++)
            {
                uint16_t colour =
                    (glyph[px] & (1 << py))
                    ? foreground
                    : background;

                for (int sx = 0; sx < scale; sx++)
                {
                    pixels[position++] = colour >> 8;
                    pixels[position++] = colour & 0xFF;
                }
            }
        }
    }

    lcd_set_window(
        x,
        y,
        x + width - 1,
        y + height - 1
    );

    lcd_data_buffer(pixels, position);
}

static void lcd_draw_text(
    int x,
    int y,
    const char *text,
    uint16_t foreground,
    uint16_t background,
    int scale)
{
    while (*text)
    {
        lcd_draw_char(
            x,
            y,
            *text,
            foreground,
            background,
            scale
        );

        x += 6 * scale;
        text++;
    }
}

static void lcd_draw_text_field(
    int x,
    int y,
    int width,
    int height,
    const char *text,
    uint16_t foreground,
    uint16_t background,
    int scale)
{
    // Clear only this text field, not the entire screen.
    lcd_fill_rect(x, y, width, height, background);

    // Draw the updated text.
    lcd_draw_text(
        x,
        y,
        text,
        foreground,
        background,
        scale
    );
}

void lcd_init(void)
{
    spi_init(spi0, 20 * 1000 * 1000);

    spi_set_format(
        spi0,
        8,
        SPI_CPOL_0,
        SPI_CPHA_0,
        SPI_MSB_FIRST
    );

    gpio_set_function(PIN_CLK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DIN, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_init(PIN_DC);
    gpio_set_dir(PIN_DC, GPIO_OUT);

    gpio_init(PIN_RESET);
    gpio_set_dir(PIN_RESET, GPIO_OUT);

    gpio_init(PIN_BL);
    gpio_set_dir(PIN_BL, GPIO_OUT);
    gpio_put(PIN_BL, 1);

    gpio_put(PIN_RESET, 1);
    sleep_ms(20);

    gpio_put(PIN_RESET, 0);
    sleep_ms(100);

    gpio_put(PIN_RESET, 1);
    sleep_ms(150);

    lcd_command(0x01);
    sleep_ms(150);

    lcd_command(0x11);
    sleep_ms(120);

    lcd_command(0x3A);
    lcd_data(0x55);

    lcd_command(0x36);
    lcd_data(LCD_MADCTL);

    lcd_command(0x21);
    lcd_command(0x13);

    lcd_command(0x29);
    sleep_ms(100);

    lcd_fill(BLACK);
}


void lcd_draw_dashboard(
    float rpm,
    float requested_rpm,  // NEW
    float iu,
    float iv,
    float iw,
    float vu,
    float vv,
    float vw,
    int state,
    int enabled,
    bool rtc_valid,
    int rtc_hour,
    int rtc_minute,
    int rtc_second,
    uint32_t faults,
    bool connected){

char text[32];

    /// Header
lcd_draw_text(4, 4, "MOTOR MONITOR", CYAN, BLACK, 1);

// RTC time between title and connection status
if (rtc_valid)
{
    char time_text[16];

    snprintf(
        time_text,
        sizeof(time_text),
        "%02d:%02d:%02d",
        rtc_hour,
        rtc_minute,
        rtc_second
    );

    lcd_draw_text_field(
        104, 4,
        84, 7,
        time_text,
        WHITE,
        BLACK,
        1
    );
}
else
{
    lcd_draw_text_field(
        104, 4,
        84, 7,
        "--:--:--",
        GREY,
        BLACK,
        1
    );
}

// Connection status
if (connected){
    lcd_draw_text_field(194, 4, 42, 7, "LINK OK", GREEN, BLACK, 1);}
else
{
    lcd_draw_text_field(194, 4, 42, 7, "NO LINK", RED, BLACK, 1);
}


// Column divider
    lcd_fill_rect(120, 20, 1, 110, GREY);

    // ---- Left column: speed and status ----
    lcd_draw_text(4, 22, "MOTOR SPEED", WHITE, BLACK, 1);

    snprintf(text, sizeof(text), "%.0f", rpm);
    lcd_draw_text_field(4, 36, 108, 21, text, GREEN, BLACK, 3);

    lcd_draw_text(4, 62, "RPM", CYAN, BLACK, 1);

    lcd_line(4, 76, 112, GREY);

    snprintf(text, sizeof(text), "STATE %d", state);
    lcd_draw_text_field(4, 84, 90, 7, text, WHITE, BLACK, 1);

    if (enabled)
        lcd_draw_text_field(4, 96, 70, 7, "ENABLED", GREEN, BLACK, 1);
    else
        lcd_draw_text_field(4, 96, 70, 7, "DISABLED", RED, BLACK, 1);

    snprintf(text, sizeof(text), "FAULT %04lX", (unsigned long)faults);
    lcd_draw_text_field(
        4, 114, 90, 7,
        text,
        faults == 0 ? GREEN : RED,
        BLACK,
        1
    );

   // ---- Right column: currents and requested speed ----

lcd_draw_text(128, 22, "CURRENT A", YELLOW, BLACK, 1);

snprintf(text, sizeof(text), "U %.2f", iu);
lcd_draw_text_field(128, 36, 84, 7, text, WHITE, BLACK, 1);

snprintf(text, sizeof(text), "V %.2f", iv);
lcd_draw_text_field(128, 48, 84, 7, text, WHITE, BLACK, 1);

snprintf(text, sizeof(text), "W %.2f", iw);
lcd_draw_text_field(128, 60, 84, 7, text, WHITE, BLACK, 1);

lcd_draw_text(128, 82, "REQUESTED", YELLOW, BLACK, 1);

snprintf(text, sizeof(text), "%.0f", requested_rpm);

lcd_draw_text_field(
    128, 96,
    108, 21,
    text,
    CYAN,
    BLACK,
    3
);

lcd_draw_text(128, 120, "RPM", CYAN, BLACK, 1);
}