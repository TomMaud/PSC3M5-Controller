#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "pico/sync.h"
#include "pico/cyw43_arch.h"
#include "pico/low_power.h"

#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"

#include "dhcpserver.h"
#include "dnsserver.h"

#include "lcd/lcd.h"
#include "telemetry/telemetry.h"
#include "webpage/webpage.h"


/* =========================================================
   CONFIGURATION
   ========================================================= */

#define AP_SSID               "MotorController"
#define AP_PASSWORD           "motor1234"      /* WPA2: 8+ characters */

#define UART_ID               uart0
#define UART_IRQ              UART0_IRQ
#define UART_TX_PIN           0
#define UART_RX_PIN           1
#define UART_BAUD             115200

#define HEARTBEAT_INTERVAL_MS 250
#define TELEMETRY_TIMEOUT_MS  1500
#define LCD_MIN_INTERVAL_MS   100

#define LINE_BUFFER_SIZE      512
#define RX_RING_SIZE          512

#define RPM_LIMIT             6000

static dhcp_server_t dhcp_server = {0};
static dns_server_t dns_server = {0};

/* =========================================================
   SHARED TYPES (core 0 <-> core 1)
   ========================================================= */

typedef enum
{
    CMD_RPM,
    CMD_STOP,
    CMD_SOURCE,
    CMD_TIME
} cmd_type_t;

typedef struct
{
    cmd_type_t type;
    int value;

    int hour;
    int minute;
    int second;

    int day;
    int month;
    int year;
} motor_cmd_t;

typedef struct
{
    float    rpm;           // Actual motor speed
    float    requested_rpm; // Requested motor speed
    float    iu, iv, iw;    // Phase currents
    float    vu, vv, vw;    // Phase voltages
    int      state;
    int      enabled;
    uint32_t faults;
    bool     connected;
} telemetry_t;

static queue_t            cmd_queue;       /* core 0 -> core 1 */
static telemetry_t        telem_shared;    /* core 1 -> core 0 */
static critical_section_t telem_lock;

/* Written and read only on core 0 (web side) */
static int control_source = 1;             /* 1 = webpage, 0 = potentiometer */
static int last_rpm_request = 0;

/* =========================================================
   CORE 1: UART RX RING BUFFER (filled by IRQ on core 1)
   ========================================================= */

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;      /* written by IRQ */
static volatile uint16_t rx_tail = 0;      /* written by main loop on core 1 */
static volatile uint32_t rx_overflows = 0;

static volatile bool sleep_requested = false;
static volatile bool core1_sleep_ready = false;
static volatile bool resume_core1 = false;

static void stop_wifi_services(void)
{
    printf("[POWER] Stopping Wi-Fi services\n");

    webpage_stop_http_servers();

    dns_server_deinit(&dns_server);
    dhcp_server_deinit(&dhcp_server);

    cyw43_arch_deinit();

    printf("[POWER] Wi-Fi services stopped\n");
}

static void uart_rx_isr(void);


static void uart_link_restore(void)
{
    uart_init(UART_ID, UART_BAUD);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_fifo_enabled(UART_ID, true);

    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);
}

static void uart_link_init(void)
{
    uart_init(UART_ID, UART_BAUD);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_fifo_enabled(UART_ID, true);

    /* Register the handler once on Core 1. */
    irq_set_exclusive_handler(UART_IRQ, uart_rx_isr);
    irq_set_enabled(UART_IRQ, true);

    uart_set_irq_enables(UART_ID, true, false);
}

static bool restore_wifi_after_wake(void)
{
    printf("[WAKE] Restoring Wi-Fi system\n");

    /* 1. Initialise CYW43 and lwIP. */
    if (cyw43_arch_init())
    {
        printf("[WAKE] ERROR: CYW43 initialisation failed\n");
        return false;
    }

    /* 2. Re-enable the access point. */
    cyw43_arch_enable_ap_mode(
        AP_SSID,
        AP_PASSWORD,
        CYW43_AUTH_WPA2_AES_PSK
    );

    /* 3. Configure the AP network address. */
    ip_addr_t gw;
    ip_addr_t mask;

    IP4_ADDR(ip_2_ip4(&gw), 192, 168, 4, 1);
    IP4_ADDR(ip_2_ip4(&mask), 255, 255, 255, 0);

    /* 4. Obtain the AP network interface. */
    struct netif *ap_netif = &cyw43_state.netif[CYW43_ITF_AP];

    /* 5. Restart DHCP. */
    dhcp_server_init(
        &dhcp_server,
        ap_netif,
        &gw,
        &mask
    );

    /* 6. Restart DNS. */
    dns_server_init(
        &dns_server,
        ap_netif,
        &gw
    );

    /* 7. Restart HTTP. */
    if (!webpage_start_http_servers())
    {
        printf("[WAKE] ERROR: HTTP server failed to start\n");

        /* Clean up the services that were successfully started. */
        dns_server_deinit(&dns_server);
        dhcp_server_deinit(&dhcp_server);
        cyw43_arch_deinit();

        return false;
    }

    printf("[WAKE] Wi-Fi access point restored\n");
    printf("[WAKE] SSID: %s\n", AP_SSID);
    printf("[WAKE] Address: http://192.168.4.1\n");

    return true;
}

static void release_core1_after_wake(void)
{
    printf("[WAKE] Releasing Core 1\n");

    resume_core1 = true;
    __sev();
}

static void enter_and_restore_dormant(void)
{
    printf("[POWER] Waiting for Core 1 to park\n");

    while (!core1_sleep_ready)
    {
        tight_loop_contents();
    }

    __dmb();

    printf("[POWER] Core 1 parked; shutting down Wi-Fi\n");

    /* Stop HTTP, DNS, DHCP and CYW43 cleanly. */
    stop_wifi_services();

    /* Configure GP1 as the dormant wake input. */
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(UART_RX_PIN, GPIO_IN);
    gpio_pull_up(UART_RX_PIN);
    lcd_deinit();

    printf("[POWER] Entering Dormant; waiting for PSoC UART\n");

    int result = low_power_dormant_until_gpio_pin_state(
        UART_RX_PIN,
        true,
        false,
        DORMANT_CLOCK_SOURCE_ROSC,
        NULL
    );

    printf("[WAKE] Dormant returned, result = %d\n", result);

    /* Restore Wi-Fi. */
    bool wifi_restored = restore_wifi_after_wake();

    if (!wifi_restored)
    {
        printf("[WAKE] WARNING: Wi-Fi restoration failed\n");
    }

    /* Release Core 1 regardless of Wi-Fi restoration outcome. */
    sleep_requested = false;
    __dmb();

    release_core1_after_wake();
}


static void uart_rx_isr(void)
{
    while (uart_is_readable(UART_ID))
    {
        uint8_t c = (uint8_t)uart_getc(UART_ID);
        uint16_t next = (uint16_t)((rx_head + 1) & (RX_RING_SIZE - 1));

        if (next == rx_tail)
        {
            rx_overflows++;                /* ring full, drop byte */
        }
        else
        {
            rx_ring[rx_head] = c;
            rx_head = next;
        }
    }
}

static bool rx_ring_pop(uint8_t *c)
{
    if (rx_tail == rx_head)
        return false;

    *c = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) & (RX_RING_SIZE - 1));
    return true;
}

/* =========================================================
   CORE 1: SEND COMMANDS TO THE PSC3M5
   ========================================================= */

static void send_command_to_infineon(const motor_cmd_t *cmd)
{
    char line[32];

    switch (cmd->type)
    {
        case CMD_RPM:
            printf("Setting RPM to %d\n", cmd->value);
            snprintf(line, sizeof(line), "R%d\n", cmd->value);
            break;

        case CMD_STOP:
            snprintf(line, sizeof(line),"R%d\n", 0);
            break;

        case CMD_SOURCE:
            snprintf(line, sizeof(line), "S:%s\n",
                     cmd->value ? "PAGE" : "POT");
            break;
        
        case CMD_TIME:
            printf("Setting RTC to %02d:%02d:%02d %02d/%02d/%04d\n",
                   cmd->hour,
                   cmd->minute,
                   cmd->second,
                   cmd->day,
                   cmd->month,
                   cmd->year);
            snprintf(
            line,
            sizeof(line),
            "T:%02d:%02d:%02d,%02d/%02d/%02d\n",
            cmd->hour,
            cmd->minute,
            cmd->second,
            cmd->day,
            cmd->month,
            cmd->year % 100
        );
    break;

        default:
            return;
    }

    uart_puts(UART_ID, line);
    printf("To Infineon: %s", line);
}

/* =========================================================
   CORE 1 MAIN
   ========================================================= */

static void core1_main(void)
{
    lcd_init();
    uart_link_init();

    char     line[LINE_BUFFER_SIZE];
    uint16_t index = 0;

    telemetry_t t = {0};

    uint32_t last_heartbeat = to_ms_since_boot(get_absolute_time());
    uint32_t last_telemetry = 0;
    uint32_t last_draw      = 0;
    bool     dirty          = true;     /* force first draw */

    printf("Core 1: UART %d baud, TX GP%d RX GP%d, LCD ready\n",
           UART_BAUD, UART_TX_PIN, UART_RX_PIN);

    while (true)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* ---- Commands from the web side ---- */
        motor_cmd_t cmd;
        while (queue_try_remove(&cmd_queue, &cmd))
        {
            send_command_to_infineon(&cmd);
        }

        /* ---- Heartbeat ---- */
        if ((now - last_heartbeat) >= HEARTBEAT_INTERVAL_MS)
        {
            uart_putc_raw(UART_ID, 'H');
            last_heartbeat = now;
        }

        /* ---- Telemetry from the ring buffer ---- */
        uint8_t c;
        while (rx_ring_pop(&c))
        {
            if (c == '\n' || c == '\r')
            {
                if (index > 0)
                {
                    line[index] = '\0';
                    printf(line);

                    if (strncmp(line, "SLEEP:", 6) == 0){
                        printf("Received sleep message: %s\n", line);
                        sleep_requested = true;
                    }

                    if (strncmp(line, "TIME:", 5) == 0)
{
    if (telemetry_parse_rtc(line))
    {
        dirty = true;

        printf("RTC updated: %02d:%02d:%02d %02d/%02d/%04d\n",
               rtc_hour,
               rtc_minute,
               rtc_second,
               rtc_day,
               rtc_month,
               rtc_year);
    }
    else
    {
        printf("Invalid RTC message: %s\n", line);
    }
}
else
{
    if (telemetry_parse_line(
        line,
        &t.rpm,
        &t.requested_rpm,
        &t.iu,
        &t.iv,
        &t.iw,
        &t.vu,
        &t.vv,
        &t.vw,
        &t.state,
        &t.enabled,
        &t.faults))
    {
        last_telemetry = now;
        t.connected = true;
        dirty = true;
    }
}
                    index = 0;
                }
            }
            else if (index < LINE_BUFFER_SIZE - 1)
            {
                line[index++] = (char)c;
            }
            else
            {
                index = 0;
                printf("Receive line too long, discarded\n");
            }
        }

        /* ---- Lost link ---- */
        if (t.connected && (now - last_telemetry) > TELEMETRY_TIMEOUT_MS)
        {
            t.connected = false;
            dirty = true;
            printf("Infineon connection lost\n");
        }

        /* ---- Display (rate limited) ---- */
        if (dirty && (now - last_draw) >= LCD_MIN_INTERVAL_MS)
        {
            lcd_draw_dashboard(
                t.rpm,
                t.requested_rpm,
                t.iu,
                t.iv,
                t.iw,
                t.vu,
                t.vv,
                t.vw,
                t.state,
                t.enabled,
                rtc_valid,
                rtc_hour,
                rtc_minute,
                rtc_second,
                t.faults,
                t.connected
            );

            /* Publish snapshot for the web side */
            critical_section_enter_blocking(&telem_lock);
            telem_shared = t;
            critical_section_exit(&telem_lock);

            last_draw = to_ms_since_boot(get_absolute_time());
            dirty = false;
        }
        /* ---- Enter sleep only when requested ---- */
if (sleep_requested)
{
    printf("[POWER] Core 1 preparing to sleep\n");
    uart_puts(UART_ID, "SLEEP_OK\n");
    uart_tx_wait_blocking(UART_ID);

    /* 2. Stop UART interrupts. */
    uart_set_irq_enables(UART_ID, false, false);
    irq_set_enabled(UART_IRQ, false);

    /* 3. Ensure all transmitted data has left the UART. */
    uart_tx_wait_blocking(UART_ID);

    /* 4. Shut down UART0. */
    uart_deinit(UART_ID);


    /* Tell Core 0 that Core 1 is ready to park. */
    core1_sleep_ready = true;
    __dmb();
    __sev();

    /* Park Core 1 until Core 0 releases it after wake. */
    while (!resume_core1)
    {
        __wfe();
    }

    __dmb();

    /* Core 0 has restored the chip clocks and Wi-Fi. */
    printf("[WAKE] Core 1 restoring UART and LCD\n");

    /* Reset the receive buffer to discard incomplete wake bytes. */
    rx_head = 0;
    rx_tail = 0;
    index = 0;

    uart_link_restore();

    lcd_init();

    /* Reset communication timing. */
    last_heartbeat = to_ms_since_boot(get_absolute_time());
    last_telemetry = 0;

    t.connected = false;
    dirty = true;

    /* Clear handshake flags for the next sleep cycle. */
    resume_core1 = false;
    core1_sleep_ready = false;
    sleep_requested = false;

    __dmb();

    printf("[WAKE] Core 1 resumed\n");
}
    }
}

/* =========================================================
   CORE 0: WEB REQUEST HANDLING
   ========================================================= */

/* Find "name=value" in a query string. Matches only at the start of the
   query or right after '&'. Copies value into out. */
static bool get_query_param(const char *query,
                            size_t query_len,
                            const char *name,
                            char *out,
                            size_t out_size)
{
    size_t name_len = strlen(name);
    const char *p = query;
    const char *end = query + query_len;

    while (p < end)
    {
        const char *amp = memchr(p, '&', (size_t)(end - p));
        const char *item_end = amp ? amp : end;

        if ((size_t)(item_end - p) > name_len &&
            strncmp(p, name, name_len) == 0 &&
            p[name_len] == '=')
        {
            size_t vlen = (size_t)(item_end - (p + name_len + 1));

            if (vlen >= out_size)
                vlen = out_size - 1;

            memcpy(out, p + name_len + 1, vlen);
            out[vlen] = '\0';
            return true;
        }

        p = item_end + 1;
    }

    return false;
}

static bool get_query_int(
    const char *query,
    size_t query_len,
    const char *name,
    int min_value,
    int max_value,
    int *result)
{
    char value[16];

    if (!get_query_param(query, query_len, name,
                         value, sizeof(value)))
        return false;

    char *end;
    long number = strtol(value, &end, 10);

    if (end == value || *end != '\0' ||
        number < min_value || number > max_value)
        return false;

    *result = (int)number;
    return true;
}
/*
 * Called from the HTTP server on core 0.
 * Returns true if the request was an API call (response filled with JSON),
 * false if the webpage itself should be served.
 */
bool handle_request(const char *request, char *response, size_t response_size)
{
    /* Request line looks like: GET /?motor=1&rpm=3000 HTTP/1.1 */
    if (strncmp(request, "GET /?", 6) != 0)
        return false;

    const char *query = request + 6;
    const char *query_end = strchr(query, ' ');

    if (query_end == NULL)
        return false;

    size_t query_len = (size_t)(query_end - query);

    char value[16];
    bool handled = false;
    bool queued  = true;

    /* ---- Control source: pot / page ---- */
    if (get_query_param(query, query_len, "source", value, sizeof(value)))
    {
        control_source = (strcmp(value, "page") == 0) ? 1 : 0;

        motor_cmd_t cmd = { CMD_SOURCE, control_source };
        queued &= queue_try_add(&cmd_queue, &cmd);
        handled = true;
    }

    /* ---- Stop (always allowed) ---- */
    if (get_query_param(query, query_len, "stop", value, sizeof(value)))
    {
        last_rpm_request = 0;

        motor_cmd_t cmd = { CMD_STOP, 0 };
        queued &= queue_try_add(&cmd_queue, &cmd);
        handled = true;
    }

    /* ---- Speed ---- */
    if (get_query_param(query, query_len, "rpm", value, sizeof(value)))
    {
        handled = true;

        if (control_source == 1)
        {
            long rpm = strtol(value, NULL, 10);

            if (rpm >  RPM_LIMIT) rpm =  RPM_LIMIT;
            if (rpm < -RPM_LIMIT) rpm = -RPM_LIMIT;

            last_rpm_request = (int)rpm;

            motor_cmd_t cmd = { CMD_RPM, (int)rpm };
            queued &= queue_try_add(&cmd_queue, &cmd);
        }
        else
        {
            queued = false;   /* potentiometer has control, ignore web RPM */
        }
    }
        /* ---- RTC time synchronisation ---- */
    if (get_query_param(query, query_len, "set_time",
                        value, sizeof(value)))
    {
        handled = true;

        motor_cmd_t cmd = {0};
        cmd.type = CMD_TIME;

        bool valid =
            get_query_int(query, query_len, "hour", 0, 23, &cmd.hour) &&
            get_query_int(query, query_len, "minute", 0, 59, &cmd.minute) &&
            get_query_int(query, query_len, "second", 0, 59, &cmd.second) &&
            get_query_int(query, query_len, "day", 1, 31, &cmd.day) &&
            get_query_int(query, query_len, "month", 1, 12, &cmd.month) &&
            get_query_int(query, query_len, "year", 2000, 2099, &cmd.year);

        if (valid)
        {
            printf("[HTTP] Time received: %02d:%02d:%02d %02d/%02d/%04d\n",
                   cmd.hour, cmd.minute, cmd.second,
                   cmd.day, cmd.month, cmd.year);

            queued &= queue_try_add(&cmd_queue, &cmd);
        }
        else
        {
            printf("[HTTP] Invalid time parameters\n");
            queued = false;
        }
    }

    if (!handled)
        return false;

    /* Snapshot of telemetry published by core 1 */
    telemetry_t snap;
    critical_section_enter_blocking(&telem_lock);
    snap = telem_shared;
    critical_section_exit(&telem_lock);

    snprintf(response, response_size,
             "{\"ok\":%d,\"source\":\"%s\",\"requested\":%d,"
             "\"rpm\":%.0f,\"link\":%d}",
             queued ? 1 : 0,
             control_source ? "page" : "pot",
             last_rpm_request,
             snap.rpm,
             snap.connected ? 1 : 0);

    return true;
}

/* =========================================================
   CORE 0 MAIN
   ========================================================= */



int main(void)
{
    stdio_init_all();
    sleep_ms(1000);

    printf("\nPico W - Infineon Motor Monitor (multicore)\n");

    queue_init(&cmd_queue, sizeof(motor_cmd_t), 8);
critical_section_init(&telem_lock);

/* Start with webpage control enabled on the PSoC */
motor_cmd_t initial_source = { CMD_SOURCE, 1 };
queue_try_add(&cmd_queue, &initial_source);

/* UART + LCD live on core 1 */
multicore_launch_core1(core1_main);

    /* ---- Wi-Fi access point (core 0 only) ---- */
    if (cyw43_arch_init())
    {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    cyw43_arch_enable_ap_mode(AP_SSID, AP_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);

    ip_addr_t gw, mask;
    IP4_ADDR(ip_2_ip4(&gw),   192, 168, 4, 1);
    IP4_ADDR(ip_2_ip4(&mask), 255, 255, 255, 0);

    struct netif *ap_netif = &cyw43_state.netif[CYW43_ITF_AP];

    dhcp_server_init(&dhcp_server, ap_netif, &gw, &mask);
    dns_server_init(&dns_server, ap_netif, &gw);

    if (!webpage_start_http_servers())
    {
        printf("HTTP server failed to start\n");
    }

    printf("AP \"%s\" up - browse to http://192.168.4.1\n", AP_SSID);

    while (true)
{
    cyw43_arch_poll();

    if (sleep_requested)
    {
        enter_and_restore_dormant();
    }

    sleep_ms(1);
}

    return 0;
}