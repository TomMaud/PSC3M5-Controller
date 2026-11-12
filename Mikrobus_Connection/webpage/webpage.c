
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include "lwip/err.h"

#include "webpage.h"
#include "htmlfile.h"

/* =========================================================
   CONFIGURATION
   ========================================================= */

#define HTTP_PORT          80
#define HTTP_REQUEST_SIZE  1024
#define HTTP_RESPONSE_SIZE 512
#define HTTP_SEND_CHUNK    1024

#define HTTP_STATUS_OK \
    "HTTP/1.1 200 OK\r\n"

#define HTTP_STATUS_BAD_REQUEST \
    "HTTP/1.1 400 Bad Request\r\n"

#define HTTP_STATUS_SERVER_ERROR \
    "HTTP/1.1 500 Internal Server Error\r\n"

/* handle_request() is implemented in main.c. */
extern bool handle_request(
    const char *request,
    char *response,
    size_t response_size
);

/* HTTP callback forward declarations */
static err_t http_recv_callback(
    void *arg,
    struct tcp_pcb *pcb,
    struct pbuf *p,
    err_t err
);

static err_t http_sent_callback(
    void *arg,
    struct tcp_pcb *pcb,
    u16_t len
);

static void http_error_callback(
    void *arg,
    err_t err
);

/* =========================================================
   HTTP CONNECTION STATE
   ========================================================= */

typedef struct http_state
{
    struct tcp_pcb *pcb;

    char request[HTTP_REQUEST_SIZE];
    size_t request_length;

    char response[HTTP_RESPONSE_SIZE];

    char headers[256];

    const char *body;
    size_t body_length;
    size_t body_offset;

    bool response_started;
    bool closing;

    struct http_state *next;

} http_state_t;

/* =========================================================
   GLOBAL HTTP SERVER STATE
   ========================================================= */

static struct tcp_pcb *http_listen_pcb = NULL;

static http_state_t *http_connections = NULL;

/* =========================================================
   CONNECTION LIST MANAGEMENT
   ========================================================= */

static void http_add_connection(http_state_t *state)
{
    state->next = http_connections;
    http_connections = state;
}

static void http_remove_connection(http_state_t *state)
{
    http_state_t **current = &http_connections;

    while (*current != NULL)
    {
        if (*current == state)
        {
            *current = state->next;
            state->next = NULL;
            return;
        }

        current = &(*current)->next;
    }
}

/* =========================================================
   CONNECTION CLEANUP
   ========================================================= */

static void http_free_state(http_state_t *state)
{
    if (state == NULL)
        return;

    http_remove_connection(state);

    free(state);
}

/*
 * Detach all callbacks before explicitly aborting a PCB.
 * This prevents callbacks from referring to freed state.
 */
static void http_abort_connection(http_state_t *state)
{
    if (state == NULL)
        return;

    struct tcp_pcb *pcb = state->pcb;

    http_remove_connection(state);

    if (pcb != NULL)
    {
        tcp_arg(pcb, NULL);
        tcp_recv(pcb, NULL);
        tcp_sent(pcb, NULL);
        tcp_err(pcb, NULL);
        tcp_poll(pcb, NULL, 0);

        state->pcb = NULL;

        tcp_abort(pcb);
    }

    free(state);
}

/* =========================================================
   HTTP RESPONSE SENDING
   ========================================================= */

/*
 * Attempt to send as much response data as the TCP send
 * buffer currently permits.
 */
static void http_send_body(http_state_t *state)
{
    if (state == NULL || state->pcb == NULL)
        return;

    if (state->closing)
        return;

    struct tcp_pcb *pcb = state->pcb;

    while (state->body_offset < state->body_length)
    {
        u16_t available = tcp_sndbuf(pcb);

        if (available == 0)
            break;

        size_t remaining =
            state->body_length - state->body_offset;

        size_t amount = remaining;

        if (amount > HTTP_SEND_CHUNK)
            amount = HTTP_SEND_CHUNK;

        if (amount > available)
            amount = available;

        err_t err = tcp_write(
            pcb,
            state->body + state->body_offset,
            (u16_t)amount,
            TCP_WRITE_FLAG_COPY
        );

        if (err == ERR_MEM)
            break;

        if (err != ERR_OK)
        {
            http_abort_connection(state);
            return;
        }

        state->body_offset += amount;
    }

    tcp_output(pcb);

    /*
     * All response bytes have been queued.
     * Close the TCP connection gracefully.
     */
    if (state->body_offset >= state->body_length)
    {
        state->closing = true;

        tcp_arg(pcb, NULL);
        tcp_recv(pcb, NULL);
        tcp_sent(pcb, NULL);
        tcp_err(pcb, NULL);
        tcp_poll(pcb, NULL, 0);

        err_t err = tcp_close(pcb);

        if (err == ERR_OK)
        {
            state->pcb = NULL;
            http_free_state(state);
        }
        else
        {
            /*
             * tcp_close() can fail with ERR_MEM.
             * Restore callbacks so we can retry when
             * the TCP send buffer becomes available.
             */
            state->closing = false;

            tcp_arg(pcb, state);
            tcp_recv(pcb, http_recv_callback);
            tcp_sent(pcb, http_sent_callback);
            tcp_err(pcb, http_error_callback);
        }
    }
}

/* =========================================================
   HTTP RESPONSE INITIALISATION
   ========================================================= */

static void http_start_response(
    http_state_t *state,
    const char *status,
    const char *content_type,
    const char *body,
    size_t body_length)
{
    if (state == NULL || state->pcb == NULL)
        return;

    state->body = body;
    state->body_length = body_length;
    state->body_offset = 0;

    int header_length = snprintf(
        state->headers,
        sizeof(state->headers),
        "%s"
        "Content-Type: %s\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-cache\r\n"
        "\r\n",
        status,
        content_type,
        (unsigned int)body_length
    );

    if (header_length < 0 ||
        (size_t)header_length >= sizeof(state->headers))
    {
        http_abort_connection(state);
        return;
    }

    err_t err = tcp_write(
        state->pcb,
        state->headers,
        (u16_t)header_length,
        TCP_WRITE_FLAG_COPY
    );

    if (err != ERR_OK)
    {
        http_abort_connection(state);
        return;
    }

    state->response_started = true;

    http_send_body(state);
}

/* =========================================================
   HTTP REQUEST PROCESSING
   ========================================================= */

static bool http_request_complete(const char *request)
{
    return strstr(request, "\r\n\r\n") != NULL;
}

static void http_process_request(http_state_t *state)
{
    if (state == NULL || state->pcb == NULL)
        return;

    state->request[state->request_length] = '\0';

    /*
     * Only process GET requests.
     */
    if (strncmp(state->request, "GET ", 4) != 0)
    {
        static const char bad_request[] =
            "Only GET requests are supported";

        http_start_response(
            state,
            HTTP_STATUS_BAD_REQUEST,
            "text/plain",
            bad_request,
            sizeof(bad_request) - 1
        );

        return;
    }

    /*
     * First ask main.c whether this is an API request.
     *
     * Examples:
     *   /?rpm=3000
     *   /?stop=1
     *   /?source=page
     *   /?set_time=1&hour=12...
     */
    bool api_request = handle_request(
        state->request,
        state->response,
        sizeof(state->response)
    );

    if (api_request)
    {
        http_start_response(
            state,
            HTTP_STATUS_OK,
            "application/json",
            state->response,
            strlen(state->response)
        );

        return;
    }

    /*
     * Otherwise serve the webpage.
     */
    static const char html_page[] =
        HTTP_RESPONSE_BODY_TEMPLATE;

    http_start_response(
        state,
        HTTP_STATUS_OK,
        "text/html; charset=utf-8",
        html_page,
        sizeof(html_page) - 1
    );
}

/* =========================================================
   TCP CALLBACKS
   ========================================================= */

static err_t http_sent_callback(
    void *arg,
    struct tcp_pcb *pcb,
    u16_t len)
{
    (void)len;
    (void)pcb;

    http_state_t *state = arg;

    if (state == NULL)
        return ERR_OK;

    if (!state->response_started || state->closing)
        return ERR_OK;

    http_send_body(state);

    return ERR_OK;
}

/* =========================================================
   RECEIVE CALLBACK
   ========================================================= */

static err_t http_recv_callback(
    void *arg,
    struct tcp_pcb *pcb,
    struct pbuf *p,
    err_t err)
{
    http_state_t *state = arg;

    if (state == NULL)
    {
        if (p != NULL)
            pbuf_free(p);

        return ERR_OK;
    }

    /*
     * Handle a remote client disconnect.
     */
    if (p == NULL)
    {
        tcp_arg(pcb, NULL);
        tcp_recv(pcb, NULL);
        tcp_sent(pcb, NULL);
        tcp_err(pcb, NULL);

        err_t close_err = tcp_close(pcb);

        if (close_err != ERR_OK)
        {
            tcp_abort(pcb);
        }

        state->pcb = NULL;
        http_free_state(state);

        return ERR_OK;
    }

    if (err != ERR_OK)
    {
        pbuf_free(p);

        http_abort_connection(state);

        return ERR_OK;
    }

    /*
     * A response has already started, so acknowledge and
     * discard any additional client data.
     */
    if (state->response_started)
    {
        tcp_recved(pcb, p->tot_len);
        pbuf_free(p);

        return ERR_OK;
    }

    /*
     * Accumulate request data across multiple TCP packets.
     */
    size_t available =
        sizeof(state->request) - 1 - state->request_length;

    size_t received = p->tot_len;

    if (received > available)
    {
        tcp_recved(pcb, p->tot_len);
        pbuf_free(p);

        static const char too_large[] =
            "HTTP request too large";

        http_start_response(
            state,
            HTTP_STATUS_BAD_REQUEST,
            "text/plain",
            too_large,
            sizeof(too_large) - 1
        );

        return ERR_OK;
    }

    u16_t copied = pbuf_copy_partial(
        p,
        state->request + state->request_length,
        (u16_t)received,
        0
    );

    state->request_length += copied;

    tcp_recved(pcb, p->tot_len);

    pbuf_free(p);

    state->request[state->request_length] = '\0';

    /*
     * Wait until the complete HTTP header has arrived.
     */
    if (http_request_complete(state->request))
    {
        http_process_request(state);
    }

    return ERR_OK;
}

/* =========================================================
   ERROR CALLBACK
   ========================================================= */

static void http_error_callback(
    void *arg,
    err_t err)
{
    (void)err;

    http_state_t *state = arg;

    if (state == NULL)
        return;

    /*
     * lwIP has already freed the PCB when this callback
     * is called. Only free our connection state.
     */
    state->pcb = NULL;

    http_free_state(state);
}

/* =========================================================
   ACCEPT CALLBACK
   ========================================================= */

static err_t http_accept_callback(
    void *arg,
    struct tcp_pcb *newpcb,
    err_t err)
{
    (void)arg;

    if (err != ERR_OK || newpcb == NULL)
        return ERR_VAL;

    http_state_t *state = calloc(1, sizeof(http_state_t));

    if (state == NULL)
    {
        tcp_abort(newpcb);
        return ERR_ABRT;
    }

    state->pcb = newpcb;

    http_add_connection(state);

    tcp_arg(newpcb, state);

    tcp_recv(newpcb, http_recv_callback);
    tcp_sent(newpcb, http_sent_callback);
    tcp_err(newpcb, http_error_callback);

    tcp_nagle_disable(newpcb);

    return ERR_OK;
}

/* =========================================================
   START HTTP SERVER
   ========================================================= */

bool webpage_start_http_servers(void)
{
    /*
     * Prevent accidentally creating multiple listeners.
     */
    if (http_listen_pcb != NULL)
    {
        printf("[HTTP] Server already running\n");
        return true;
    }

    printf("[HTTP] Starting HTTP server\n");

    struct tcp_pcb *pcb = tcp_new();

    if (pcb == NULL)
    {
        printf("[HTTP] ERROR: tcp_new failed\n");
        return false;
    }

    err_t err = tcp_bind(
        pcb,
        IP_ADDR_ANY,
        HTTP_PORT
    );

    if (err != ERR_OK)
    {
        printf("[HTTP] ERROR: tcp_bind failed: %d\n", err);

        tcp_abort(pcb);

        return false;
    }

    struct tcp_pcb *listen_pcb = tcp_listen(pcb);

    if (listen_pcb == NULL)
    {
        printf("[HTTP] ERROR: tcp_listen failed\n");

        tcp_abort(pcb);

        return false;
    }

    http_listen_pcb = listen_pcb;

    tcp_accept(
        http_listen_pcb,
        http_accept_callback
    );

    printf("[HTTP] Listening on port %d\n", HTTP_PORT);

    return true;
}

/* =========================================================
   STOP HTTP SERVER
   ========================================================= */

void webpage_stop_http_servers(void)
{
    printf("[HTTP] Stopping HTTP server\n");

    /*
     * First stop accepting new connections.
     */
    if (http_listen_pcb != NULL)
    {
        struct tcp_pcb *pcb = http_listen_pcb;

        http_listen_pcb = NULL;

        tcp_accept(pcb, NULL);

        err_t err = tcp_close(pcb);

        if (err != ERR_OK)
        {
            /*
             * If graceful close fails, abort the listener.
             */
            tcp_abort(pcb);
        }
    }

    /*
     * Abort all existing client connections.
     *
     * This is important before cyw43_arch_deinit(),
     * because active TCP connections must not be left
     * using the old lwIP state.
     */
    while (http_connections != NULL)
    {
        http_state_t *state = http_connections;

        http_abort_connection(state);
    }

    printf("[HTTP] HTTP server stopped\n");
}
