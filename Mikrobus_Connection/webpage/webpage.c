
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h> 
#include <inttypes.h>
#include <string.h>

#include "pico/stdlib.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "webpage.h"
#include "htmlfile.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "dhcpserver.h"
#include "dnsserver.h"
#include "htmlfile.h"

#define HTTP_PORT 80

typedef struct
{
    const char *ptr;
    size_t left;
} http_state_t;


/* =========================================================
   HTTP CONNECTION MANAGEMENT
   ========================================================= */

static void http_close_connection(struct tcp_pcb *tpcb)
{
    if (tpcb != NULL)
    {
        http_state_t *hs = (http_state_t *)tpcb->callback_arg;

        if (hs)
        {
            free(hs);
        }

        tcp_arg(tpcb, NULL);
        tcp_sent(tpcb, NULL);
        tcp_recv(tpcb, NULL);
        tcp_err(tpcb, NULL);

        tcp_close(tpcb);
    }
}


static void http_err_callback(void *arg, err_t err)
{
    (void)err;

    http_state_t *hs = (http_state_t *)arg;

    if (hs)
    {
        free(hs);
    }
}


/* =========================================================
   HTTP DATA TRANSMISSION
   ========================================================= */

static void http_send_data(struct tcp_pcb *tpcb, http_state_t *hs)
{
    while (hs->left > 0 && tcp_sndbuf(tpcb) > 0)
    {
        uint16_t available = tcp_sndbuf(tpcb);
        uint16_t chunk_size = (available > 512) ? 512 : available;

        if (chunk_size > hs->left)
        {
            chunk_size = (uint16_t)hs->left;
        }

        err_t err = tcp_write(
            tpcb,
            hs->ptr,
            chunk_size,
            TCP_WRITE_FLAG_COPY
        );

        if (err != ERR_OK)
        {
            break;
        }

        hs->ptr += chunk_size;
        hs->left -= chunk_size;
    }

    tcp_output(tpcb);
}


static err_t http_sent_callback(
    void *arg,
    struct tcp_pcb *tpcb,
    uint16_t len
)
{
    (void)len;

    http_state_t *hs = (http_state_t *)arg;

    if (hs && hs->left > 0)
    {
        http_send_data(tpcb, hs);
    }
    else
    {
        http_close_connection(tpcb);
    }

    return ERR_OK;
}


/* =========================================================
   HTTP RESPONSE
   ========================================================= */

static void http_send_response(
    struct tcp_pcb *tpcb,
    const char *content_type,
    const char *body
)
{
    char headers[192];

    int header_len = snprintf(
        headers,
        sizeof(headers),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %u\r\n"
        "Connection: close\r\n"
        "\r\n",
        content_type,
        (unsigned int)strlen(body)
    );

    tcp_write(tpcb, headers, header_len, TCP_WRITE_FLAG_COPY);
    tcp_write(tpcb, body, strlen(body), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    http_close_connection(tpcb);
}


/* =========================================================
   HTTP RECEIVE CALLBACK
   ========================================================= */


static err_t http_recv_callback(void *arg,
                                struct tcp_pcb *tpcb,
                                struct pbuf *p,
                                err_t err)
{
    (void)arg;

    if (p == NULL)
    {
        http_close_connection(tpcb);
        return ERR_OK;
    }

    if (err != ERR_OK)
    {
        pbuf_free(p);
        http_close_connection(tpcb);
        return err;
    }

    tcp_recved(tpcb, p->tot_len);

    char request[1024] = {0};
    u16_t copy_len = p->tot_len;

    if (copy_len >= sizeof(request))
    {
        copy_len = sizeof(request) - 1;
    }

    pbuf_copy_partial(p, request, copy_len, 0);
    pbuf_free(p);

    /* printf("HTTP request:\n%s\n", request); */

    /* Ignore anything that isn't GET / or GET /?... (e.g. favicon.ico) */
    if (strncmp(request, "GET / ", 6) != 0 &&
        strncmp(request, "GET /?", 6) != 0)
    {
        static const char not_found[] =
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Length: 0\r\n"
            "Connection: close\r\n"
            "\r\n";

        tcp_write(tpcb, not_found, sizeof(not_found) - 1, TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);
        http_close_connection(tpcb);
        return ERR_OK;
    }

    /* API request */
    char response[256] = {0};

    if (handle_request(request, response, sizeof(response)))
    {
        char http_response[384];

        snprintf(http_response, sizeof(http_response),
                 "HTTP/1.1 200 OK\r\n"
                 "Content-Type: application/json\r\n"
                 "Access-Control-Allow-Origin: *\r\n"
                 "Content-Length: %u\r\n"
                 "Connection: close\r\n"
                 "\r\n"
                 "%s",
                 (unsigned int)strlen(response),
                 response);

        if (tcp_write(tpcb, http_response, strlen(http_response),
                      TCP_WRITE_FLAG_COPY) == ERR_OK)
        {
            tcp_output(tpcb);
        }

        http_close_connection(tpcb);
        return ERR_OK;
    }

    /* Page request */
    http_state_t *hs = malloc(sizeof(http_state_t));

    if (hs == NULL)
    {
        http_close_connection(tpcb);
        return ERR_OK;
    }

    hs->ptr  = HTTP_RESPONSE_BODY_TEMPLATE;
    hs->left = strlen(HTTP_RESPONSE_BODY_TEMPLATE);

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent_callback);
    tcp_err(tpcb, http_err_callback);

    /* Header first */
    char headers[160];
    int header_len = snprintf(headers, sizeof(headers),
                              "HTTP/1.1 200 OK\r\n"
                              "Content-Type: text/html\r\n"
                              "Content-Length: %u\r\n"
                              "Connection: close\r\n"
                              "\r\n",
                              (unsigned int)hs->left);

    tcp_write(tpcb, headers, header_len, TCP_WRITE_FLAG_COPY);
    http_send_data(tpcb, hs);

    return ERR_OK;
}

/* =========================================================
   HTTP ACCEPT CALLBACK
   ========================================================= */

static err_t http_accept_callback(
    void *arg,
    struct tcp_pcb *newpcb,
    err_t err
)
{
    (void)arg;

    if (err != ERR_OK || newpcb == NULL)
    {
        return ERR_VAL;
    }

    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, http_recv_callback);
    tcp_err(newpcb, http_err_callback);

    return ERR_OK;
}


/* =========================================================
   START HTTP SERVER
   ========================================================= */

bool webpage_start_http_servers(void)
{
    struct tcp_pcb *pcb = tcp_new();

    if (!pcb)
    {
        return false;
    }

    if (tcp_bind(pcb, IP_ADDR_ANY, HTTP_PORT) != ERR_OK)
    {
        tcp_close(pcb);
        return false;
    }

    struct tcp_pcb *listen_pcb = tcp_listen(pcb);

    if (!listen_pcb)
    {
        return false;
    }

    tcp_accept(listen_pcb, http_accept_callback);

    printf("Motor controller webpage running on port %d\n", HTTP_PORT);

    return true;
}
