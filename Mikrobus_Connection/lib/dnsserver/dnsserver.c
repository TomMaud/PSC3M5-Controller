/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <stdbool.h>

#include "dnsserver.h"
#include "lwip/udp.h"

#define PORT_DNS_SERVER 53
#define DUMP_DATA 0

#define DEBUG_//printf(...)
#define ERROR_//printf //printf

typedef struct dns_header_t_ {
    uint16_t id;
    uint16_t flags;
    uint16_t question_count;
    uint16_t answer_record_count;
    uint16_t authority_record_count;
    uint16_t additional_record_count;
} dns_header_t;

#define MAX_DNS_MSG_SIZE 300

static int dns_socket_new_dgram(struct udp_pcb **udp, void *cb_data, udp_recv_fn cb_udp_recv) {
    *udp = udp_new();
    if (*udp == NULL) {
        return -ENOMEM;
    }
    udp_recv(*udp, cb_udp_recv, (void *)cb_data);
    return ERR_OK;
}

static void dns_socket_free(struct udp_pcb **udp) {
    if (*udp != NULL) {
        udp_remove(*udp);
        *udp = NULL;
    }
}

static int dns_socket_bind(struct udp_pcb **udp, uint32_t ip, uint16_t port) {
    ip_addr_t addr;
    IP4_ADDR(&addr, ip >> 24 & 0xff, ip >> 16 & 0xff, ip >> 8 & 0xff, ip & 0xff);
    err_t err = udp_bind(*udp, &addr, port);
    if (err != ERR_OK) {
        ERROR_//printf("dns failed to bind to port %u: %d", port, err);
        assert(false);
    }
    return err;
}

#if DUMP_DATA
static void dump_bytes(const uint8_t *bptr, uint32_t len) {
    unsigned int i = 0;

    for (i = 0; i < len;) {
        if ((i & 0x0f) == 0) {
            //printf("\n");
        } else if ((i & 0x07) == 0) {
            //printf(" ");
        }
        //printf("%02x ", bptr[i++]);
    }
    //printf("\n");
}
#endif

static int dns_socket_sendto(struct udp_pcb **udp, const void *buf, size_t len, const ip_addr_t *dest, uint16_t port) {
    if (len > 0xffff) {
        len = 0xffff;
    }

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (p == NULL) {
        ERROR_//printf("DNS: Failed to send message out of memory\n");
        return -ENOMEM;
    }

    memcpy(p->payload, buf, len);
    err_t err = udp_sendto(*udp, p, dest, port);

    pbuf_free(p);

    if (err != ERR_OK) {
        ERROR_//printf("DNS: Failed to send message %d\n", err);
        return err;
    }

#if DUMP_DATA
    dump_bytes(buf, len);
#endif
    return len;
}

static void dns_server_process(
    void *arg,
    struct udp_pcb *upcb,
    struct pbuf *p,
    const ip_addr_t *src_addr,
    u16_t src_port)
{
    dns_server_t *d = arg;

    (void)upcb;

    //DEBUG_//printf("DNS request: %u bytes\n", p->tot_len);

    uint8_t dns_msg[MAX_DNS_MSG_SIZE];
    dns_header_t *dns_hdr = (dns_header_t *)dns_msg;

    size_t msg_len = pbuf_copy_partial(
        p,
        dns_msg,
        sizeof(dns_msg),
        0
    );

    if (msg_len < sizeof(dns_header_t)) {
        goto ignore_request;
    }

    uint16_t flags = lwip_ntohs(dns_hdr->flags);
    uint16_t question_count = lwip_ntohs(dns_hdr->question_count);

    /* Must be a query */
    if ((flags & 0x8000) != 0) {
        goto ignore_request;
    }

    /* Only standard queries */
    if (((flags >> 11) & 0x0F) != 0) {
        goto ignore_request;
    }

    if (question_count < 1) {
        goto ignore_request;
    }

    /*
     * ---------------------------------------------------------
     * Parse QNAME
     * ---------------------------------------------------------
     */

    const uint8_t *question_ptr_start =
        dns_msg + sizeof(dns_header_t);

    const uint8_t *question_ptr_end =
        dns_msg + msg_len;

    const uint8_t *question_ptr =
        question_ptr_start;

    char hostname[256];
    size_t hostname_len = 0;

    while (question_ptr < question_ptr_end) {

        uint8_t label_len = *question_ptr++;

        if (label_len == 0) {
            break;
        }

        if (label_len > 63) {
            goto ignore_request;
        }

        if (question_ptr + label_len > question_ptr_end) {
            goto ignore_request;
        }

        if (hostname_len != 0) {
            if (hostname_len >= sizeof(hostname) - 1) {
                goto ignore_request;
            }

            hostname[hostname_len++] = '.';
        }

        if (hostname_len + label_len >= sizeof(hostname)) {
            goto ignore_request;
        }

        memcpy(
            &hostname[hostname_len],
            question_ptr,
            label_len
        );

        hostname_len += label_len;
        question_ptr += label_len;
    }

    hostname[hostname_len] = '\0';

    /*
     * Need QTYPE + QCLASS
     */
    if (question_ptr + 4 > question_ptr_end) {
        goto ignore_request;
    }

    uint16_t qtype =
        ((uint16_t)question_ptr[0] << 8) |
        question_ptr[1];

    uint16_t qclass =
        ((uint16_t)question_ptr[2] << 8) |
        question_ptr[3];

    question_ptr += 4;

    //printf(    "[DNS] Query: %s  type=%u class=%u\n",hostname, qtype,  qclass);

    /*
     * ---------------------------------------------------------
     * We specifically support:
     *
     *     motor.pico
     *
     * ---------------------------------------------------------
     */

    bool is_motor_pico =
        (strcasecmp(hostname, "motor.pico") == 0);

    /*
     * We only provide IPv4 A records.
     *
     * A = 1
     * AAAA = 28
     */

    if (qtype != 1) {

        //printf(
        //    "[DNS] Ignoring non-A query for %s (type %u)\n",
        //    hostname,
        //    qtype
        //);

        goto ignore_request;
    }

    /*
     * ---------------------------------------------------------
     * Build DNS response
     * ---------------------------------------------------------
     */

    uint8_t *answer_ptr =
        dns_msg + (question_ptr - dns_msg);

    /*
     * Compression pointer back to QNAME.
     */
    *answer_ptr++ = 0xC0;
    *answer_ptr++ =
        (uint8_t)(question_ptr_start - dns_msg);

    /*
     * TYPE = A
     */
    *answer_ptr++ = 0x00;
    *answer_ptr++ = 0x01;

    /*
     * CLASS = IN
     */
    *answer_ptr++ = 0x00;
    *answer_ptr++ = 0x01;

    /*
     * TTL = 60 seconds
     */
    *answer_ptr++ = 0x00;
    *answer_ptr++ = 0x00;
    *answer_ptr++ = 0x00;
    *answer_ptr++ = 0x3C;

    /*
     * IPv4 address length
     */
    *answer_ptr++ = 0x00;
    *answer_ptr++ = 0x04;

    /*
     * Return Pico AP address:
     *
     *     192.168.4.1
     */
    memcpy(
        answer_ptr,
        &d->ip.addr,
        4
    );

    answer_ptr += 4;

    /*
     * ---------------------------------------------------------
     * DNS header
     * ---------------------------------------------------------
     */

    dns_hdr->flags = lwip_htons(
        0x8000 |   /* QR = response */
        0x0400 |   /* AA = authoritative */
        0x0080     /* RA */
    );

    dns_hdr->question_count =
        lwip_htons(1);

    dns_hdr->answer_record_count =
        lwip_htons(1);

    dns_hdr->authority_record_count =
        lwip_htons(0);

    dns_hdr->additional_record_count =
        lwip_htons(0);

    //printf(
     //   "[DNS] %s -> 192.168.4.1%s\n",
      //  hostname,
      //  is_motor_pico ? " [motor.PICO]" : ""
    //);

    /*
     * Send response
     */
    dns_socket_sendto(
        &d->udp,
        dns_msg,
        answer_ptr - dns_msg,
        src_addr,
        src_port
    );

ignore_request:

    pbuf_free(p);
}

void dns_server_init(dns_server_t *d, struct netif *nif, ip_addr_t *ip) {
    if (dns_socket_new_dgram(&d->udp, d, dns_server_process) != ERR_OK) {
        DEBUG_//printf("dns server failed to start\n");
        return;
    }
    if (dns_socket_bind(&d->udp, 0, PORT_DNS_SERVER) != ERR_OK) {
        DEBUG_//printf("dns server failed to bind\n");
        return;
    }
    // Restrict the server to a single interface (the AP). The bind above uses
    // IP_ADDR_ANY, so without this the PCB listens on every netif and would
    // answer DNS queries arriving via an active STA connection too. This also
    // pins replies to the same netif (udp_sendto here is not interface-bound).
    if (nif != NULL) {
        udp_bind_netif(d->udp, nif);
    }
    ip_addr_copy(d->ip, *ip);
    DEBUG_//printf("dns server listening on port %d\n", PORT_DNS_SERVER);
}

void dns_server_deinit(dns_server_t *d) {
    dns_socket_free(&d->udp);
}