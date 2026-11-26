// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "app.h"
#include "app_udp.h"
#include "app_lora.h"
#include "app_serialflash.h"
#include "tcpip/tcpip.h"
#include "system/tmr/sys_tmr.h"
#include "time.h"
#include "../../vendor/jsmn/jsmn.h"

/* ── tunables ─────────────────────────────────────────────────────────── */
#define UDP_PULL_INTERVAL_TICKS (5  * SYS_TMR_TickCounterFrequencyGet())
#define UDP_STAT_INTERVAL_TICKS (30 * SYS_TMR_TickCounterFrequencyGet())
#define UDP_CONNECTED_TIMEOUT   30 /* seconds without PULL_ACK → disconnected */
#define UDP_TX_BUFFER_SIZE      700
#define UDP_RX_BUFFER_SIZE      384
#define SEMTECH_PROTO_VER       0x02
#define MSG_PUSH_DATA           0x00
#define MSG_PUSH_ACK            0x01
#define MSG_PULL_DATA           0x02
#define MSG_PULL_RESP           0x03
#define MSG_PULL_ACK            0x04
#define MSG_TX_ACK              0x05

/* ── base64 alphabet ──────────────────────────────────────────────────── */
static const char b64chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static uint32_t b64_encoded_len(uint32_t n)
{
    return ((n + 2) / 3) * 4;
}

static void b64_encode(const uint8_t* src, uint32_t srclen, char* dst)
{
    uint32_t i = 0, j = 0;
    for(; i + 2 < srclen; i += 3)
    {
        dst[j++] = b64chars[(src[i] >> 2) & 0x3F];
        dst[j++] = b64chars[((src[i] & 0x03) << 4) | ((src[i + 1] >> 4) & 0x0F)];
        dst[j++] = b64chars[((src[i + 1] & 0x0F) << 2) | ((src[i + 2] >> 6) & 0x03)];
        dst[j++] = b64chars[src[i + 2] & 0x3F];
    }
    if(i < srclen)
    {
        dst[j++] = b64chars[(src[i] >> 2) & 0x3F];
        if(i + 1 < srclen)
        {
            dst[j++] = b64chars[((src[i] & 0x03) << 4) | ((src[i + 1] >> 4) & 0x0F)];
            dst[j++] = b64chars[(src[i + 1] & 0x0F) << 2];
        }
        else
        {
            dst[j++] = b64chars[(src[i] & 0x03) << 4];
            dst[j++] = '=';
        }
        dst[j++] = '=';
    }
    dst[j] = '\0';
}

static int8_t b64_decode_byte(char c)
{
    if(c >= 'A' && c <= 'Z') return c - 'A';
    if(c >= 'a' && c <= 'z') return c - 'a' + 26;
    if(c >= '0' && c <= '9') return c - '0' + 52;
    if(c == '+') return 62;
    if(c == '/') return 63;
    return -1;
}

static uint32_t b64_decode(const char* src, uint32_t srclen, uint8_t* dst)
{
    uint32_t i, j = 0;
    for(i = 0; i + 3 < srclen; i += 4)
    {
        int8_t a = b64_decode_byte(src[i]);
        int8_t b = b64_decode_byte(src[i + 1]);
        int8_t c = b64_decode_byte(src[i + 2]);
        int8_t d = b64_decode_byte(src[i + 3]);
        if(a < 0 || b < 0) break;
        dst[j++] = (a << 2) | (b >> 4);
        if(src[i + 2] != '=' && c >= 0) dst[j++] = (b << 4) | (c >> 2);
        if(src[i + 3] != '=' && d >= 0) dst[j++] = (c << 6) | d;
    }
    return j;
}

/* ── EUI-64 from WiFi MAC ─────────────────────────────────────────────── */
static uint8_t s_eui64[8] = {0};

static void build_eui64(void)
{
    TCPIP_NET_HANDLE hNet = TCPIP_STACK_NetHandleGet("MRF24WN");
    if(hNet == NULL)
        return;
    const uint8_t* mac = TCPIP_STACK_NetAddressMac(hNet);
    if(mac == NULL)
        return;
    s_eui64[0] = mac[0];
    s_eui64[1] = mac[1];
    s_eui64[2] = mac[2];
    s_eui64[3] = 0xFF;
    s_eui64[4] = 0xFE;
    s_eui64[5] = mac[3];
    s_eui64[6] = mac[4];
    s_eui64[7] = mac[5];
}

void APP_UDP_GetEUI64String(char out[17])
{
    if(s_eui64[0] == 0 && s_eui64[1] == 0 && s_eui64[2] == 0)
        build_eui64();
    snprintf(out, 17, "%02X%02X%02X%02X%02X%02X%02X%02X",
             s_eui64[0], s_eui64[1], s_eui64[2], s_eui64[3],
             s_eui64[4], s_eui64[5], s_eui64[6], s_eui64[7]);
}

/* ── random token (xorshift16, seeded at init) ────────────────────────── */
static uint16_t s_token_state = 0xACE1u;
static uint16_t next_token(void)
{
    s_token_state ^= (uint16_t)(s_token_state << 7);
    s_token_state ^= (uint16_t)(s_token_state >> 9);
    s_token_state ^= (uint16_t)(s_token_state << 8);
    return s_token_state;
}

/* ── coderate string ──────────────────────────────────────────────────── */
static const char* coderate_str(uint8_t cr)
{
    switch(cr)
    {
        case 1: return "4/5";
        case 2: return "4/6";
        case 3: return "4/7";
        case 4: return "4/8";
        default: return "4/5";
    }
}

/* ── bandwidth string (for FSK or reference) ─────────────────────────── */
static uint32_t bw_khz(uint8_t bw)
{
    switch(bw)
    {
        case 0x01: return 500;  /* LORA_BW_500K */
        case 0x02: return 250;  /* LORA_BW_250K */
        case 0x03: return 125;  /* LORA_BW_125K */
        default:   return 125;
    }
}

/* ── Parse SF+BW string e.g. "SF7BW125" → modulation, datarate, bw ───── */
static void parse_datr(const char* s, uint8_t* mod, uint32_t* dr, uint8_t* bw_enum)
{
    if(s[0] == 'S' && s[1] == 'F')
    {
        *mod = 0x10; /* LORA modulation byte used by connector */
        *dr  = (uint32_t)atoi(s + 2);
        const char* bwp = strchr(s, 'B');
        uint32_t    bw  = bwp ? (uint32_t)atoi(bwp + 2) : 125;
        if(bw == 500)      *bw_enum = 0x03;
        else if(bw == 250) *bw_enum = 0x02;
        else               *bw_enum = 0x01;
    }
    else
    {
        *mod     = 0x20; /* FSK */
        *dr      = (uint32_t)atoi(s);
        *bw_enum = 0x01;
    }
}

/* ── JSMN helper: extract null-terminated token value ─────────────────── */
static void jsmn_str(const char* json, jsmntok_t* t, char* out, size_t maxlen)
{
    size_t len = (size_t)(t->end - t->start);
    if(len >= maxlen) len = maxlen - 1;
    memcpy(out, json + t->start, len);
    out[len] = '\0';
}

static int jsmn_streq(const char* json, jsmntok_t* t, const char* s)
{
    return (int)(strncmp(json + t->start, s, (size_t)(t->end - t->start)) == 0 &&
                 strlen(s) == (size_t)(t->end - t->start));
}

static uint32_t parse_freq_mhz(const char* s);

/* ── PULL_RESP parser ─────────────────────────────────────────────────── */
static bool parse_pull_resp(const uint8_t* buf, uint16_t len, loraTXPacket* pkt)
{
    if(len < 12) return false;
    /* skip 4-byte header (no EUI in PULL_RESP), then find JSON */
    const char* json = NULL;
    uint16_t    jlen = 0;
    /* PULL_RESP: 4 bytes header, then JSON */
    if(buf[3] != MSG_PULL_RESP) return false;
    json = (const char*)(buf + 4);
    jlen = len - 4;

    jsmn_parser  p;
    jsmntok_t    tok[64];
    int          r, txpk_obj, end, i;
    jsmn_init(&p);
    r = jsmn_parse(&p, json, jlen, tok, 64);
    if(r < 0) return false;

    /* find "txpk" object */
    txpk_obj = -1;
    for(i = 1; i < r - 1; i++)
    {
        if(tok[i].type == JSMN_STRING && jsmn_streq(json, &tok[i], "txpk"))
        {
            txpk_obj = i + 1;
            break;
        }
    }
    if(txpk_obj < 0) return false;

    memset(pkt, 0, sizeof(*pkt));
    pkt->tx_mode = 0;

    end = tok[txpk_obj].end;
    i   = txpk_obj + 1;
    while(i < r && tok[i].start < end)
    {
        if(tok[i].type != JSMN_STRING || i + 1 >= r)
        {
            i++;
            continue;
        }
        char val[256];
        jsmn_str(json, &tok[i + 1], val, sizeof(val));

        if(jsmn_streq(json, &tok[i], "tmst"))
        {
            pkt->timestamp = (uint32_t)strtoul(val, NULL, 10);
        }
        else if(jsmn_streq(json, &tok[i], "imme"))
        {
            if(val[0] == 't') pkt->tx_mode = 1; /* immediate */
        }
        else if(jsmn_streq(json, &tok[i], "freq"))
        {
            pkt->frequency = parse_freq_mhz(val);
        }
        else if(jsmn_streq(json, &tok[i], "rfch"))
        {
            pkt->rf_chain = (uint8_t)atoi(val);
        }
        else if(jsmn_streq(json, &tok[i], "powe"))
        {
            pkt->tx_power = (uint8_t)atoi(val);
        }
        else if(jsmn_streq(json, &tok[i], "modu"))
        {
            /* modulation set by datr parse below */
        }
        else if(jsmn_streq(json, &tok[i], "datr"))
        {
            parse_datr(val, &pkt->modulation, &pkt->datarate, &pkt->bandwidth);
        }
        else if(jsmn_streq(json, &tok[i], "codr"))
        {
            /* map "4/5" → 1 etc. */
            if(val[2] == '5')      pkt->coderate = 1;
            else if(val[2] == '6') pkt->coderate = 2;
            else if(val[2] == '7') pkt->coderate = 3;
            else if(val[2] == '8') pkt->coderate = 4;
        }
        else if(jsmn_streq(json, &tok[i], "ipol"))
        {
            pkt->invert_polarity = (val[0] == 't');
        }
        else if(jsmn_streq(json, &tok[i], "prea"))
        {
            pkt->preamble = (uint8_t)atoi(val);
        }
        else if(jsmn_streq(json, &tok[i], "ncrc") || jsmn_streq(json, &tok[i], "nocrc"))
        {
            pkt->no_crc = (val[0] == 't');
        }
        else if(jsmn_streq(json, &tok[i], "nhdr"))
        {
            pkt->no_header = (val[0] == 't');
        }
        else if(jsmn_streq(json, &tok[i], "size"))
        {
            pkt->payload_size = (uint16_t)atoi(val);
        }
        else if(jsmn_streq(json, &tok[i], "data"))
        {
            uint16_t vlen = (uint16_t)(tok[i + 1].end - tok[i + 1].start);
            uint32_t dec  = b64_decode(json + tok[i + 1].start, vlen, pkt->payload);
            if(pkt->payload_size == 0) pkt->payload_size = (uint16_t)dec;
        }
        i += 2;
    }
    return true;
}

/* ── frequency as exact "MHz" string ──────────────────────────────────── */
/* XC32 builds this firmware with 32-bit doubles, so "(double)hz / 1e6" with
 * "%.6f" prints values like 868.300048 instead of 868.300000. ChirpStack matches
 * "freq" exactly against the region's channel list and drops anything else
 * ("Unknown channel for frequency"); TTN tolerates it. Format with integer
 * arithmetic instead. The module value is rounded to 500 Hz as a safety net
 * (all LoRaWAN channel frequencies are multiples of 500 Hz). */
static void freq_mhz_str(uint32_t hz, char* out, size_t outsz)
{
    hz = ((hz + 250u) / 500u) * 500u;
    snprintf(out, outsz, "%lu.%06lu", (unsigned long)(hz / 1000000u),
             (unsigned long)(hz % 1000000u));
}

/* Parse "868.1" / "868.100000" to Hz without floating point. */
static uint32_t parse_freq_mhz(const char* s)
{
    uint32_t mhz = 0, frac = 0, digits = 0;
    while(*s >= '0' && *s <= '9') mhz = mhz * 10u + (uint32_t)(*s++ - '0');
    if(*s == '.')
    {
        s++;
        while(*s >= '0' && *s <= '9' && digits < 6)
        {
            frac = frac * 10u + (uint32_t)(*s++ - '0');
            digits++;
        }
        while(digits++ < 6) frac *= 10u;
    }
    return mhz * 1000000u + frac;
}

/* ── PUSH_DATA builder ────────────────────────────────────────────────── */
static uint16_t build_push_data(const loraRXPacket* pkt, uint8_t* buf, uint16_t bufsz)
{
    uint16_t token = next_token();

    /* header */
    buf[0] = SEMTECH_PROTO_VER;
    buf[1] = (uint8_t)(token & 0xFF);
    buf[2] = (uint8_t)(token >> 8);
    buf[3] = MSG_PUSH_DATA;
    memcpy(buf + 4, s_eui64, 8);

    /* JSON body */
    char    b64buf[350]; /* 256 bytes payload → 344 base64 chars */
    uint32_t b64len = b64_encoded_len(pkt->payload_size);
    if(b64len >= sizeof(b64buf)) b64len = sizeof(b64buf) - 1;
    b64_encode(pkt->payload, pkt->payload_size, b64buf);

    /* modulation and data rate string */
    char datr[16];
    if(pkt->modulation == 0x10)
    {
        unsigned sf;
        switch(pkt->datarate)
        {
            case 0x02: sf = 7;  break;
            case 0x04: sf = 8;  break;
            case 0x08: sf = 9;  break;
            case 0x10: sf = 10; break;
            case 0x20: sf = 11; break;
            default:   sf = 12; break;
        }
        snprintf(datr, sizeof(datr), "SF%uBW%u", sf, (unsigned)bw_khz(pkt->bandwidth));
    }
    else
        snprintf(datr, sizeof(datr), "%u", (unsigned)pkt->datarate);

    char freq_str[16];
    freq_mhz_str(pkt->frequency, freq_str, sizeof(freq_str));

    float rssi_f, snr_f;
    memcpy(&rssi_f, &pkt->rssi,        sizeof(float));
    memcpy(&snr_f,  &pkt->snr_average, sizeof(float));

    int8_t crc_stat = 0;
    if(pkt->pkt_status == 0x10)      crc_stat = 1;
    else if(pkt->pkt_status == 0x01) crc_stat = -1;

    /* time field — only include if SNTP has run (year >= 2020) */
    char time_field[48] = "";
    time_t now = time(NULL);
    if(now >= 1577836800L) /* 2020-01-01 00:00:00 UTC */
    {
        char timestr[32];
        struct tm* t = gmtime(&now);
        if(t)
        {
            strftime(timestr, sizeof(timestr), "%Y-%m-%dT%H:%M:%SZ", t);
            snprintf(time_field, sizeof(time_field), "\"time\":\"%s\",", timestr);
        }
    }

    int json_len;
    if(pkt->modulation == 0x10) /* LORA */
    {
        json_len = snprintf(
            (char*)(buf + 12), bufsz - 12,
            "{\"rxpk\":[{"
            "%s"
            "\"tmst\":%lu,"
            "\"freq\":%s,"
            "\"chan\":%u,"
            "\"rfch\":%u,"
            "\"stat\":%d,"
            "\"modu\":\"LORA\","
            "\"datr\":\"%s\","
            "\"codr\":\"%s\","
            "\"rssi\":%d,"
            "\"lsnr\":%.1f,"
            "\"size\":%u,"
            "\"data\":\"%s\""
            "}]}",
            time_field,
            (unsigned long)pkt->timestamp,
            freq_str,
            (unsigned)pkt->if_chain,
            (unsigned)pkt->rf_chain,
            (int)crc_stat,
            datr,
            coderate_str(pkt->coderate),
            (int)rssi_f,
            (double)snr_f,
            (unsigned)pkt->payload_size,
            b64buf);
    }
    else /* FSK — datr is bitrate as JSON number, no codr */
    {
        json_len = snprintf(
            (char*)(buf + 12), bufsz - 12,
            "{\"rxpk\":[{"
            "%s"
            "\"tmst\":%lu,"
            "\"freq\":%s,"
            "\"chan\":%u,"
            "\"rfch\":%u,"
            "\"stat\":%d,"
            "\"modu\":\"FSK\","
            "\"datr\":%lu,"
            "\"rssi\":%d,"
            "\"size\":%u,"
            "\"data\":\"%s\""
            "}]}",
            time_field,
            (unsigned long)pkt->timestamp,
            freq_str,
            (unsigned)pkt->if_chain,
            (unsigned)pkt->rf_chain,
            (int)crc_stat,
            (unsigned long)pkt->datarate,
            (int)rssi_f,
            (unsigned)pkt->payload_size,
            b64buf);
    }

    if(json_len < 0 || (json_len + 12) > (int)bufsz) return 0;
    return (uint16_t)(12 + json_len);
}

/* ── packet counters for stat report ─────────────────────────────────── */
static uint32_t s_rxnb       = 0;
static uint32_t s_rxok       = 0;
static uint32_t s_rxfw       = 0;
static uint32_t s_dwnb       = 0;
static uint32_t s_txnb       = 0;
static uint32_t s_push_sent  = 0;
static uint32_t s_push_acked = 0;

/* ── STAT PUSH_DATA builder ───────────────────────────────────────────── */
static uint16_t build_stat_push_data(uint8_t* buf, uint16_t bufsz)
{
    uint16_t token = next_token();

    buf[0] = SEMTECH_PROTO_VER;
    buf[1] = (uint8_t)(token & 0xFF);
    buf[2] = (uint8_t)(token >> 8);
    buf[3] = MSG_PUSH_DATA;
    memcpy(buf + 4, s_eui64, 8);

    char timestr[32];
    time_t now = time(NULL);
    if(now >= 1577836800L)
    {
        struct tm* t = gmtime(&now);
        if(t)
            snprintf(timestr, sizeof(timestr), "%04d-%02d-%02d %02d:%02d:%02d GMT",
                     t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                     t->tm_hour, t->tm_min, t->tm_sec);
        else
            snprintf(timestr, sizeof(timestr), "1970-01-01 00:00:00 GMT");
    }
    else
        snprintf(timestr, sizeof(timestr), "1970-01-01 00:00:00 GMT");

    double ackr = (s_push_sent > 0)
                  ? ((double)s_push_acked * 100.0 / (double)s_push_sent)
                  : 100.0;

    int json_len = snprintf((char*)(buf + 12), bufsz - 12,
        "{\"stat\":{"
        "\"time\":\"%s\","
        "\"rxnb\":%lu,"
        "\"rxok\":%lu,"
        "\"rxfw\":%lu,"
        "\"ackr\":%.1f,"
        "\"dwnb\":%lu,"
        "\"txnb\":%lu,"
        "\"pfrm\":\"TTN Gateway\""
        "}}",
        timestr,
        (unsigned long)s_rxnb,
        (unsigned long)s_rxok,
        (unsigned long)s_rxfw,
        ackr,
        (unsigned long)s_dwnb,
        (unsigned long)s_txnb);

    if(json_len < 0 || (json_len + 12) > (int)bufsz) return 0;
    return (uint16_t)(12 + json_len);
}

/* ── PULL_DATA builder ────────────────────────────────────────────────── */
static uint16_t build_pull_data(uint8_t* buf)
{
    uint16_t token = next_token();
    buf[0] = SEMTECH_PROTO_VER;
    buf[1] = (uint8_t)(token & 0xFF);
    buf[2] = (uint8_t)(token >> 8);
    buf[3] = MSG_PULL_DATA;
    memcpy(buf + 4, s_eui64, 8);
    return 12;
}

/* ── TX_ACK builder ───────────────────────────────────────────────────── */
static uint16_t build_tx_ack(uint16_t token, const char* error, uint8_t* buf, uint16_t bufsz)
{
    buf[0] = SEMTECH_PROTO_VER;
    buf[1] = (uint8_t)(token & 0xFF);
    buf[2] = (uint8_t)(token >> 8);
    buf[3] = MSG_TX_ACK;
    memcpy(buf + 4, s_eui64, 8);
    int n = snprintf((char*)(buf + 12), bufsz - 12,
                     "{\"txpk_ack\":{\"error\":\"%s\"}}", error ? error : "NONE");
    if(n < 0) n = 0;
    return (uint16_t)(12 + n);
}

/* ── DNS pump (same pattern as app_http_request.c) ────────────────────── */
static int8_t pump_dns(const char* host, IPV4_ADDR* addr)
{
    IP_MULTI_ADDRESS mAddr;
    TCPIP_DNS_RESULT r = TCPIP_DNS_IsResolved(host, &mAddr, IP_ADDRESS_TYPE_IPV4);
    switch(r)
    {
        case TCPIP_DNS_RES_OK:
            addr->Val = mAddr.v4Add.Val;
            return 1;
        case TCPIP_DNS_RES_PENDING:
        case TCPIP_DNS_RES_SERVER_TMO:
            return 0;
        default:
            return -1;
    }
}

/* ── state machine ────────────────────────────────────────────────────── */
typedef enum
{
    APP_UDP_INIT = 0,
    APP_UDP_FETCH_SERVER_IP,
    APP_UDP_FETCHING_SERVER_IP,
    APP_UDP_OPEN_SOCKET,
    APP_UDP_OPERATIONAL,
    APP_UDP_DISABLED
} APP_UDP_STATE;

static APP_UDP_STATE s_state         = APP_UDP_INIT;
static UDP_SOCKET    s_sock          = INVALID_UDP_SOCKET; /* port_up */
static UDP_SOCKET    s_sock_down     = INVALID_UDP_SOCKET; /* port_down */
static IPV4_ADDR     s_server_ip     = {0};
static bool          s_connected     = false;
static uint32_t      s_last_pull_ack = 0;
static uint32_t      s_last_pull_tx  = 0;
static uint32_t      s_last_stat_tx  = 0;
static uint32_t      s_dns_timeout   = 0;


extern UDP_GW_CONF g_udp_gw_conf;

void APP_UDP_Initialize(void)
{
    s_state       = APP_UDP_INIT;
    s_sock        = INVALID_UDP_SOCKET;
    s_sock_down   = INVALID_UDP_SOCKET;
    s_connected   = false;
    s_token_state = (uint16_t)(SYS_TMR_TickCountGet() ^ 0xACE1u);
    build_eui64();
}

void APP_UDP_Reset(void)
{
    if(s_sock != INVALID_UDP_SOCKET)
    {
        TCPIP_UDP_Close(s_sock);
        s_sock = INVALID_UDP_SOCKET;
    }
    if(s_sock_down != INVALID_UDP_SOCKET && s_sock_down != s_sock)
    {
        TCPIP_UDP_Close(s_sock_down);
        s_sock_down = INVALID_UDP_SOCKET;
    }
    s_sock_down = INVALID_UDP_SOCKET;
    s_connected = false;
    s_state     = APP_UDP_INIT;
}

bool APP_UDP_IsConnected(void)
{
    return s_connected;
}

// Uplinks sent to the UDP server and downlinks accepted from it, since boot
void APP_UDP_GetPacketCount(uint32_t* pup, uint32_t* pdown)
{
    *pup   = s_rxfw;
    *pdown = s_txnb;
}

static void handle_pull_resp(uint8_t* buf, uint16_t rlen, uint8_t* txbuf, uint16_t txbufsz)
{
    if(g_udp_gw_conf.uplink_only) return;
    loraTXPacket txpkt;
    uint16_t pull_token = (uint16_t)(buf[1] | ((uint16_t)buf[2] << 8));
    const char* err = parse_pull_resp(buf, rlen, &txpkt) ? "NONE" : "LATE";
    if(err[0] == 'N') { s_dwnb++; enqueueLoRaTX(&txpkt); s_txnb++; }
    SYS_PRINT("UDP: PULL_RESP %s\r\n", (err[0] == 'N') ? "accepted" : "rejected (LATE)");
    uint16_t acklen = build_tx_ack(pull_token, err, txbuf, txbufsz);
    if(acklen > 0 && TCPIP_UDP_PutIsReady(s_sock) >= acklen)
    { TCPIP_UDP_ArrayPut(s_sock, txbuf, acklen); TCPIP_UDP_Flush(s_sock); }
}

static void recv_upstream(uint8_t* rxbuf, uint16_t rxbufsz)
{
    uint16_t avail = TCPIP_UDP_GetIsReady(s_sock);
    while(avail > 0)
    {
        uint16_t rlen = (avail > rxbufsz) ? rxbufsz : avail;
        TCPIP_UDP_ArrayGet(s_sock, rxbuf, rlen);
        avail = TCPIP_UDP_GetIsReady(s_sock);
        if(rlen < 4 || rxbuf[0] != SEMTECH_PROTO_VER) continue;
        if(rxbuf[3] == MSG_PUSH_ACK)
        {
            s_push_acked++;
            SYS_PRINT("UDP: PUSH_ACK\r\n");
        }
        /* single-socket mode: PULL_ACK and PULL_RESP also arrive here */
        if(s_sock_down == s_sock)
        {
            uint8_t txbuf[UDP_TX_BUFFER_SIZE];
            if(rxbuf[3] == MSG_PULL_ACK)
            {
                SYS_PRINT("UDP: PULL_ACK\r\n");
                s_connected = true; s_last_pull_ack = SYS_TMR_TickCountGet();
            }
            else if(rxbuf[3] == MSG_PULL_RESP)
                handle_pull_resp(rxbuf, rlen, txbuf, sizeof(txbuf));
        }
    }
}

static void recv_downstream(uint8_t* rxbuf, uint16_t rxbufsz)
{
    if(s_sock_down == s_sock) return; /* handled in recv_upstream */
    uint16_t avail = TCPIP_UDP_GetIsReady(s_sock_down);
    while(avail > 0)
    {
        uint16_t rlen = (avail > rxbufsz) ? rxbufsz : avail;
        TCPIP_UDP_ArrayGet(s_sock_down, rxbuf, rlen);
        avail = TCPIP_UDP_GetIsReady(s_sock_down);
        if(rlen < 4 || rxbuf[0] != SEMTECH_PROTO_VER) continue;
        uint8_t txbuf[UDP_TX_BUFFER_SIZE];
        if(rxbuf[3] == MSG_PULL_ACK)
        {
            SYS_PRINT("UDP: PULL_ACK\r\n");
            s_connected = true; s_last_pull_ack = SYS_TMR_TickCountGet();
        }
        else if(rxbuf[3] == MSG_PULL_RESP)
            handle_pull_resp(rxbuf, rlen, txbuf, sizeof(txbuf));
    }
}

void APP_UDP_Tasks(void)
{
    uint8_t txbuf[UDP_TX_BUFFER_SIZE];
    uint8_t rxbuf[UDP_RX_BUFFER_SIZE];

    switch(s_state)
    {
        /* ── INIT: check config ──────────────────────────────────────── */
        case APP_UDP_INIT:
        {
            if(!g_udp_gw_conf.enabled || g_udp_gw_conf.server_address[0] == '\0')
            {
                s_state = APP_UDP_DISABLED;
                break;
            }
            s_state = APP_UDP_FETCH_SERVER_IP;
            break;
        }

        /* ── FETCH_SERVER_IP: resolve or parse IP ─────────────────────── */
        case APP_UDP_FETCH_SERVER_IP:
        {
            TCPIP_DNS_RESULT r = TCPIP_DNS_Resolve(g_udp_gw_conf.server_address, TCPIP_DNS_TYPE_A);
            if(r == TCPIP_DNS_RES_NAME_IS_IPADDRESS)
            {
                TCPIP_Helper_StringToIPAddress(g_udp_gw_conf.server_address, &s_server_ip);
                s_state = APP_UDP_OPEN_SOCKET;
            }
            else if(r < 0)
            {
                s_state = APP_UDP_FETCH_SERVER_IP; /* retry next tick */
            }
            else
            {
                s_dns_timeout = SYS_TMR_TickCountGet();
                s_state       = APP_UDP_FETCHING_SERVER_IP;
            }
            break;
        }

        /* ── FETCHING_SERVER_IP: wait for DNS ────────────────────────── */
        case APP_UDP_FETCHING_SERVER_IP:
        {
            int8_t r = pump_dns(g_udp_gw_conf.server_address, &s_server_ip);
            if(r == 1)
            {
                s_state = APP_UDP_OPEN_SOCKET;
            }
            else if(r < 0)
            {
                s_state = APP_UDP_FETCH_SERVER_IP;
            }
            else if(SYS_TMR_TickCountGet() - s_dns_timeout >
                    (uint32_t)(5 * SYS_TMR_TickCounterFrequencyGet()))
            {
                SYS_DEBUG(SYS_ERROR_WARNING, "UDP: DNS timeout, retrying\r\n");
                s_state = APP_UDP_FETCH_SERVER_IP;
            }
            break;
        }

        /* ── OPEN_SOCKET ─────────────────────────────────────────────── */
        case APP_UDP_OPEN_SOCKET:
        {
            build_eui64(); /* WiFi is up by now — ensure EUI is populated */
            s_sock = TCPIP_UDP_ClientOpen(IP_ADDRESS_TYPE_IPV4, g_udp_gw_conf.port_up,
                                          (IP_MULTI_ADDRESS*)&s_server_ip);
            if(s_sock == INVALID_UDP_SOCKET)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "UDP: Failed to open upstream socket\r\n");
                s_state = APP_UDP_FETCH_SERVER_IP;
                break;
            }
            TCPIP_UDP_OptionsSet(s_sock, UDP_OPTION_TX_BUFF, (void*)UDP_TX_BUFFER_SIZE);

            /* open separate downstream socket if port_down differs from port_up */
            if(g_udp_gw_conf.port_down != 0 &&
               g_udp_gw_conf.port_down != g_udp_gw_conf.port_up)
            {
                s_sock_down = TCPIP_UDP_ClientOpen(IP_ADDRESS_TYPE_IPV4,
                                                   g_udp_gw_conf.port_down,
                                                   (IP_MULTI_ADDRESS*)&s_server_ip);
                if(s_sock_down == INVALID_UDP_SOCKET)
                {
                    SYS_DEBUG(SYS_ERROR_WARNING, "UDP: Failed to open downstream socket, using upstream\r\n");
                    s_sock_down = s_sock;
                }
                else
                {
                    TCPIP_UDP_OptionsSet(s_sock_down, UDP_OPTION_TX_BUFF, (void*)UDP_TX_BUFFER_SIZE);
                }
            }
            else
            {
                s_sock_down = s_sock; /* same port for up and down */
            }

            s_last_pull_tx  = SYS_TMR_TickCountGet() - UDP_PULL_INTERVAL_TICKS; /* send immediately */
            s_last_pull_ack = SYS_TMR_TickCountGet();
            s_connected     = false;
            s_state         = APP_UDP_OPERATIONAL;
            char eui_str[17];
            APP_UDP_GetEUI64String(eui_str);
            SYS_PRINT("UDP: socket open, server %s up:%u down:%u, EUI %s\r\n",
                      g_udp_gw_conf.server_address, g_udp_gw_conf.port_up,
                      g_udp_gw_conf.port_down, eui_str);
            break;
        }

        /* ── OPERATIONAL ─────────────────────────────────────────────── */
        case APP_UDP_OPERATIONAL:
        {
            uint32_t now = SYS_TMR_TickCountGet();

            /* ── detect disconnect ── */
            if((now - s_last_pull_ack) > (uint32_t)(UDP_CONNECTED_TIMEOUT * SYS_TMR_TickCounterFrequencyGet()))
            {
                s_connected = false;
            }

            /* ── PULL_DATA keepalive → downstream socket ── */
            if((now - s_last_pull_tx) >= (uint32_t)UDP_PULL_INTERVAL_TICKS)
            {
                uint16_t len = build_pull_data(txbuf);
                if(TCPIP_UDP_PutIsReady(s_sock_down) >= len)
                {
                    TCPIP_UDP_ArrayPut(s_sock_down, txbuf, len);
                    TCPIP_UDP_Flush(s_sock_down);
                    SYS_PRINT("UDP: PULL_DATA sent\r\n");
                }
                s_last_pull_tx = now;
            }

            /* ── send queued RX packets as PUSH_DATA → upstream socket ── */
            loraRXPacket rxpkt;
            while(hasUDPRXPacketInQueue() && dequeueUDPRX(&rxpkt))
            {
                s_rxnb++;
                if(rxpkt.pkt_status == 0x10) s_rxok++;
                uint16_t len = build_push_data(&rxpkt, txbuf, sizeof(txbuf));
                if(len > 0 && TCPIP_UDP_PutIsReady(s_sock) >= len)
                {
                    TCPIP_UDP_ArrayPut(s_sock, txbuf, len);
                    TCPIP_UDP_Flush(s_sock);
                    s_rxfw++;
                    s_push_sent++;
                    SYS_PRINT("UDP: PUSH_DATA sent (%u bytes)\r\n", len);
                }
                else if(len == 0)
                {
                    SYS_PRINT("UDP: PUSH_DATA dropped (build failed, payload too large?)\r\n");
                }
            }

            /* ── periodic gateway stat report → upstream socket ── */
            if((now - s_last_stat_tx) >= (uint32_t)UDP_STAT_INTERVAL_TICKS)
            {
                uint16_t len = build_stat_push_data(txbuf, sizeof(txbuf));
                if(len > 0 && TCPIP_UDP_PutIsReady(s_sock) >= len)
                {
                    TCPIP_UDP_ArrayPut(s_sock, txbuf, len);
                    TCPIP_UDP_Flush(s_sock);
                    s_push_sent++;
                    SYS_PRINT("UDP: stat sent (rxnb=%lu rxok=%lu rxfw=%lu ackr=%.0f%%)\r\n",
                              (unsigned long)s_rxnb, (unsigned long)s_rxok, (unsigned long)s_rxfw,
                              s_push_sent > 0 ? (double)s_push_acked * 100.0 / s_push_sent : 100.0);
                }
                s_last_stat_tx = now;
            }

            /* ── receive frames ── */
            recv_upstream(rxbuf, sizeof(rxbuf));
            recv_downstream(rxbuf, sizeof(rxbuf));
            break;
        }

        /* ── DISABLED: discard queued RX packets so they are not sent stale later ── */
        case APP_UDP_DISABLED:
        {
            loraRXPacket drop;
            while(dequeueUDPRX(&drop)) {}
            break;
        }

        default:
            s_state = APP_UDP_INIT;
            break;
    }
}
