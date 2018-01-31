/*******************************************************************************
 Source file for the Net Pres Encryption glue functions to work with Harmony


  Summary:


  Description:

 *******************************************************************************/

/*******************************************************************************
File Name: net_pres_enc_glue.c
Copyright (c) 2013 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

#include "net_pres_enc_glue.h"
#include "net/pres/net_pres_transportapi.h"
#include "net/pres/net_pres_certstore.h"

#include <time.h>
#include "crypto/src/random.h"

#include "app.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"

#if defined(MBEDTLS_PEM_PARSE_C)
#include "mbedtls/pem.h"

#endif

//#define NPEG_DEBUG_PRINT(fmt, ...) {SYS_PRINT(fmt, ##__VA_ARGS__);vTaskDelay(5 / portTICK_PERIOD_MS);}
#define NPEG_DEBUG_PRINT(fmt, ...) 

#define DEBUG_LEVEL 0

typedef union {
    uint64_t u64;
    uint32_t u32[2];
    uint8_t u8[8];
} epoche_t;

typedef struct {
    mbedtls_net_context server_fd;
    uint32_t flags;
    char *pers;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_ssl_context ssl;
    mbedtls_ssl_config conf;
    mbedtls_x509_crt cacert;
    char *server_name;
} mbedtls_context_t;
mbedtls_context_t mbed_ctx;

typedef struct {
    mbedtls_context_t *mbedtls_context;
    NET_PRES_TransportObject *transObject;
    bool isInited;
} net_pres_mbedTLSInfo;

int InitRng(RNG* rng) {
    return wc_InitRng(rng);
}

static void my_debug(void *ctx, int level,
        const char *file, int line,
        const char *str);

uint32_t Console_UART_GetWriteQueueElements(void);

/*******************************************************************************
 *        
 *                            Stream Client 0: mbedTLS
 * 
 ******************************************************************************/
// <editor-fold defaultstate="collapsed" desc="Stream Client 0 Functions">

NET_PRES_EncProviderObject net_pres_EncProviderStreamClient0 = 
{
    .fpInit =    NET_PRES_EncProviderStreamClientInit0,
    .fpDeinit =  NET_PRES_EncProviderStreamClientDeinit0,
    .fpOpen =    NET_PRES_EncProviderStreamClientOpen0,
    .fpConnect = NET_PRES_EncProviderClientConnect0,
    .fpClose =   NET_PRES_EncProviderConnectionClose0,
    .fpWrite =   NET_PRES_EncProviderWrite0,
    .fpWriteReady =   NET_PRES_EncProviderWriteReady0,
    .fpRead =    NET_PRES_EncProviderRead0,
    .fpReadReady = NET_PRES_EncProviderReadReady0,
    .fpPeek =    NET_PRES_EncProviderPeek0,
    .fpIsInited = NET_PRES_EncProviderStreamClientIsInited0,
};
net_pres_mbedTLSInfo net_pres_mbedTLSInfoStreamClient0;

int NET_PRES_EncGlue_StreamClientReceiveCb0(void *ctx, unsigned char *buf, size_t len) {
    uint16_t bufferSize;
    uint16_t ncount = 0;
    int fd = ((mbedtls_net_context *) ctx)->fd;

    if (fd < 0)
        return ( MBEDTLS_ERR_NET_INVALID_CONTEXT);

    uint16_t available = (*net_pres_mbedTLSInfoStreamClient0.transObject->fpReadyToRead)((uintptr_t) fd);
    NPEG_DEBUG_PRINT("TLS rx: want:%d,available:%d\r\n", len, available);
    if (available < len) {
        return MBEDTLS_ERR_SSL_WANT_READ;
    }

    bufferSize = (*net_pres_mbedTLSInfoStreamClient0.transObject->fpRead)((uintptr_t) fd, (uint8_t*) buf, len);
    return bufferSize;
}

int NET_PRES_EncGlue_StreamClientSendCb0(void *ctx, const unsigned char *buf, size_t len) {
    int fd = *(int *) ctx;
    uint16_t bufferSize;
    uint16_t readytowrite;

    readytowrite = (*net_pres_mbedTLSInfoStreamClient0.transObject->fpReadyToWrite)((uintptr_t) fd);
    if (readytowrite < len) {
        return MBEDTLS_ERR_SSL_WANT_WRITE;
    }
    
    bufferSize = (*net_pres_mbedTLSInfoStreamClient0.transObject->fpWrite)((uintptr_t) fd, (uint8_t*) buf, (uint16_t) len);

    NPEG_DEBUG_PRINT("TLS: want:%d,place:%d,written:%d\r\n", len, readytowrite, bufferSize);

    return bufferSize;
}

uint32_t ssl_init_flag = 0;

bool NET_PRES_EncProviderStreamClientInit0(NET_PRES_TransportObject * transObject)
{
    int ret = 0;

#if defined(MBEDTLS_DEBUG_C)
    mbedtls_debug_set_threshold(DEBUG_LEVEL);
#endif

    NPEG_DEBUG_PRINT(" mbedTLS NET_PRES_EncProviderStreamClientInit1\r\n");

    net_pres_mbedTLSInfoStreamClient0.transObject = transObject;
    net_pres_mbedTLSInfoStreamClient0.mbedtls_context = &mbed_ctx;
    
    mbed_ctx.pers = "ssl_client1";
    mbedtls_ssl_init(& mbed_ctx.ssl);
    
    mbedtls_ssl_config_init(& mbed_ctx.conf);
    mbedtls_x509_crt_init(& mbed_ctx.cacert);
    mbedtls_ctr_drbg_init(& mbed_ctx.ctr_drbg);
    
    NPEG_DEBUG_PRINT("\n  . Seeding the random number generator...\r\n");
    mbedtls_entropy_init(& mbed_ctx.entropy);

    ret = mbedtls_ctr_drbg_seed(& mbed_ctx.ctr_drbg, mbedtls_entropy_func, & mbed_ctx.entropy,
            (const unsigned char *) mbed_ctx.pers,
            strlen(mbed_ctx.pers));
    ASSERT(ret == 0, "mbedtls_ctr_drbg_seed returned %d", ret);

    mbedtls_ssl_set_bio(&mbed_ctx.ssl, &mbed_ctx.server_fd,
            NET_PRES_EncGlue_StreamClientSendCb0,
            NET_PRES_EncGlue_StreamClientReceiveCb0, NULL);

    NPEG_DEBUG_PRINT("  . Loading the CA root certificate ... ");
    const uint8_t * caCertsPtr;
    int32_t caCertsLen;
    
    int i = 0;
    while (NET_PRES_CertStoreGetCACerts(&caCertsPtr, &caCertsLen, i)) {
        ret = mbedtls_x509_crt_parse(&mbed_ctx.cacert, caCertsPtr, caCertsLen);
        ASSERT(ret == 0, "mbedtls_x509_crt_parse returned -0x%x", -ret);
        i++;
    }
    NPEG_DEBUG_PRINT("loaded %d certificates\r\n", i);
    
    return true;
}

NET_PRES_TransportObject My_transObject;
uint32_t check_flag = 0;

bool NET_PRES_EncProviderStreamClientDeinit0() 
{

    NPEG_DEBUG_PRINT(" mbedTLS NET_PRES_EncProviderStreamClientDeinit1\r\n");

    mbedtls_entropy_free(&mbed_ctx.entropy);
    mbedtls_ctr_drbg_free(&mbed_ctx.ctr_drbg);
    mbedtls_x509_crt_free(& mbed_ctx.cacert);
    mbedtls_ssl_config_free(&mbed_ctx.conf);

    net_pres_mbedTLSInfoStreamClient0.isInited = false;
    NPEG_DEBUG_PRINT(" mbedTLS Deinit Ready\r\n");

    return true;
}


bool open_flag = 0;

bool NET_PRES_EncProviderStreamClientOpen0(uintptr_t transHandle, void * providerData)
{
    bool ret = 0;

    NPEG_DEBUG_PRINT(" mbedTLS NET_PRES_EncProviderStreamClientOpen1 %d\r\n", transHandle);
    
    if (open_flag) {
        NPEG_DEBUG_PRINT("\r\n\r\n#### was already open, close first\r\n\r\n");
        NET_PRES_EncProviderConnectionClose0(providerData);
    }
    open_flag = true;

    (&mbed_ctx)->server_fd.fd = (int) transHandle;


    /*
     * Setup stuff
     */
    NPEG_DEBUG_PRINT("  . Setting up the SSL/TLS structure...");

    ret = mbedtls_ssl_config_defaults(&mbed_ctx.conf,
            MBEDTLS_SSL_IS_CLIENT,
            MBEDTLS_SSL_TRANSPORT_STREAM,
            MBEDTLS_SSL_PRESET_DEFAULT);
    ASSERT(ret == 0, "mbedtls_ssl_config_defaults returned %d", ret);

    NPEG_DEBUG_PRINT(" ok\r\n");


    mbedtls_ssl_conf_authmode(&mbed_ctx.conf, MBEDTLS_SSL_VERIFY_REQUIRED);
    mbedtls_ssl_conf_ca_chain(&mbed_ctx.conf, &mbed_ctx.cacert, NULL);
    mbedtls_ssl_conf_rng(&mbed_ctx.conf, mbedtls_ctr_drbg_random, & mbed_ctx.ctr_drbg);
    mbedtls_ssl_conf_dbg(&mbed_ctx.conf, my_debug, stdout);

    net_pres_mbedTLSInfoStreamClient0.isInited = true;

    (&mbed_ctx)->server_fd.fd = (int) transHandle;
    ret = mbedtls_ssl_setup(&mbed_ctx.ssl, &mbed_ctx.conf);
    ASSERT(ret == 0, "mbedtls_ssl_setup returned %d\r\n", ret);

    return true;
}

bool NET_PRES_EncProviderStreamClientIsInited0()
{
    return net_pres_mbedTLSInfoStreamClient0.isInited;
}

NET_PRES_EncSessionStatus NET_PRES_EncProviderClientConnect0(void * providerData)
{
    int ret;

    NPEG_DEBUG_PRINT(" mbedTLS NET_PRES_EncProviderClientConnect1\r\n");

    mbedtls_ssl_set_bio(&mbed_ctx.ssl, &mbed_ctx.server_fd,
            NET_PRES_EncGlue_StreamClientSendCb0,
            NET_PRES_EncGlue_StreamClientReceiveCb0, NULL);

    NPEG_DEBUG_PRINT("  . Performing the SSL/TLS handshake...");
    while ((ret = mbedtls_ssl_handshake(&mbed_ctx.ssl)) != 0) {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
            NPEG_DEBUG_PRINT(" failed\n\r  ! mbedtls_ssl_handshake returned -0x%x\r\n", -ret);
            NET_PRES_EncProviderStreamClientDeinit0();
            return ret;
        }
    }

    NPEG_DEBUG_PRINT("  . Verifying peer X.509 certificate...");

    if ((mbed_ctx.flags = mbedtls_ssl_get_verify_result(&mbed_ctx.ssl)) != 0) {
        char vrfy_buf[512];

        NPEG_DEBUG_PRINT(" failed\r\n");

        mbedtls_x509_crt_verify_info(vrfy_buf, sizeof ( vrfy_buf), "  ! ", mbed_ctx.flags);

        NPEG_DEBUG_PRINT("%s\r\n", vrfy_buf);
        return NET_PRES_ENC_SS_FAILED;
    } else {
        NPEG_DEBUG_PRINT(" ok\r\n");
        return NET_PRES_ENC_SS_OPEN;
    }
}

NET_PRES_EncSessionStatus NET_PRES_EncProviderConnectionClose0(void * providerData)
{

    NPEG_DEBUG_PRINT(" mbedTLS NET_PRES_EncProviderConnectionClose1\r\n");

    mbedtls_ssl_close_notify(&mbed_ctx.ssl);
    mbedtls_ssl_free(&mbed_ctx.ssl);
    
    open_flag = false;

    return NET_PRES_ENC_SS_CLOSED;
}

int32_t NET_PRES_EncProviderWrite0(void * providerData, const uint8_t * buffer, uint16_t size)
{
    int ret;
    NPEG_DEBUG_PRINT(" mbedTLS Write %d\r\n", size);
    ret = mbedtls_ssl_write(&mbed_ctx.ssl, buffer, size);
    if (ret < 0) {
        // Note that in NET_PRES_SocketWrite the return value is explicitly casted to an uint16_t. Truncate any negative number returned from mbedtls_ssl_write
        NPEG_DEBUG_PRINT(" mbedtls_ssl_write returned negative -%d (-0x%X)\r\n", -ret, -ret);
        ret = 0;
    } else if (ret != size) {
        NPEG_DEBUG_PRINT(" mbedtls_ssl_write returned %d, not same as size %d\r\n", ret, size);
    }
    return ret;
}

uint16_t NET_PRES_EncProviderWriteReady0(void * providerData, uint16_t reqSize, uint16_t minSize)
{
    return reqSize;
}

int32_t NET_PRES_EncProviderRead0(void * providerData, uint8_t * buffer, uint16_t size)
{
    int ret;
    NPEG_DEBUG_PRINT(" mbedTLS Read %d\r\n", size);
    ret = mbedtls_ssl_read(&mbed_ctx.ssl, buffer, size);
    if (ret < 0) {
        // Note that in NET_PRES_SocketRead the return value is explicitly casted to an uint16_t. Truncate any negative number returned from mbedtls_ssl_read
        NPEG_DEBUG_PRINT(" mbedtls_ssl_read returned negative -%d (-0x%X)\r\n", -ret, -ret);
        ret = 0;
    } else if (ret != size) {
        NPEG_DEBUG_PRINT(" mbedtls_ssl_read returned %d, not same as size %d\r\n", ret, size);
    }
    return ret;
}

int32_t NET_PRES_EncProviderReadReady0(void * providerData)
{
    unsigned char dummy;
    mbedtls_ssl_read(&mbed_ctx.ssl, &dummy, 0);
    return mbedtls_ssl_get_bytes_avail(&mbed_ctx.ssl);
}

int32_t NET_PRES_EncProviderPeek0(void * providerData, uint8_t * buffer, uint16_t size)
{
    FATAL("Not implemented: peek at data held by provider");
    return -1;
}

static void my_debug(void *ctx, int level,
        const char *file, int line,
        const char *str) {
    ((void) level);

    NPEG_DEBUG_PRINT("%s:%04d: %s\r\n", file, line, str);
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="UTC to Time Functions for NTP">

typedef struct {
    unsigned char second; // 0-59
    unsigned char minute; // 0-59
    unsigned char hour; // 0-23
    unsigned char day; // 1-31
    unsigned char month; // 1-12
    unsigned char year; // 0-99 (representing 2000-2099)
} date_time_t;


static unsigned short days[4][12] = {
    { 0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335},
    { 366, 397, 425, 456, 486, 517, 547, 578, 609, 639, 670, 700},
    { 731, 762, 790, 821, 851, 882, 912, 943, 974, 1004, 1035, 1065},
    {1096, 1127, 1155, 1186, 1216, 1247, 1277, 1308, 1339, 1369, 1400, 1430},
};

//
//unsigned int date_time_to_epoch(date_time_t* date_time)
//{
//    unsigned int second = date_time->second;  // 0-59
//    unsigned int minute = date_time->minute;  // 0-59
//    unsigned int hour   = date_time->hour;    // 0-23
//    unsigned int day    = date_time->day-1;   // 0-30
//    unsigned int month  = date_time->month-1; // 0-11
//    unsigned int year   = date_time->year;    // 0-99
//    return (((year/4*(365*4+1)+days[year%4][month]+day)*24+hour)*60+minute)*60+second;
//}

void epoch_to_date_time(date_time_t* date_time, unsigned int epoch) {
    /* The function epoch_to_date_time() does only work from 1.1.2000
     * And the NTC timestamp starts from the 1.1.1900
     * 
     * 2208988800 seconds between 1.1.1900 and 1.1.1970
     * 946684800  seconds between 1.1.1970 and 1.1.2000
     * 3155673600 seconds between 1.1.1900 and 1.1.2000
     * 
     * See: https://www.aelius.com/njh/unixtime/?y=1900&m=1&d=1&h=0&i=0&s=0     
     */

    /* 946684800 => seconds between 1.1.1970 and 1.1.2000 */
    epoch -= 946684800;

    date_time->second = epoch % 60;
    epoch /= 60;
    date_time->minute = epoch % 60;
    epoch /= 60;
    date_time->hour = epoch % 24;
    epoch /= 24;

    unsigned int years = epoch / (365 * 4 + 1)*4;
    epoch %= 365 * 4 + 1;

    unsigned int year;
    for (year = 3; year > 0; year--) {
        if (epoch >= days[year][0])
            break;
    }

    unsigned int month;
    for (month = 11; month > 0; month--) {
        if (epoch >= days[year][month])
            break;
    }

    date_time->year = years + year;
    date_time->month = month + 1;
    date_time->day = epoch - days[year][month] + 1;
}

void GetTimeString(char *str) {
    epoche_t MyEpoche;
    date_time_t MyTime;

    MyEpoche.u32[0] = TCPIP_SNTP_UTCSecondsGet();
    epoch_to_date_time(&MyTime, MyEpoche.u32[0]);
    sprintf(str, "%02d.%02d.%02d %02d:%02d:%02d", MyTime.day, MyTime.month, MyTime.year + 2000, MyTime.hour, MyTime.minute, MyTime.second);
}

int gettimeofday(void *pTime, void *p) {
    uint32_t LastUpdate = 0;
    TCPIP_SNTP_TimeStampGet((uint64_t*) pTime, &LastUpdate);
    return 0;
}

time_t time(time_t *t) {
    epoche_t my_epoche;
    my_epoche.u32[0] = TCPIP_SNTP_UTCSecondsGet();
    return my_epoche.u32[0];
}

struct tm * gmtime(const time_t *pTime) {
    date_time_t MyTime;
    epoche_t MyEpoche;
    static struct tm MyTimeStruct;

    MyEpoche.u32[0] = *pTime;
    epoch_to_date_time(&MyTime, MyEpoche.u32[0]);

    MyTimeStruct.tm_sec = MyTime.second;
    MyTimeStruct.tm_min = MyTime.minute;
    MyTimeStruct.tm_hour = MyTime.hour;
    MyTimeStruct.tm_mday = MyTime.day;
    MyTimeStruct.tm_mon = MyTime.month - 1;
    MyTimeStruct.tm_year = (MyTime.year + 2000) - 1900;

    return &MyTimeStruct;
}

// </editor-fold>
