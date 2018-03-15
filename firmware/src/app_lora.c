// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "app.h"
#include "subsystem_controller.h"
#include "app_lora.h"
#include "stdlib.h"
#include "helper_wdt.h"
#include "error_messages.h"

// These are only needed for the blocking delay loop.
#include "peripheral/wdt/plib_wdt.h"
#include "peripheral/tmr/plib_tmr.h"
#include "peripheral/reset/plib_reset.h"
#include "peripheral/int/plib_int.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */
#define _GET_BYTE(value, index) ((((uint32_t)value) >> (index)*8) & 0xFF)
#define _SET_BYTES_32BIT(b0, b1, b2, b3) \
    ((uint32_t)((uint8_t)b0) | ((uint8_t)b1) << 8 | ((uint8_t)b2) << 16 | ((uint8_t)b3) << 24)
#define _SET_BYTES_16BIT(b0, b1) ((uint16_t)((uint8_t)b0) | ((uint8_t)b1) << 8)

#define TXPACKET_HEADER_LENGTH 26
#define TXPACKET_PAYLOAD_MAX_LENGTH 247
#define MAX_TXPACKET_LENGTH (TXPACKET_HEADER_LENGTH + TXPACKET_PAYLOAD_MAX_LENGTH)
#define KICK_MODULE_AFTER_LAST_ACK_TIMEOUT 60
#define WAIT_FOR_INIT_COMPLETE_TIME_MS 7000
#define CHECK_FOR_COMMUNICATION_TIMEOUT_MS 10000
#define UART_ERROR_NO_DATA 0
#define UART_ERROR_INCOMPLETE_DATA 1
#define UART_ERROR_REJECTED_DATA 2
#define UART_ERROR_BROKEN_DATA 3
#define UART_NO_ERROR 4
#define UART_DEFAULT_BAUD 115200
#define UART_SAFE_BAUD 115200
#define INITIAL_COMMUNICATION_RETRY_COUNT 3 // retries before we tell module is not found

static bool                   allowed_to_start = false; // TODO: Can be part of APP_DATA_LORA when it made local
static APP_DATA_LORA          appData          = {0};
extern APP_GW_ACTIVATION_DATA appGWActivationData;

static SYS_TMR_HANDLE timeoutTimerHandle;
static TaskHandle_t   loraRxThreadHandle = NULL;

static uint32_t last_rx_timestamp          = 0;
static uint32_t lora_config_failed_counter = 0;
static bool     freqplan_correct           = 0;
static uint8_t  frequency_band             = 0;
static uint32_t baudSetting = UART_DEFAULT_BAUD;
static uint8_t initialCommunicationRetryCount;

QueueHandle_t xRXQueue, xTXQueue, xUARTRXQueue;

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

void    flushUart(DRV_HANDLE handle);
bool    readUart(DRV_HANDLE handle, uint8_t* msg, uint16_t maxlen);
uint8_t readUartBulk(DRV_HANDLE handle, uint8_t* msg, uint16_t maxlen);
void    initLora();
bool    configLora();
bool    setLeds(bool led0, bool led1, bool led2);
uint8_t getStatus(uint8_t command);
bool    startLora();
bool    stopLora();
uint8_t getVersion();
bool doSave(void);
bool doReset(void);
bool setBaudrate(uint32_t baud);
uint32_t getBaudrate(void);
bool    txAbort();
bool    sendRXReply(bool ack);
bool    getRXChain(uint8_t RFChain);
bool    getIFChain(uint8_t IFChain);
bool    configureRXChain(uint8_t RFChain, bool enable, uint32_t center_freq);
bool    configureIFChainX(uint8_t IFChannel, bool enable, uint8_t RFChain, int32_t offset_freq);
bool    configureIFChain8(bool enable, uint8_t RFChain, int32_t offset_freq, uint32_t bandwidth, uint8_t spread_factor);
bool    configureIFChain9(bool enable, uint8_t RFChain, int32_t offset_freq, uint32_t bandwidth, uint32_t datarate);
bool    sendPacket(loraTXPacket* pkt);
bool    sendCommand(uint8_t command, uint8_t* payload, uint16_t len);
bool    sendReply(uint8_t command, uint8_t* payload, uint16_t len);
bool    parsePacket(uint8_t* buffer, loraRXPacket* pkt);
bool    constructTXPacket(loraTXPacket* pkt, uint8_t* tx);
void    printRXPacket(loraRXPacket* rxpkt);
void    printTXPacket(loraTXPacket* txpkt);
static bool getSync();
static bool setSync(bool public);
static void _setState(APP_STATES_LORA newState);
static void restart_lora_configuration();
static uint32_t getPreferredBaud(void);
static void shortSpinloopDelay(void);

static uint32_t timeout_timer = 0;
static uint32_t lastAckTime   = 0;

#define TIMEOUTSTART (timeout_timer = SYS_TMR_TickCountGet())
#define TIMEOUT(X)                             \
    (timeout_timer > SYS_TMR_TickCountGet() || \
     ((SYS_TMR_TickCountGet() - timeout_timer) >= (SYS_TMR_TickCounterFrequencyGet() * X)))

#include "Harmony/MQTTHarmony.h"

// flushUart removes any pending bytes from the receive buffer.
void flushUart(DRV_HANDLE handle)
{
    bool flushing = false;
    uint8_t buffer[1];
    while(DRV_USART_Read(handle, buffer, 1) > 0)
    {
        if(!flushing)
        {
            SYS_DEBUG(SYS_ERROR_DEBUG, "LORA: flushing: ");
            flushing = true;
        }
        SYS_DEBUG(SYS_ERROR_DEBUG, "%02x ", buffer[0]);
    }
    if(flushing)
        SYS_DEBUG(SYS_ERROR_DEBUG, "\r\n");
}

// readUart reads bytes into msg. It keeps track of the number of characters received.
// If the number exceeds maxlen, then wraps around and begins to write to msg at
// beginning. Returns TRUE (1) if the entire user message has been received, or FALSE (0)
// if the end of the message has not been reached.
bool readUart(DRV_HANDLE handle, uint8_t* msg, uint16_t maxlen)
{
    static size_t recv  = 0; // number of characters received
    size_t        nread = 0; // number of bytes read
    TIMEOUTSTART;
    while((nread = DRV_USART_Read(handle, msg + recv, 1)) == 0)
    {
        if(TIMEOUT(2))
            return 0;
    }
    if(nread)
    { // if we have read one byte
        if(msg[recv] == LORA_CR)
        {             // check for carriage return
            recv = 0; // prepare to receive another string
            return 1; // indicate that the string is ready
        }
        else
        {
            recv += nread;
            if(recv >= maxlen)
            { // wrap around to the beginning
                recv = 0;
            }
            return 0;
        }
    }
    return 0;
}

uint8_t readUartBulk(DRV_HANDLE handle, uint8_t* msg, uint16_t maxlen)
{
    uint16_t nread = 0; // number of bytes read
    uint16_t len   = 0;

    nread = DRV_USART_Read(handle, msg, 1); // Read one byte to determine LORA_FRAME_START
    if(!nread)
        return UART_ERROR_NO_DATA;
    if(msg[0] != LORA_FRAME_START)
    {
        msg[0] = 0;
        return UART_ERROR_INCOMPLETE_DATA;
    }

    TIMEOUTSTART; // Start endless loop protection

    do
    {
        nread += DRV_USART_Read(handle, msg + nread, (4 - nread)); // Read three more bytes to determine length
        if(TIMEOUT(2))
        {
            msg[0] = 0;
            return UART_ERROR_INCOMPLETE_DATA;
        }

    } while(nread < 4);

    len = _SET_BYTES_16BIT(msg[2], msg[3]);
    if(len > 300)
        return UART_ERROR_REJECTED_DATA;

    SYS_DEBUG(SYS_ERROR_DEBUG, "LENGTH: %d\r\n", len);

    uint16_t brokentransmission = 0;
    uint16_t oldnread           = nread;
    TIMEOUTSTART;
    do
    {
        if((int)(len + 6) - (int)(nread) <= 0)
            break;
        nread += DRV_USART_Read(handle, msg + nread, (len + 6) - nread);
        if(nread == oldnread)
        {
            if(brokentransmission > 300 || TIMEOUT(2))
                return UART_ERROR_INCOMPLETE_DATA;
            else
                brokentransmission++;
            continue;
        }
        brokentransmission = 0;
        oldnread           = nread;
    } while(nread < (len + 6));

    uint32_t checksum   = 0;
    uint16_t checksum_i = 0;
    for(checksum_i = 0; checksum_i < (nread - 2); checksum_i++)
    {
        checksum += msg[checksum_i];
    }
    checksum = checksum & 0xFF;
    if(checksum == msg[nread - 2])
    {
        SYS_DEBUG(SYS_ERROR_DEBUG, "LORA: Checksum passed\r\n");
        return UART_NO_ERROR;
    }

    SYS_DEBUG(SYS_ERROR_WARNING, "LORA: Checksum failed\r\n");
    return UART_ERROR_BROKEN_DATA;
}

bool writeUart(DRV_HANDLE handle, uint8_t* msg, size_t pkt_len)
{
    size_t bytesProcessed = 0;
    TIMEOUTSTART;
    do
    {
        // Write data to the USART and use the return value to
        // update the source data pointer and pending bytes number.
        bytesProcessed += DRV_USART_Write(handle, (msg + bytesProcessed), (pkt_len - bytesProcessed));

        if(TIMEOUT(2))
            return false;
    } while(bytesProcessed < pkt_len);

    if(bytesProcessed == pkt_len)
    {
        return true;
    }
    return false;
}

void initLora(void)
{
    baudSetting = getPreferredBaud();
    initialCommunicationRetryCount = INITIAL_COMMUNICATION_RETRY_COUNT;
    appData.rxwrite_pos = 0;
    appData.rxread_pos  = 0;

    // RF ENABLE LOW
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_5);

    // RESET HIGH
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_6);

    // RESET LOW
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_6);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // RESET HIGH
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_6);
}

bool configLora(void)
{
    bool    status = 1;

    stopLora();

    frequency_band = getVersion();
    shortSpinloopDelay(); // FIXME: this blocking delay is a workaround for the synchronization issue with the LoRa module
    SYS_PRINT("LORA: version: %02X\r\n", frequency_band);
    uint32_t config_freq_band = appGWActivationData.configuration_sx1301.rfchain[0].freq;

    // Do a check to find out if the configuration matches with the hardware
    if(config_freq_band == 0 || (config_freq_band >= 900000000 && frequency_band == LORA_BAND_868) ||
       (config_freq_band < 900000000 && frequency_band == LORA_BAND_915))
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "LORA: ERROR: wrong frequency configuration, fallback on default\r\n");
        freqplan_correct = 0;
        ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_INVALID_FREQUENCY_PLAN);
    }
    else
        freqplan_correct = 1;

    if(freqplan_correct == 0)
    {
        /* Static test configuration */
        if(frequency_band == LORA_BAND_868)
        {
            SYS_DEBUG(SYS_ERROR_WARNING, "LORA: use default EU config\r\n");
            status *= configureRXChain(0, 1, 867500000);
            status *= configureRXChain(1, 1, 868500000);
            status *= configureIFChainX(0, 1, 1, -400000);
            status *= configureIFChainX(1, 1, 1, -200000);
            status *= configureIFChainX(2, 1, 1, 0);
            status *= configureIFChainX(3, 1, 0, -400000);
            status *= configureIFChainX(4, 1, 0, -200000);
            status *= configureIFChainX(5, 1, 0, 0);
            status *= configureIFChainX(6, 1, 0, 200000);
            status *= configureIFChainX(7, 1, 0, 400000);
            status *= configureIFChain8(1, 1, -200000, 250000, 7);
            status *= configureIFChain9(1, 1, 300000, 125000, 50000);
        }
        else if(frequency_band == LORA_BAND_915)
        {
            SYS_DEBUG(SYS_ERROR_WARNING, "LORA: use default US config\r\n");
            status *= configureRXChain(0, 1, 904200000);
            status *= configureRXChain(1, 1, 905000000);
            status *= configureIFChainX(0, 1, 0, -300000);
            status *= configureIFChainX(1, 1, 0, -100000);
            status *= configureIFChainX(2, 1, 0, 100000);
            status *= configureIFChainX(3, 1, 0, 300000);
            status *= configureIFChainX(4, 1, 1, -300000);
            status *= configureIFChainX(5, 1, 1, -100000);
            status *= configureIFChainX(6, 1, 1, 100000);
            status *= configureIFChainX(7, 1, 1, 300000);
            status *= configureIFChain8(1, 0, 400000, 500000, 8);
            status *= configureIFChain9(0, 0, 0, 0, 0);
        }
    }

    else
    {
        status *= configureRXChain(0, appGWActivationData.configuration_sx1301.rfchain[0].enable,
                                   appGWActivationData.configuration_sx1301.rfchain[0].freq);
        status *= configureRXChain(1, appGWActivationData.configuration_sx1301.rfchain[1].enable,
                                   appGWActivationData.configuration_sx1301.rfchain[1].freq);

        int i = 0;
        for(i = 0; i <= 7; i++)
        {
            status *= configureIFChainX(i, appGWActivationData.configuration_sx1301.ifchain[i].enable,
                                        appGWActivationData.configuration_sx1301.ifchain[i].radio,
                                        appGWActivationData.configuration_sx1301.ifchain[i].freqOffset);
        }

        status *= configureIFChain8(appGWActivationData.configuration_sx1301.ifchain[8].enable,
                                    appGWActivationData.configuration_sx1301.ifchain[8].radio,
                                    appGWActivationData.configuration_sx1301.ifchain[8].freqOffset,
                                    appGWActivationData.configuration_sx1301.ifchain[8].bandwidth,
                                    appGWActivationData.configuration_sx1301.ifchain[8].spread_factor);

        status *= configureIFChain9(appGWActivationData.configuration_sx1301.ifchain[9].enable,
                                    appGWActivationData.configuration_sx1301.ifchain[9].radio,
                                    appGWActivationData.configuration_sx1301.ifchain[9].freqOffset,
                                    appGWActivationData.configuration_sx1301.ifchain[9].bandwidth,
                                    appGWActivationData.configuration_sx1301.ifchain[9].datarate);
    }

    setSync(appGWActivationData.configuration_sx1301.lorawan_public);

    // RF ENABLE HIGH
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_5);

    /* Reset command for module */
    stopLora();
    startLora();
    SYS_DEBUG(SYS_ERROR_DEBUG, "LORA: configLora %s\r\n", status ? "OK" : "ERROR");
    return status;
}

bool setLeds(bool led0, bool led1, bool led2)
{
    uint8_t payload = ((led2 << 2) | (led1 << 1) | (led0));
    return sendCommand(LORA_COMMAND_SETLEDS, &payload, 1);
}

static bool getSync()
{
    return sendCommand(LORA_COMMAND_GETSYNC, 0, 0);
}

static bool setSync(bool public)
{
    uint8_t payload = public ? 0x34 : 0x12;
    return sendCommand(LORA_COMMAND_SETSYNC, &payload, 1);
}

uint8_t getStatus(uint8_t command)
{
    uint8_t status = 0;
    uint8_t pkt[6];
    pkt[0]              = LORA_FRAME_START;
    pkt[1]              = command;
    pkt[2]              = 0;
    pkt[3]              = 0;
    uint32_t checksum   = 0;
    uint8_t  checksum_i = 0;
    for(checksum_i = 0; checksum_i < 4; checksum_i++)
    {
        checksum += pkt[checksum_i];
    }
    checksum = checksum & 0xFF;
    pkt[4]   = (uint8_t)checksum;
    pkt[5]   = LORA_CR;

    if(!DRV_USART_Write(appData.USARTHandle, (uint8_t*)pkt, 6))
    {
        return 0;
    }
    TIMEOUTSTART;
    while(readUart(appData.USARTHandle, appData.rx_uart_buffer, UART_BUFF_LENGTH) == 0)
    {
        if(TIMEOUT(2))
            return 0;
    }
    if(appData.rx_uart_buffer[1] == command)
    {
        status = appData.rx_uart_buffer[4];
    }
#if 1
    SYS_DEBUG(SYS_ERROR_INFO, "LORA: recv_status_rpl: ");
    uint8_t j = 0;
    while(j < UART_BUFF_LENGTH && appData.rx_uart_buffer[j] != LORA_CR)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "%#x ", appData.rx_uart_buffer[j]);
        j++;
    }
    SYS_DEBUG(SYS_ERROR_INFO, "%#x\r\n", appData.rx_uart_buffer[j]);
#endif

    return status;
}

bool startLora()
{
    uint8_t payload = 0;
    return sendCommand(LORA_COMMAND_START, &payload, 1);
}

bool stopLora()
{
    uint8_t payload = 0;
    return sendCommand(LORA_COMMAND_STOP, &payload, 1);
}

uint8_t getVersion()
{
    uint8_t payload = 0;
    if(sendCommand(LORA_COMMAND_VERSION, &payload, 1))
    {
        // Return freq band. 868MHz = 1, 915MHz = 2;
        return appData.rx_uart_buffer[4];
    }

    return 0;
}

bool doSave(void)
{
    return sendCommand(LORA_COMMAND_SAVE, 0, 0);
}

bool doReset(void)
{
    return sendCommand(LORA_COMMAND_RESET, 0, 0);
}

bool setBaudrate(uint32_t baud)
{
    uint8_t bytes[4];
    bytes[0] = baud & 0xFF;
    bytes[1] = (baud >> 8) & 0xFF;
    bytes[2] = (baud >> 16) & 0xFF;
    bytes[3] = (baud >> 24) & 0xFF;
    return sendCommand(LORA_COMMAND_SETUART, bytes, 4);
}

uint32_t getBaudrate(void)
{
    uint32_t baud = 0;
    if (sendCommand(LORA_COMMAND_GETUART, 0, 0)) {
        baud = appData.rx_uart_buffer[4];
        baud |= appData.rx_uart_buffer[5] << 8;
        baud |= appData.rx_uart_buffer[6] << 16;
        baud |= appData.rx_uart_buffer[7] << 24;
        
        return baud;
    }
    
    return 0;
}

bool txAbort()
{
    uint8_t payload = 0;
    return sendCommand(LORA_COMMAND_TXABORT, &payload, 1);
}

bool sendRXReply(bool ack)
{
    uint8_t payload = (uint8_t)ack;
    lastAckTime     = SYS_TMR_TickCountGet();
    return sendReply(LORA_COMMAND_RECEIVE, &payload, 1);
}

bool getRXChain(uint8_t RFChain)
{
    uint8_t packet = RFChain;
    return sendCommand(LORA_COMMAND_RFCHAIN, &packet, 1);
}

bool getIFChain(uint8_t IFChain)
{
    uint8_t packet = IFChain;
    return sendCommand(LORA_COMMAND_IFCHAIN, &packet, 1);
}

bool configureRXChain(uint8_t RFChain, bool enable, uint32_t center_freq)
{
    // SYS_CONSOLE_PRINT("\r\nRF: %d,%d,%d", RFChain, enable, center_freq);

    uint8_t payload[6] = {0};
    payload[0]         = RFChain;
    payload[1]         = (uint8_t)enable;
    payload[2]         = _GET_BYTE(center_freq, 0);
    payload[3]         = _GET_BYTE(center_freq, 1);
    payload[4]         = _GET_BYTE(center_freq, 2);
    payload[5]         = _GET_BYTE(center_freq, 3);
    return sendCommand(LORA_COMMAND_RFCONFIG, payload, 6);
}

bool configureIFChainX(uint8_t IFChannel, bool enable, uint8_t RFChain, int32_t offset_freq)
{
    // SYS_CONSOLE_PRINT("\r\nIF: %d,%d,%d,%d", IFChannel, enable, RFChain, offset_freq);

    uint8_t payload[7] = {0};
    payload[0]         = IFChannel;
    payload[1]         = (uint8_t)enable;
    payload[2]         = RFChain;
    payload[3]         = _GET_BYTE(offset_freq, 0);
    payload[4]         = _GET_BYTE(offset_freq, 1);
    payload[5]         = _GET_BYTE(offset_freq, 2);
    payload[6]         = _GET_BYTE(offset_freq, 3);

    return sendCommand(LORA_COMMAND_IFCONFIG, payload, 7);
}

bool configureIFChain8(bool enable, uint8_t RFChain, int32_t offset_freq, uint32_t bandwidth, uint8_t spread_factor)
{
    // SYS_CONSOLE_PRINT("\r\nIF8: %d,%d,%d,%d,%d", enable, RFChain, offset_freq, bandwidth, spread_factor);
    uint8_t payload[8] = {0};
    payload[0]         = (uint8_t)enable;
    payload[1]         = RFChain;
    if(payload[0] != 0)
    {
        payload[2] = _GET_BYTE(offset_freq, 0);
        payload[3] = _GET_BYTE(offset_freq, 1);
        payload[4] = _GET_BYTE(offset_freq, 2);
        payload[5] = _GET_BYTE(offset_freq, 3);

        switch(bandwidth)
        {
            case 500000:
                payload[6] = 0x01;
                break;
            case 250000:
                payload[6] = 0x02;
                break;
            case 125000:
                payload[6] = 0x03;
                break;
            default:
                payload[6] = 0x02;
                break;
        }

        uint8_t spreads[] = {
            LORA_SPREAD_SF7, LORA_SPREAD_SF8, LORA_SPREAD_SF9, LORA_SPREAD_SF10, LORA_SPREAD_SF11, LORA_SPREAD_SF12,
        };

        if(spread_factor >= 7 && spread_factor <= 12)
        {
            payload[7] = spreads[spread_factor - 7];
        }
        else
        {
            payload[7] = LORA_SPREAD_SF7;
        }
    }
    else
    {
        memset(&payload[2], 0x00, 6);
    }
    return sendCommand(LORA_COMMAND_IF8CONFIG, payload, 8);
}

bool configureIFChain9(bool enable, uint8_t RFChain, int32_t offset_freq, uint32_t bandwidth, uint32_t datarate)
{
    //  SYS_CONSOLE_PRINT("\r\nIF9: %d,%d,%d,%d,%d", enable, RFChain, offset_freq, bandwidth, datarate);
    uint8_t payload[11] = {0};
    payload[0]          = (uint8_t)enable;
    payload[1]          = RFChain;
    if(payload[0] != 0)
    {
        payload[2] = _GET_BYTE(offset_freq, 0);
        payload[3] = _GET_BYTE(offset_freq, 1);
        payload[4] = _GET_BYTE(offset_freq, 2);
        payload[5] = _GET_BYTE(offset_freq, 3);

        switch(bandwidth)
        {
            case 500000:
                payload[6] = 0x01;
                break;
            case 250000:
                payload[6] = 0x02;
                break;
            case 125000:
                payload[6] = 0x03;
                break;
            default:
                payload[6] = 0x02;
                break;
        }

        payload[7]  = _GET_BYTE(datarate, 0);
        payload[8]  = _GET_BYTE(datarate, 1);
        payload[9]  = _GET_BYTE(datarate, 2);
        payload[10] = _GET_BYTE(datarate, 3);
    }
    else
    {
        memset(&payload[2], 0x00, 9);
    }
    return sendCommand(LORA_COMMAND_IF9CONFIG, payload, 11);
}

bool sendPacket(loraTXPacket* txpkt)
{
    // txAbort();
    bool    status = 1;
    uint8_t data_payload[MAX_TXPACKET_LENGTH];

    status *= constructTXPacket(txpkt, data_payload);

    if(status == 1)
    {
        status *= sendCommand(LORA_COMMAND_SEND, data_payload, txpkt->payload_size + TXPACKET_HEADER_LENGTH);
    }

    return status;
}

bool sendCommand(uint8_t command, uint8_t* payload, uint16_t len)
{  
    // FIXME: this blocking delay is a workaround for the synchronization issue with the LoRa module
    // a magic delay before sending the command, if not some modules refuse to work
    shortSpinloopDelay();
   
    // flush Lora UART RX before sending any command
    if (appData.state != APP_LORA_GO_ASYNC && appData.state != APP_LORA_POLL_UART)
    {
        // only flush when not in ASYNC mode
        flushUart(appData.USARTHandle);
    }

    bool     gotresponse   = false;
    uint16_t packet_length = len + 6;
    uint8_t  pkt[packet_length + 1];
    pkt[0]     = LORA_FRAME_START;
    pkt[1]     = command;
    pkt[2]     = _GET_BYTE(len, 0);
    pkt[3]     = _GET_BYTE(len, 1);
    uint16_t i = 0;
    for(i = 0; i < len; i++)
    {
        pkt[4 + i] = payload[i];
    }

    uint32_t checksum   = 0;
    uint16_t checksum_i = 0;
    for(checksum_i = 0; checksum_i < (packet_length - 2); checksum_i++)
    {
        checksum += pkt[checksum_i];
    }
    checksum   = checksum & 0xFF;
    pkt[4 + i] = (uint8_t)checksum;
    pkt[5 + i] = LORA_CR;

#if 0
    SYS_DEBUG(SYS_ERROR_DEBUG, "\r\n\r\nLORA: send_cmd: ");
    for(i = 0; i < packet_length; i++)
    {
        SYS_DEBUG(SYS_ERROR_DEBUG, "%#x ", pkt[i]);
    }
    SYS_DEBUG(SYS_ERROR_DEBUG, "\r\n");
#endif

    if(!writeUart(appData.USARTHandle, pkt, packet_length))
    {
        SYS_DEBUG(SYS_ERROR_WARNING, "LORA: UART WRITE ERROR!\r\n");
        return false;
    }
    
    if(command != LORA_COMMAND_SEND)
    { // REVIEW: Why this exception?
        TIMEOUTSTART;
        while(readUart(appData.USARTHandle, appData.rx_uart_buffer, UART_BUFF_LENGTH) == 0)
        {
            if(TIMEOUT(1)) 
            {
                SYS_DEBUG(SYS_ERROR_WARNING, "LORA: UART TIMEOUT\r\n");
                return false;
            }
        }
        if(appData.rx_uart_buffer[1] == command)
        {
            gotresponse = true;
        }

#if 1
        SYS_DEBUG(SYS_ERROR_DEBUG, "LORA: recv_rpl: ");
        uint8_t j = 0;
        while(j < UART_BUFF_LENGTH && appData.rx_uart_buffer[j] != LORA_CR)
        {
            SYS_DEBUG(SYS_ERROR_DEBUG, "%#x ", appData.rx_uart_buffer[j]);
            j++;
        }
        SYS_DEBUG(SYS_ERROR_DEBUG, "%#x\r\n", appData.rx_uart_buffer[j]);
#endif
    }
    SYS_DEBUG(SYS_ERROR_DEBUG, "LORA: sendCommand %s\r\n", gotresponse ? "OK" : "ERROR");
    return gotresponse;
}

bool sendReply(uint8_t command, uint8_t* payload, uint16_t len)
{
    size_t  packet_length = len + 6;
    uint8_t pkt[packet_length];
    pkt[0]    = LORA_FRAME_START;
    pkt[1]    = command;
    pkt[2]    = _GET_BYTE(len, 0);
    pkt[3]    = _GET_BYTE(len, 1);
    uint8_t i = 0;
    for(i = 0; i < len; i++)
    {
        // if(payload[i] != NULL)
        pkt[4 + i] = payload[i];
        // else
        //    pkt[4 + i] = 0x00;
    }

    uint32_t checksum   = 0;
    uint8_t  checksum_i = 0;
    for(checksum_i = 0; checksum_i < (packet_length - 2); checksum_i++)
    {
        checksum += pkt[checksum_i];
    }
    checksum = checksum & 0xFF;
    // checksum = checksum % 0x100;
    pkt[4 + i] = (uint8_t)checksum;
    pkt[5 + i] = LORA_CR;

#if 0
    SYS_DEBUG(SYS_ERROR_DEBUG, "LORA: send_rpl: ");
    for(i = 0; i < packet_length; i++)
    {
        SYS_DEBUG(SYS_ERROR_DEBUG, "%#x ", pkt[i]);
    }
    SYS_DEBUG(SYS_ERROR_DEBUG, "\r\n");
#endif

    if(writeUart(appData.USARTHandle, pkt, packet_length))
    {
        return true;
    }
    return false;
}

bool parseRXPacket(uint8_t* rx, loraRXPacket* pkt)
{

    pkt->rx_status    = rx[0];
    pkt->frequency    = _SET_BYTES_32BIT(rx[1], rx[2], rx[3], rx[4]);
    pkt->if_chain     = rx[5];
    pkt->pkt_status   = rx[6];
    pkt->timestamp    = _SET_BYTES_32BIT(rx[7], rx[8], rx[9], rx[10]);
    pkt->rf_chain     = rx[11];
    pkt->modulation   = rx[12];
    pkt->bandwidth    = rx[13];
    pkt->datarate     = _SET_BYTES_32BIT(rx[14], rx[15], rx[16], rx[17]);
    pkt->coderate     = rx[18];
    pkt->rssi         = _SET_BYTES_32BIT(rx[19], rx[20], rx[21], rx[22]);
    pkt->snr_average  = _SET_BYTES_32BIT(rx[23], rx[24], rx[25], rx[26]);
    pkt->snr_minimum  = _SET_BYTES_32BIT(rx[27], rx[28], rx[29], rx[30]);
    pkt->snr_maximum  = _SET_BYTES_32BIT(rx[31], rx[32], rx[33], rx[34]);
    pkt->crc          = _SET_BYTES_16BIT(rx[36], rx[35]);
    pkt->payload_size = _SET_BYTES_16BIT(rx[37], rx[38]);
    uint8_t i         = 0;
    for(i = 0; i < pkt->payload_size; i++)
    {
        pkt->payload[i] = rx[39 + i];
    }
}

bool constructTXPacket(loraTXPacket* txpkt, uint8_t* tx)
{
    // uint32_t fchan = ((double)txpkt->frequency/32000000.00)*524288.00;

    tx[0] = _GET_BYTE(txpkt->frequency, 0);
    tx[1] = _GET_BYTE(txpkt->frequency, 1);
    tx[2] = _GET_BYTE(txpkt->frequency, 2);
    tx[3] = _GET_BYTE(txpkt->frequency, 3);

    tx[4] = txpkt->tx_mode;

    tx[5] = _GET_BYTE(txpkt->timestamp, 0);
    tx[6] = _GET_BYTE(txpkt->timestamp, 1);
    tx[7] = _GET_BYTE(txpkt->timestamp, 2);
    tx[8] = _GET_BYTE(txpkt->timestamp, 3);

    tx[9]  = txpkt->rf_chain;
    tx[10] = txpkt->tx_power;
    tx[11] = txpkt->modulation;
    tx[12] = txpkt->bandwidth;

    tx[13] = _GET_BYTE(txpkt->datarate, 0);
    tx[14] = _GET_BYTE(txpkt->datarate, 1);
    tx[15] = _GET_BYTE(txpkt->datarate, 2);
    tx[16] = _GET_BYTE(txpkt->datarate, 3);

    // tx[16] = (4 / (4 + txpkt->coderate));
    tx[17] = txpkt->coderate;
    tx[18] = txpkt->invert_polarity;
    tx[19] = txpkt->frequency_deviation;

    tx[20] = _GET_BYTE(txpkt->preamble, 0);
    tx[21] = _GET_BYTE(txpkt->preamble, 1);

    tx[22] = txpkt->no_crc;
    tx[23] = txpkt->no_header;

    tx[24] = _GET_BYTE(txpkt->payload_size, 0);
    tx[25] = _GET_BYTE(txpkt->payload_size, 1);

    uint8_t i = 0;
    for(i = 0; i < txpkt->payload_size; i++)
    {
        tx[26 + i] = txpkt->payload[i];
    }
    tx[26 + i] = '\0';
    return true;
}

void printRXPacket(loraRXPacket* rxpkt)
{
    SYS_DEBUG(SYS_ERROR_INFO, "\r\n==== PACKET ====\r\n");
    SYS_DEBUG(SYS_ERROR_INFO, "rxst: %#x\r\n", rxpkt->rx_status);
    SYS_DEBUG(SYS_ERROR_INFO, "freq: %u\r\n", rxpkt->frequency);
    SYS_DEBUG(SYS_ERROR_INFO, "ifch: %u\r\n", rxpkt->if_chain);
    SYS_DEBUG(SYS_ERROR_INFO, "pkts: %u\r\n", rxpkt->pkt_status);
    SYS_DEBUG(SYS_ERROR_INFO, "time: %u\r\n", rxpkt->timestamp);
    SYS_DEBUG(SYS_ERROR_INFO, "rfch: %u\r\n", rxpkt->rf_chain);
    SYS_DEBUG(SYS_ERROR_INFO, "modu: %u\r\n", rxpkt->modulation);
    SYS_DEBUG(SYS_ERROR_INFO, "bdwt: %u\r\n", rxpkt->bandwidth);
    SYS_DEBUG(SYS_ERROR_INFO, "rate: %u\r\n", rxpkt->datarate);
    SYS_DEBUG(SYS_ERROR_INFO, "code: %u\r\n", rxpkt->coderate);
    SYS_DEBUG(SYS_ERROR_INFO, "rssi: %.2f\r\n", rxpkt->rssi);
    SYS_DEBUG(SYS_ERROR_INFO, "snra: %.2f\r\n", rxpkt->snr_average);
    SYS_DEBUG(SYS_ERROR_INFO, "snrm: %.2f\r\n", rxpkt->snr_minimum);
    SYS_DEBUG(SYS_ERROR_INFO, "snrM: %.2f\r\n", rxpkt->snr_maximum);
    SYS_DEBUG(SYS_ERROR_INFO, "crc: %u\r\n", rxpkt->crc);
    SYS_DEBUG(SYS_ERROR_INFO, "psiz: %u\r\n", rxpkt->payload_size);
    /*uint8_t i = 0;
    for(i = 0; i <= rxpkt->payload_size; i++)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "pckt: %#x\r\n", rxpkt->payload[i]);
    }*/
    SYS_DEBUG(SYS_ERROR_INFO, "==== /PACKET ===\r\n\n");
}

void printTXPacket(loraTXPacket* txpkt)
{
    SYS_DEBUG(SYS_ERROR_INFO, "\r\n==== TX PKT ====\r\n");
    SYS_DEBUG(SYS_ERROR_INFO, "freq: %u\r\n", txpkt->frequency);
    SYS_DEBUG(SYS_ERROR_INFO, "txmd: %u\r\n", txpkt->tx_mode);
    SYS_DEBUG(SYS_ERROR_INFO, "time: %u\r\n", txpkt->timestamp);
    SYS_DEBUG(SYS_ERROR_INFO, "rfch: %u\r\n", txpkt->rf_chain);
    SYS_DEBUG(SYS_ERROR_INFO, "txpw: %u\r\n", txpkt->tx_power);
    SYS_DEBUG(SYS_ERROR_INFO, "modu: %u\r\n", txpkt->modulation);
    SYS_DEBUG(SYS_ERROR_INFO, "bdwt: %u\r\n", txpkt->bandwidth);
    SYS_DEBUG(SYS_ERROR_INFO, "rate: %u\r\n", txpkt->datarate);
    SYS_DEBUG(SYS_ERROR_INFO, "code: %u\r\n", txpkt->coderate);
    SYS_DEBUG(SYS_ERROR_INFO, "pola: %u\r\n", txpkt->invert_polarity);
    SYS_DEBUG(SYS_ERROR_INFO, "frdv: %u\r\n", txpkt->frequency_deviation);
    SYS_DEBUG(SYS_ERROR_INFO, "prea: %u\r\n", txpkt->preamble);
    SYS_DEBUG(SYS_ERROR_INFO, "ncrc: %u\r\n", txpkt->no_crc);
    SYS_DEBUG(SYS_ERROR_INFO, "nhdr: %u\r\n", txpkt->no_header);
    SYS_DEBUG(SYS_ERROR_INFO, "psiz: %u\r\n", txpkt->payload_size);
    /*uint8_t i = 0;
    for(i = 0; i <= txpkt->payload_size; i++)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "pckt: %#x\r\n", txpkt->payload[i]);
    }*/
    SYS_DEBUG(SYS_ERROR_INFO, "==== /TX PKT ===\r\n\n");
}

uint8_t hasLoraRXPacketInQueue(void)
{
    return uxQueueMessagesWaiting(xRXQueue);
}

uint8_t hasLoraTXPacketInQueue(void)
{
    return uxQueueMessagesWaiting(xTXQueue);
}

bool dequeueLoRaRX(loraRXPacket* pkt)
{
    return xQueueReceive(xRXQueue, pkt, 0) == pdTRUE ? true : false;
}

void enqueueLoRaRX(loraRXPacket* pkt)
{
    xQueueSend(xRXQueue, pkt, 0U);
}

bool dequeueLoRaTX(loraTXPacket* pkt)
{
    return xQueueReceive(xTXQueue, pkt, 0) == pdTRUE ? true : false;
}

void enqueueLoRaTX(loraTXPacket* pkt)
{
    xQueueSend(xTXQueue, pkt, 0U);
}

void APP_LORA_Initialize(void)
{
    allowed_to_start = false;
    /* Place the App state machine in its initial state. */
    _setState(APP_LORA_INIT);

    xRXQueue = xQueueCreate(5, sizeof(loraRXPacket));
    xTXQueue = xQueueCreate(5, sizeof(loraTXPacket));

    xUARTRXQueue = xQueueCreate(300, sizeof(uint8_t));
    // appData.state = APP_LORA_IDLE;
    //  LoraMutexInit();
}

void APP_LORA_SetStartEvent(void)
{
    if(loraRxThreadHandle != NULL)
    { // sanity check
        FATAL("LoRa can not be restarted when RX thread already exists");
    }
    // Set event that the module can be started. Don't set state directly!!
    allowed_to_start = true;
}

uint8_t APP_LORA_GET_APP_STATE(void)
{
    return appData.state;
}

/******************************************************************************
  Function:
    void APP_LORA_Tasks ( void )

  Remarks:
    None.
 */

uint8_t ndr = 0;
uint8_t nbw = 0;

uint8_t uartrx[300];

bool     gotuartrx_packet = false;
uint16_t uartrx_len;

static void restart_lora_configuration(void)
{
    SYS_DEBUG(SYS_ERROR_FATAL, "LORA: RESET MODULE\r\n");
    DRV_USART_Close(appData.USARTHandle);
    // TODO Verify untested fix. Here we power cycle the module as resetting (done in loraInit) the module was not
    // enough.
    loraSet(false);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    loraSet(true);
    _setState(APP_LORA_INIT);
}

void APP_LORA_Tasks(void)
{
    static loraRXPacket rxpkt = {0}, empty_rxpkt = {0};
    static loraTXPacket txpkt = {0}, empty_txpkt = {0};

    switch(appData.state)
    {
        case APP_LORA_INIT:
        {
            initLora();
            SYS_PRINT("LORA: Initialisation complete\r\n");
            timeoutTimerHandle = SYS_TMR_DelayMS(WAIT_FOR_INIT_COMPLETE_TIME_MS);
            _setState(APP_LORA_WAIT_INIT_COMPLETE);
            break;
        }

        case APP_LORA_WAIT_INIT_COMPLETE:
        {
            if(SYS_TMR_DelayStatusGet(timeoutTimerHandle))
            {
                SYS_ReInitializeUsart1(baudSetting);
                /* Open USART Driver instance 1 (USART1 in this case) and obtain a handle to it. */
                appData.USARTHandle = DRV_USART_Open(DRV_USART_INDEX_1, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING | DRV_IO_INTENT_EXCLUSIVE);
                ASSERT(appData.USARTHandle != DRV_HANDLE_INVALID, "Error opening LoRa UART");
                
                timeoutTimerHandle = SYS_TMR_DelayMS(CHECK_FOR_COMMUNICATION_TIMEOUT_MS); // Set timeout for initial communications
                SYS_PRINT("LORA: Wait init complete, check for communication on %d baud.\r\n", baudSetting);
                _setState(APP_LORA_CHECK_COMMUNICATION);
            }
            break;
        }
        case APP_LORA_CHECK_COMMUNICATION:
        {
            if (getVersion() != 0) {
                if (baudSetting != getPreferredBaud()) {
                    baudSetting = getPreferredBaud();
                    SYS_PRINT("LORA: Running on FRC, switching to %d\r\n", baudSetting);
                    timeoutTimerHandle = SYS_TMR_DelayMS(CHECK_FOR_COMMUNICATION_TIMEOUT_MS);
                    _setState(APP_LORA_SWITCH_BAUD);
                } else {
                    SYS_PRINT("LORA: Communication on %d OK, waiting for application.\r\n", baudSetting);
                    _setState(APP_LORA_WAIT_FOR_APP);
                }
            } else if (SYS_TMR_DelayStatusGet(timeoutTimerHandle)) {
                if (initialCommunicationRetryCount == 0) {
                    SYS_PRINT("LORA: LoRa module not found. Make sure the module is connected correctly.\r\n");
                    _setState(APP_LORA_NOT_FOUND);
                } else {
                    if (baudSetting == UART_DEFAULT_BAUD) {
                        baudSetting = UART_SAFE_BAUD;
                    } else {
                        baudSetting = UART_DEFAULT_BAUD;
                        initialCommunicationRetryCount--;
                    }

                    SYS_PRINT("LORA: Close and reopen UART on %d baud.\r\n", baudSetting);
                    DRV_USART_Close(appData.USARTHandle);
                    appData.USARTHandle = DRV_HANDLE_INVALID;
                    
                    SYS_TMR_CallbackStop(timeoutTimerHandle); // cancel pending timeout
                    timeoutTimerHandle = SYS_TMR_DelayMS(WAIT_FOR_INIT_COMPLETE_TIME_MS);
                    _setState(APP_LORA_WAIT_INIT_COMPLETE);
                }
            }
            break;
        }
        case APP_LORA_SWITCH_BAUD:
        {
            SYS_PRINT("LORA: Current baud: %d.\r\n", getBaudrate());
            if (setBaudrate(baudSetting)) {
                SYS_PRINT("LORA: Set baud succeeded.\r\n");
                SYS_PRINT("LORA: Current baud: %d.\r\n", getBaudrate());
                if (doSave()) {
                    doReset();
                    SYS_PRINT("LORA: Close and reopen UART on safe baud.\r\n");
                    DRV_USART_Close(appData.USARTHandle);
                    appData.USARTHandle = DRV_HANDLE_INVALID;
                    SYS_TMR_CallbackStop(timeoutTimerHandle); // cancel pending timeout
                    timeoutTimerHandle = SYS_TMR_DelayMS(WAIT_FOR_INIT_COMPLETE_TIME_MS);
                    _setState(APP_LORA_WAIT_INIT_COMPLETE);
                } else {
                    SYS_PRINT("LORA: Save failed.\r\n");                                    
                }
            } else {
                SYS_PRINT("LORA: Set baud failed.\r\n");                
            }
            if (SYS_TMR_DelayStatusGet(timeoutTimerHandle)) {
                SYS_PRINT("LORA: Failed to switch to %d baud rate.\r\n", baudSetting);
                _setState(APP_LORA_NOT_FOUND);
            }            
            break;
        }   
        case APP_LORA_WAIT_FOR_APP:
        {
            // Wait for the start event
            if(allowed_to_start)
            {
                _setState(APP_LORA_CONFIG);
            }
            break;
        }
        case APP_LORA_CONFIG:
        {
            SYS_PRINT("LORA: Starting reconfiguration\r\n");
            if(configLora())
            {
                SYS_PRINT("LORA: Configuration succeeded\r\n");
                SYS_PRINT("LORA: Starting operation\r\n");
                SYS_TMR_CallbackStop(timeoutTimerHandle); // cancel pending timeout
                timeoutTimerHandle = SYS_TMR_DelayMS(6000);
                _setState(APP_LORA_WAIT_TTN_CONFIG_COMPLETE);
            }
            else
            {
                ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_LORA_CONFIG_FAILURE);

                SYS_DEBUG(SYS_ERROR_WARNING, "LORA: Configuration failed, retry\r\n");
                lora_config_failed_counter++;
                if(lora_config_failed_counter > 2)
                {
                    lora_config_failed_counter = 0;
                    restart_lora_configuration();
                    break;
                }
            }
            break;
        }
        case APP_LORA_WAIT_TTN_CONFIG_COMPLETE:
        {
            if(SYS_TMR_DelayStatusGet(timeoutTimerHandle))
            {
                _setState(APP_LORA_GO_ASYNC);
            }

            break;
        }
        case APP_LORA_GO_ASYNC:
        {
            SYS_PRINT("LORA: GOING ASYNC\r\n");
            _setState(APP_LORA_POLL_UART);

            DRV_USART_Close(appData.USARTHandle);

            appData.USARTHandle = DRV_USART_Open(DRV_USART_INDEX_1, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_BLOCKING);

            appData.rxwrite_pos = 0;		
    	    appData.rxread_pos  = 0;	
            uint16_t    usTaskStackSize = configMINIMAL_STACK_SIZE;
            UBaseType_t uxTaskPriority  = TASK_PRIORITY_LORA_READ;

            xTaskCreate((TaskFunction_t)lora_read, "LORARX", usTaskStackSize, NULL, uxTaskPriority,
                        &loraRxThreadHandle); /* The task handle is not used. */

            break;
        }
        case APP_LORA_POLL_UART:
        {
            rxpkt = empty_rxpkt;

            if(SYS_TMR_TickCountGet() - lastAckTime >=
               SYS_TMR_TickCounterFrequencyGet() * KICK_MODULE_AFTER_LAST_ACK_TIMEOUT)
            {
                lastAckTime = SYS_TMR_TickCountGet();
                SYS_PRINT("LORA: Kick LoRa module with ACK after not acked it for %ds\r\n",
                          KICK_MODULE_AFTER_LAST_ACK_TIMEOUT);
                sendRXReply(LORA_ACK);
            }

            if(hasLoraTXPacketInQueue())
            {
                loraTXPacket send_packet = {0};
                dequeueLoRaTX(&send_packet);

                if(send_packet.timestamp > last_rx_timestamp)
                {
                    uint8_t txstatus = LORA_TX_STATUS_READY; // TODO getStatus(LORA_COMMAND_TXSTATUS);
                    // uint8_t txstatus = getStatus(LORA_COMMAND_TXSTATUS);
                    if(txstatus == LORA_TX_STATUS_READY || txstatus == LORA_TX_STATUS_LOADED)
                    {
                        sendPacket(&send_packet);
                        // SYS_DEBUG(SYS_ERROR_ERROR, "LORA: tx status: %u\r\n", txstatus);
                        // printTXPacket(send_packet);
                    }
                    else
                        SYS_DEBUG(SYS_ERROR_ERROR, "LORA: TX not ready, TX aborted, status: %u\r\n", txstatus);
                }
                else
                {
                    SYS_DEBUG(SYS_ERROR_ERROR, "LORA: TXPKT failed, too late! tx: %u rx: %u\r\n", send_packet.timestamp,
                              last_rx_timestamp);
                    ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_LORA_TX_TOO_LATE);
                }
                break;
            }

            if(!gotuartrx_packet && uxQueueMessagesWaiting(xUARTRXQueue) > 4)
            {
                uint8_t msg[1];
                if(xQueueReceive(xUARTRXQueue, msg, 0) != pdTRUE)
                    break;
                if(msg[0] != LORA_FRAME_START)
                    break;
                uartrx[0] = LORA_FRAME_START;

                if(xQueueReceive(xUARTRXQueue, msg, 0) != pdTRUE)
                    break;
                uartrx[1] = msg[0];

                if(xQueueReceive(xUARTRXQueue, msg, 0) != pdTRUE)
                    break;
                uartrx[2] = msg[0];

                if(xQueueReceive(xUARTRXQueue, msg, 0) != pdTRUE)
                    break;
                uartrx[3] = msg[0];

                uint16_t len = _SET_BYTES_16BIT(uartrx[2], uartrx[3]);
                if(len >= 295)
                {
                    sendRXReply(LORA_ACK);
                    break;
                }

                gotuartrx_packet = true;
                uartrx_len       = len;
                //   SYS_CONSOLE_PRINT("LORA: PKTL:%d\r\n",len);
            }

            if(gotuartrx_packet &&
               uxQueueMessagesWaiting(xUARTRXQueue) >= uartrx_len + 2) // +2 for checksum & frame end
            {
                uint16_t byte_counter = 0;
                while(byte_counter < (uartrx_len + 2))
                {
                    uint8_t msg[1];
                    if(xQueueReceive(xUARTRXQueue, msg, 0) != pdTRUE)
                    {
                        gotuartrx_packet = false;
                        uartrx_len       = 0;
                        sendRXReply(LORA_ACK);
                        break;
                    }
                    uartrx[4 + byte_counter] = msg[0];
                    byte_counter++;
                }
                //  SYS_CONSOLE_MESSAGE("LORA: PKT\r\n");
                uint16_t pktit = 0;
                for(pktit = 0; pktit < uartrx_len + 6; pktit++)
                {
                    //      SYS_CONSOLE_PRINT("%d\r\n",uartrx[pktit]);
                }
                uint32_t checksum   = 0;
                uint16_t checksum_i = 0;

                for(checksum_i = 0; checksum_i < (uartrx_len + 4); checksum_i++) // +4 for packet header type length
                {
                    checksum += uartrx[checksum_i];
                }
                checksum = checksum & 0xFF;
                if(checksum == uartrx[uartrx_len + 4])
                {

                    if(uartrx[1] != LORA_COMMAND_RECEIVE)
                    {
                        gotuartrx_packet = false;
                        memset(uartrx, 0, uartrx_len + 6);
                        uartrx_len = 0;
                        sendRXReply(LORA_ACK);
                        //    SYS_CONSOLE_MESSAGE("LORA: REGLOR\r\n");
                        //    SYS_CONSOLE_PRINT("LORA: INBUF:%d\r\n",uxQueueMessagesWaiting( xUARTRXQueue));
                        break;
                    }
                    // SYS_CONSOLE_MESSAGE("LORA: LPKTOK\r\n");
                    parseRXPacket(&uartrx[4], &rxpkt);
                    if(rxpkt.pkt_status == 0x10 ||
                       rxpkt.pkt_status == 0x01) // Place the packet in a queue such that app_mqtt can send it
                    {
                        //   SYS_CONSOLE_MESSAGE("LORA: PKTOUT\r\n");
                        enqueueLoRaRX(&rxpkt);
                        SYS_DEBUG(SYS_ERROR_INFO, "LORA: Accepted packet\r\n");
                        // printRXPacket(&rxpkt);
                        last_rx_timestamp = rxpkt.timestamp;
                    }
                    else
                    {
                        SYS_DEBUG(SYS_ERROR_INFO, "LORA: Packet dropped! Bad CRC\r\n");
                    }
                }
                else
                {
                    SYS_CONSOLE_MESSAGE("LORA: PKT CHKFAIL\r\n");
                }
                gotuartrx_packet = false;
                memset(uartrx, 0, uartrx_len + 6);
                uartrx_len = 0;
                sendRXReply(LORA_ACK);
                // SYS_CONSOLE_MESSAGE("LORA: EMPTY PKT MEM\r\n");
                // SYS_CONSOLE_PRINT("LORA: INBUF:%d\r\n",uxQueueMessagesWaiting( xUARTRXQueue));
            }
            break;
        }

        case APP_LORA_SEND:
        {
            // txStatus();
            if(!SYS_TMR_DelayStatusGet(timeoutTimerHandle))
                return;
            txpkt.timestamp           = 0;
            txpkt.tx_mode             = 0;
            txpkt.tx_power            = 7;
            txpkt.frequency           = 868500000ul; // 868300000;//
            txpkt.rf_chain            = 0;           // rxpkt.rf_chain;
            txpkt.modulation          = 0x10;
            txpkt.bandwidth           = 3; // 3;
            txpkt.coderate            = 1; // 1;
            txpkt.datarate            = 2; // 16;
            txpkt.invert_polarity     = 0;
            txpkt.frequency_deviation = 0;
            txpkt.preamble            = 8;
            txpkt.no_crc              = 0;
            txpkt.no_header           = 0;
            txpkt.payload_size        = 8;
            txpkt.payload[0]          = 0x11;
            txpkt.payload[1]          = 0x22;
            txpkt.payload[2]          = 0x33;
            txpkt.payload[3]          = 0x44;
            txpkt.payload[4]          = 0xAA;
            txpkt.payload[5]          = 0xBB;
            txpkt.payload[6]          = 0xCC;
            txpkt.payload[7]          = 0xDD;
            ndr++;
            if(ndr > 5)
                ndr = 0;
            SYS_PRINT("ndr: %d\r\n", ndr);
            // txpkt.payload_size = rxpkt.payload_size;
            /* uint8_t i = 0;
            for (i = 0; i <= txpkt.payload_size; i++)
            {
                txpkt.payload[i] = rxpkt.payload[i];
            } */
            sendPacket(&txpkt);
            printTXPacket(&txpkt);

            txpkt = empty_txpkt;

            _setState(APP_LORA_WAIT_SEND_COMPLETE);

            break;
        }
        case APP_LORA_WAIT_SEND_COMPLETE:
        {
            if(getStatus(LORA_COMMAND_TXSTATUS) == LORA_TX_STATUS_READY)
            {
                // SYS_DEBUG(SYS_ERROR_DEBUG, "LORA: TX STATUS %d\r\n", getStatus(LORA_COMMAND_TXSTATUS));
                _setState(APP_LORA_RECEIVE);
            }

            break;
        }

        case APP_LORA_IDLE:
        {
            SYS_TMR_CallbackStop(timeoutTimerHandle); // cancel pending timeout
            timeoutTimerHandle = SYS_TMR_DelayMS(10000);
            _setState(APP_LORA_SEND);
            Nop();
            break;
        }
    }
}

static void _setState(APP_STATES_LORA newState)
{
    SYS_DEBUG(SYS_ERROR_WARNING, "LORA: Changing state from %d to %d\r\n", appData.state, newState);
    appData.state = newState;
}

void lora_read(void)
{
    SYS_CONSOLE_MESSAGE("LORA: Starting RX TASK (kickstart LoRa module by sending an ACK)\r\n");
    sendRXReply(LORA_ACK); // Kickstart the LoRa module

    while(1)
    {
        uint8_t msg[1];
        uint8_t nread = DRV_USART_Read(appData.USARTHandle, msg, 1);
        if(nread > 0)
        {
            xQueueSend(xUARTRXQueue, msg, 0U);
        }
    }
}

bool APP_LORA_HAS_CORRECT_FREQ_PLAN(void)
{
    return (freqplan_correct == 1);
}

uint8_t APP_LORA_GW_CARD_VERSION(void)
{
    return frequency_band;
}

// Set baud to preferred according to used clock
static uint32_t getPreferredBaud(void)
{
    uint32_t baud = UART_DEFAULT_BAUD;
    if (PLIB_OSC_SysPLLInputClockSourceGet(OSC_ID_0) == OSC_SYSPLL_IN_CLK_SOURCE_FRC) {
        baud = UART_SAFE_BAUD;        
    }
    return baud;
}

// Blocking delay loop. This delay is used for a synchronization issue workaround.
static void shortSpinloopDelay(void)
{
    uint32_t t=0x0000FFFF; // Busy while loop 64k times
    while(t>0) {
        t--;
        PLIB_WDT_TimerClear(WDT_ID_0);
    }
}

