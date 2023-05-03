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
#include "gateway-module-interface.h"

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

#define UART_ERROR_NO_DATA 0
#define UART_ERROR_INCOMPLETE_DATA 1
#define UART_ERROR_REJECTED_DATA 2
#define UART_ERROR_BROKEN_DATA 3
#define UART_NO_ERROR 4

typedef struct {
        uint8_t band;
        uint8_t hwrev;
        uint8_t serial_number[12];
        uint8_t minor;
        uint8_t major;
} version_t;


static bool                   allowed_to_start = false; // TODO: Can be part of APP_DATA_LORA when it made local
static APP_DATA_LORA          appData          = {0};
extern APP_GW_ACTIVATION_DATA appGWActivationData;

static SYS_TMR_HANDLE handle;
static TaskHandle_t   loraRxThreadHandle = NULL;

static uint32_t last_rx_timestamp          = 0;
static uint32_t lora_config_failed_counter = 0;
static bool     freqplan_correct           = 0;
static version_t module_version_info       = {0};

QueueHandle_t xRXQueue, xTXQueue;

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

static bool initLora();
static bool configLora();
static bool setLeds(bool led0, bool led1, bool led2);
static bool startLora();
static bool stopLora();
static bool getVersion(version_t *version);
static void sendRXReply(bool ack);
static bool configureRXChain(uint8_t RFChain, bool enable, uint32_t center_freq);
static bool configureIFChainX(uint8_t IFChannel, bool enable, uint8_t RFChain, int32_t offset_freq);
static bool configureIFChain8(bool enable, uint8_t RFChain, int32_t offset_freq, uint32_t bandwidth, uint8_t spread_factor);
static bool configureIFChain9(bool enable, uint8_t RFChain, int32_t offset_freq, uint32_t bandwidth, uint32_t datarate);
static bool sendPacket(loraTXPacket* pkt);
static bool parseRXPacket(uint8_t* buffer, loraRXPacket* pkt, size_t size);
static bool constructTXPacket(loraTXPacket* pkt, uint8_t* tx);
static void printRXPacket(loraRXPacket* rxpkt);
static void printTXPacket(loraTXPacket* txpkt);
static bool getSync();
static bool setSync(bool public);
static void _setState(APP_STATES_LORA newState);
static void restart_lora_configuration();

/*
 * Platform specific functions for gateway module interface library
 */
SemaphoreHandle_t g_gateway_module_interface_lock = NULL;
SemaphoreHandle_t g_gateway_module_interface_log_lock = NULL;
SemaphoreHandle_t g_gateway_module_interface_signal = NULL;
static void lock_uart(bool lock);
static bool write_uart(uint8_t *data, size_t size);
static bool signal_wait(int timeout);
static void signal_set(void);
static void receive_callback(uint8_t *data, size_t size);
static void dispatch_thread(void);
static void GATEWAY_MODULE_INTERFACE_LOG (const char *__restrict __format, ...);

static uint32_t timeout_timer = 0;
static uint32_t lastAckTime   = 0;

#define TIMEOUTSTART (timeout_timer = SYS_TMR_TickCountGet())
#define TIMEOUT(X)                             \
    (timeout_timer > SYS_TMR_TickCountGet() || \
     ((SYS_TMR_TickCountGet() - timeout_timer) >= (SYS_TMR_TickCounterFrequencyGet() * X)))

#include "Harmony/MQTTHarmony.h"

static bool initLora(void)
{
    bool init_complete = false;

    // RF ENABLE LOW
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_5);

    // RESET HIGH
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_6);

    /* Open USART Driver instance 1 (USART1 in this case) and obtain a handle to it. */
    appData.USARTHandle = DRV_USART_Open(DRV_USART_INDEX_1, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_BLOCKING);

    uint16_t    usTaskStackSize = configMINIMAL_STACK_SIZE;
    UBaseType_t uxTaskPriority  = TASK_PRIORITY_LORA_READ;

    xTaskCreate((TaskFunction_t)dispatch_thread, "LORARX", usTaskStackSize, NULL, uxTaskPriority,
                &loraRxThreadHandle); /* The task handle is not used. */

    // RESET LOW
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_6);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // RESET HIGH
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_6);

    if(appData.USARTHandle != DRV_HANDLE_INVALID)
    {
        init_complete = true;
    }

    return init_complete;
}

static bool configLora(void)
{
    bool    status;

    status = stopLora();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if (!status || !getVersion(&module_version_info))
    {
        return false;
    }
    SYS_PRINT("Version, hwrev: %d, major: %d, minor: %d, band: %d\r\n", module_version_info.hwrev, module_version_info.major, module_version_info.minor, module_version_info.band);
    SYS_PRINT("Serial: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\r\n",
            module_version_info.serial_number[0], module_version_info.serial_number[1], module_version_info.serial_number[2], module_version_info.serial_number[3], module_version_info.serial_number[4], module_version_info.serial_number[5],
            module_version_info.serial_number[6], module_version_info.serial_number[7], module_version_info.serial_number[8], module_version_info.serial_number[9], module_version_info.serial_number[10], module_version_info.serial_number[11]
            );
    uint32_t config_freq_band = appGWActivationData.configuration_sx1301.rfchain[0].freq;

    // Do a check to find out if the configuration matches with the hardware
    if(config_freq_band == 0 || (config_freq_band >= 900000000 && module_version_info.band == LORA_BAND_868) ||
       (config_freq_band < 900000000 && module_version_info.band == LORA_BAND_915))
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
        if(module_version_info.band == LORA_BAND_868)
        {
            SYS_DEBUG(SYS_ERROR_WARNING, "LORA: use default EU config\r\n");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureRXChain(0, 1, 867500000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureRXChain(1, 1, 868500000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(0, 1, 1, -400000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(1, 1, 1, -200000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(2, 1, 1, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(3, 1, 0, -400000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(4, 1, 0, -200000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(5, 1, 0, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(6, 1, 0, 200000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(7, 1, 0, 400000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChain8(1, 1, -200000, 250000, 7);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChain9(1, 1, 300000, 125000, 50000);
        }
        else if(module_version_info.band == LORA_BAND_915)
        {
            SYS_DEBUG(SYS_ERROR_WARNING, "LORA: use default US config\r\n");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureRXChain(0, 1, 904200000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureRXChain(1, 1, 905000000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(0, 1, 0, -300000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(1, 1, 0, -100000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(2, 1, 0, 100000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(3, 1, 0, 300000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(4, 1, 1, -300000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(5, 1, 1, -100000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(6, 1, 1, 100000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(7, 1, 1, 300000);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChain8(1, 0, 400000, 500000, 8);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChain9(0, 0, 0, 0, 0);
        }
    }

    else
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        status = status && configureRXChain(0, appGWActivationData.configuration_sx1301.rfchain[0].enable,
                                   appGWActivationData.configuration_sx1301.rfchain[0].freq);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        status = status && configureRXChain(1, appGWActivationData.configuration_sx1301.rfchain[1].enable,
                                   appGWActivationData.configuration_sx1301.rfchain[1].freq);

        int i = 0;
        for(i = 0; i <= 7; i++)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            status = status && configureIFChainX(i, appGWActivationData.configuration_sx1301.ifchain[i].enable,
                                        appGWActivationData.configuration_sx1301.ifchain[i].radio,
                                        appGWActivationData.configuration_sx1301.ifchain[i].freqOffset);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
        status = status && configureIFChain8(appGWActivationData.configuration_sx1301.ifchain[8].enable,
                                    appGWActivationData.configuration_sx1301.ifchain[8].radio,
                                    appGWActivationData.configuration_sx1301.ifchain[8].freqOffset,
                                    appGWActivationData.configuration_sx1301.ifchain[8].bandwidth,
                                    appGWActivationData.configuration_sx1301.ifchain[8].spread_factor);

        vTaskDelay(100 / portTICK_PERIOD_MS);
        status = status && configureIFChain9(appGWActivationData.configuration_sx1301.ifchain[9].enable,
                                    appGWActivationData.configuration_sx1301.ifchain[9].radio,
                                    appGWActivationData.configuration_sx1301.ifchain[9].freqOffset,
                                    appGWActivationData.configuration_sx1301.ifchain[9].bandwidth,
                                    appGWActivationData.configuration_sx1301.ifchain[9].datarate);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    status = status && setSync(appGWActivationData.configuration_sx1301.lorawan_public);

    // RF ENABLE HIGH
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_5);

    /* Reset command for module */
    vTaskDelay(100 / portTICK_PERIOD_MS);
    status = status && stopLora();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    startLora(); // note that the ACK on start takes about 6 seconds
    SYS_DEBUG(SYS_ERROR_INFO, "LORA: configLora %s\r\n", status ? "OK" : "ERROR");
    return status;
}

static bool setLeds(bool led0, bool led1, bool led2)
{
    uint8_t payload = ((led2 << 2) | (led1 << 1) | (led0));
    return GatewayModuleInterface_sendCommandWaitAck(GATEWAY_MODULE_CMD_SETLEDS, &payload, 1);
}

static bool getSync()
{
    uint8_t sync;
    return GatewayModuleInterface_sendCommandWaitAnswer(GATEWAY_MODULE_CMD_GETSYNC, NULL, 0, &sync, 1);
}

static bool setSync(bool public)
{
    uint8_t payload = public ? 0x34 : 0x12;
    return GatewayModuleInterface_sendCommandWaitAck(GATEWAY_MODULE_CMD_SETSYNC, &payload, 1);
}

static bool startLora()
{
    return GatewayModuleInterface_sendCommandWaitAck(GATEWAY_MODULE_CMD_START, NULL, 0);
}

static bool stopLora()
{
    return GatewayModuleInterface_sendCommandWaitAck(GATEWAY_MODULE_CMD_STOP, NULL, 0);
}

static bool getVersion(version_t *version)
{
    return GatewayModuleInterface_sendCommandWaitAnswer(GATEWAY_MODULE_CMD_VERSION, NULL, 0, (uint8_t*)version, sizeof(version_t));
}

static void sendRXReply(bool ack)
{
    lastAckTime     = SYS_TMR_TickCountGet();
    GatewayModuleInterface_sendAck(GATEWAY_MODULE_CMD_RECEIVE, ack);
}

static bool configureRXChain(uint8_t RFChain, bool enable, uint32_t center_freq)
{
    // SYS_CONSOLE_PRINT("\r\nRF: %d,%d,%d", RFChain, enable, center_freq);

    uint8_t payload[6] = {0};
    payload[0]         = RFChain;
    payload[1]         = (uint8_t)enable;
    payload[2]         = _GET_BYTE(center_freq, 0);
    payload[3]         = _GET_BYTE(center_freq, 1);
    payload[4]         = _GET_BYTE(center_freq, 2);
    payload[5]         = _GET_BYTE(center_freq, 3);
    return GatewayModuleInterface_sendCommandWaitAck(GATEWAY_MODULE_CMD_RFCONFIG, payload, sizeof(payload));
}

static bool configureIFChainX(uint8_t IFChannel, bool enable, uint8_t RFChain, int32_t offset_freq)
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

    return GatewayModuleInterface_sendCommandWaitAck(GATEWAY_MODULE_CMD_IFCONFIG, payload, sizeof(payload));
}

static bool configureIFChain8(bool enable, uint8_t RFChain, int32_t offset_freq, uint32_t bandwidth, uint8_t spread_factor)
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

    // An extra undocumented byte is needed at the start of the command.
    // The byte seems to be ignored, but is relevant for command alignment.
    // This was discovered by checking the output of the GATEWAY_MODULE_CMD_IF8CHAIN command.
    uint8_t extended_payload[sizeof(payload) + 1] = {0};
    memcpy(&extended_payload[1], &payload[0], sizeof(payload));

    return GatewayModuleInterface_sendCommandWaitAck(GATEWAY_MODULE_CMD_IF8CONFIG, extended_payload, sizeof(extended_payload));
}

static bool configureIFChain9(bool enable, uint8_t RFChain, int32_t offset_freq, uint32_t bandwidth, uint32_t datarate)
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
    return GatewayModuleInterface_sendCommandWaitAck(GATEWAY_MODULE_CMD_IF9CONFIG, payload, sizeof(payload));
}

/*
 * This functions is called periodically and handles the LoRa module state machine.
 *
 * This function is non-reentrant as it contains static data. It is used only from a single point only.
 */
static bool sendPacket(loraTXPacket* txpkt)
{
    bool    status = 1;
    static uint8_t data_payload[MAX_TXPACKET_LENGTH];

    status = status && constructTXPacket(txpkt, data_payload);

    if(status)
    {
        status = status && GatewayModuleInterface_sendCommandWaitAck(GATEWAY_MODULE_CMD_SEND, data_payload, txpkt->payload_size + TXPACKET_HEADER_LENGTH);
    }

    return status;
}

static bool parseRXPacket(uint8_t* rx, loraRXPacket* pkt, size_t size)
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

    if (pkt->payload_size + 39 != size)
    {
        return false;
    }

    uint8_t i         = 0;
    for(i = 0; i < pkt->payload_size; i++)
    {
        pkt->payload[i] = rx[39 + i];
    }

    return true;
}

static bool constructTXPacket(loraTXPacket* txpkt, uint8_t* tx)
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

static void printRXPacket(loraRXPacket* rxpkt)
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

static void printTXPacket(loraTXPacket* txpkt)
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
    g_gateway_module_interface_lock = xSemaphoreCreateMutex();
    g_gateway_module_interface_log_lock = xSemaphoreCreateMutex();
    g_gateway_module_interface_signal = xSemaphoreCreateBinary();
    GatewayModuleInterface_init(&lock_uart, &write_uart, &signal_wait, &signal_set, &receive_callback, &GATEWAY_MODULE_INTERFACE_LOG);
    allowed_to_start = false;
    /* Place the App state machine in its initial state. */
    _setState(APP_LORA_INIT);

    xRXQueue = xQueueCreate(5, sizeof(loraRXPacket));
    xTXQueue = xQueueCreate(5, sizeof(loraTXPacket));
}

void APP_LORA_SetStartEvent(void)
{
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

/*
 * This functions is called periodically and handles the LoRa module state machine.
 *
 * This function is non-reentrant as it contains static data and calls non-reentrant functions.
 */
void APP_LORA_Tasks(void)
{
    static loraTXPacket txpkt = {0}, empty_txpkt = {0};

    switch(appData.state)
    {
        case APP_LORA_INIT:
        {
            if(initLora())
            {
                SYS_PRINT("LORA: Initialisation complete\r\n");
                handle = SYS_TMR_DelayMS(7000);
                _setState(APP_LORA_WAIT_INIT_COMPLETE);
            }
            appData.rxwrite_pos = 0;
            appData.rxread_pos  = 0;
            break;
        }

        case APP_LORA_WAIT_INIT_COMPLETE:
        {
            // Super temporary ugly delay to wait on the other apps
            if(SYS_TMR_DelayStatusGet(handle))
            {
                SYS_PRINT("LORA: Wait init complete, waiting for application.\r\n");
                // Empty UART driver buffer
                // uint8_t buf[1] = {};
                // if(readUart(appData.USARTHandle, buf, 1))  {;}
                // SYS_DEBUG(SYS_ERROR_DEBUG, "BUFFER: %#x\r\n", buf[1]);

                _setState(APP_LORA_WAIT_FOR_APP);
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
                handle = SYS_TMR_DelayMS(7000);
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
            if(SYS_TMR_DelayStatusGet(handle))
            {
                sendRXReply(true); // kickstart by acknowledging such that it will send next in queue
                _setState(APP_LORA_OPERATIONAL);
            }

            break;
        }
        case APP_LORA_OPERATIONAL:
        {
            if(SYS_TMR_TickCountGet() - lastAckTime >=
               SYS_TMR_TickCounterFrequencyGet() * KICK_MODULE_AFTER_LAST_ACK_TIMEOUT)
            {
                lastAckTime = SYS_TMR_TickCountGet();
                SYS_PRINT("LORA: Kick LoRa module with ACK after not acked it for %ds\r\n",
                          KICK_MODULE_AFTER_LAST_ACK_TIMEOUT);
                sendRXReply(true);
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
            }
            break;
        }
    }
}

static void _setState(APP_STATES_LORA newState)
{
    SYS_DEBUG(SYS_ERROR_WARNING, "LORA: Changing state from %d to %d\r\n", appData.state, newState);
    appData.state = newState;
}

bool APP_LORA_HAS_CORRECT_FREQ_PLAN(void)
{
    return (freqplan_correct == 1);
}

uint8_t APP_LORA_GW_CARD_VERSION(void)
{
    return module_version_info.band;
}


static void lock_uart(bool lock)
{
    if (lock)
    {
        BaseType_t ret = xSemaphoreTake(g_gateway_module_interface_lock, portMAX_DELAY);
        ASSERT(ret == pdPASS, "Mutex should never fail")
    }
    else
    {
        xSemaphoreGive(g_gateway_module_interface_lock);
    }
}

static bool write_uart(uint8_t *data, size_t size)
{
    if (size > 0)
    {
        size_t w = DRV_USART_Write(appData.USARTHandle, data, size);
        ASSERT(w == size, "LORA Uart write should be blocking");
        if (w != size)
        {
            return false;
        }
    }
    return true;
}

static bool signal_wait(int timeout)
{
    return xSemaphoreTake(g_gateway_module_interface_signal, timeout / portTICK_PERIOD_MS) == pdPASS;
}

static void signal_set(void)
{
    xSemaphoreGive(g_gateway_module_interface_signal);
}

static void receive_callback(uint8_t *data, size_t size)
{
    static loraRXPacket rxpkt = {0}, empty_rxpkt = {0};
    rxpkt = empty_rxpkt;

    if (parseRXPacket(data, &rxpkt, size))
    {
        if(rxpkt.pkt_status == 0x10 || rxpkt.pkt_status == 0x01)
        {
            enqueueLoRaRX(&rxpkt); // package is copied in the queue
            sendRXReply(true);
            GATEWAY_MODULE_INTERFACE_LOG("LORA: Accepted packet\r\n");
            // printRXPacket(&rxpkt);
            last_rx_timestamp = rxpkt.timestamp;
        }
        else
        {
            GATEWAY_MODULE_INTERFACE_LOG("Rejected packet (0x%02X)\r\n", rxpkt.pkt_status);
        }
    }
    else
    {
        GATEWAY_MODULE_INTERFACE_LOG("Unable to parse\r\n");
    }

    sendRXReply(true); // request for next package
}

static void dispatch_thread(void)
{
    char c;
    while (true)
    {
        size_t r = DRV_USART_Read(appData.USARTHandle, &c, 1);
        //ASSERT(r == 1, "LORA Uart read should be blocking");
        if (r != 1)
        {
            // ignore GATEWAY_MODULE_INTERFACE_LOG("Read returned code: %i", r);
            // Due to instable clock (some versions using the internal oscillator) many
            // bit error may occur resulting in DRV_USART_Read returning -1. Missing or
            // corrupt bytes are detected by higher layer using a checksum and retries.
            ;
        }
        else
        {
            GatewayModuleInterface_dispatch(c);
        }
    }

    GATEWAY_MODULE_INTERFACE_LOG("Thread exit.");
}

static void GATEWAY_MODULE_INTERFACE_LOG(const char *__restrict __format, ...)
{
    static char str[128];
    BaseType_t ret = xSemaphoreTake(g_gateway_module_interface_log_lock, portMAX_DELAY);
    ASSERT(ret == pdPASS, "Mutex should never fail")

    str[0] = 'L';
    str[1] = 'G';
    str[2] = 'M';
    str[3] = 'D';
    str[4] = ':';

    va_list args;
    va_start (args, __format);
    int len = vsnprintf(&str[5], sizeof(str) - 8, __format, args);
    if (len > (sizeof(str) - 8))
    {
        len = sizeof(str) - 8;
    }
    str[5 + len] = '\r';
    str[5 + len + 1] = '\n';
    str[5 + len + 2] = '\0';
    SYS_MESSAGE(str);
    va_end (args);
    xSemaphoreGive(g_gateway_module_interface_log_lock);
}
