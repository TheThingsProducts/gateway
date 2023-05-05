// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app_mqtt.h"
#include "app.h"
#include "app_lora.h"
#include "Harmony/MQTTHarmony.h"
#include "subsystem_controller.h"
#include "system_definitions.h"
#include "version.h"
#include "bootloader_version.h"
#include "time.h"
#include "session.h"
#include "error_messages.h"

#define STATUS_UPLOAD_INTERVAL 30
#define LORA_UPLINK_IDLE_ONE_DAY \
    (60 * 60 * (24 - 1)) // only 23 hours as for now the device is rebooted every 24h for firmware check
#define LORA_UPLINK_IDLE_ONE_WEEK \
    (60 * 60 * 24 * 7) // for now this never will be reached as the device is rebooted every 24h for firmware check

static int8_t _pumpDNS(const char* hostname, IPV4_ADDR* ipv4Addr);

static APP_DATA_MQTT          appData = {0};
extern APP_GW_ACTIVATION_DATA appGWActivationData;

void APP_MQTT_Initialize(void)
{
    appData.state = APP_TTNGWC_INIT;
}

uint32_t localStatusCounterSinceBoot = 0; // Used for logging only
uint32_t startTick                   = 0;
uint32_t retryTick                   = 0;
uint32_t rx_pkt_cnt                  = 0;
uint32_t tx_pkt_cnt                  = 0;

uint32_t lastLoraUplinkTick        = 0;
bool     loraUplinkIdleOneDaySent  = false;
bool     loraUplinkIdleOneWeekSent = false;

IP_MULTI_ADDRESS server_address;
char             serverip[20];
uint32_t         dnstimeout         = 0;
bool             hasdnstimeout      = false;
bool             dnstimeout_wait    = false;
static uint8_t   socket_setup_retry = 0;
static Network*  network            = NULL;
static mqttConnected                = false;

void APP_MQTT_Tasks(void)
{
    int err = -1;

    switch(appData.state)
    {
        case APP_TTNGWC_INIT:
        {
            ttngwc_init(&appData.ttn, appGWActivationData.configuration.id, &handleDownlink, NULL);
            if(appData.ttn)
            {
                struct Session* session = (struct Session*)appData.ttn;
                network                 = &session->network;
                appData.state           = APP_TTNGWC_FETCH_SERVER_IP;
            }
            else
            {
                appData.state = APP_TTNGWC_ERROR;
            }
            dnstimeout = SYS_TMR_TickCountGet();
            break;
        }
        case APP_TTNGWC_FETCH_SERVER_IP:
        {
            if(hasdnstimeout)
            {
                dnstimeout_wait = true;
                hasdnstimeout   = false;
            }
            if(dnstimeout_wait && (SYS_TMR_TickCountGet() - dnstimeout < SYS_TMR_TickCounterFrequencyGet() * 1))
            {
                // tcpTimeout = SYS_TMR_TickCountGet();
                dnstimeout      = 0;
                dnstimeout_wait = false;
                hasdnstimeout   = false;
            }
            TCPIP_DNS_RESULT result = TCPIP_DNS_Resolve(
                appGWActivationData.configuration_gateway.ttn_servers[0].server_address, TCPIP_DNS_TYPE_A);
            if(result == TCPIP_DNS_RES_NAME_IS_IPADDRESS)
            {
                strcpy(serverip, appGWActivationData.configuration_gateway.ttn_servers[0].server_address);
                appData.state = APP_TTNGWC_SOCKET_SETUP;
            }
            else
                appData.state = APP_TTNGWC_FETCHING_SERVER_IP;
        }
        break;
        case APP_TTNGWC_FETCHING_SERVER_IP:
        {
            IPV4_ADDR addr;
            switch(_pumpDNS(appGWActivationData.configuration_gateway.ttn_servers[0].server_address, &addr))
            {
                case -1:
                {
                    // Some sort of error, already reported
                    SYS_DEBUG(SYS_ERROR_WARNING, "MQTT: Wrong DNS, stopping operation\r\n");
                    appData.state = APP_TTNGWC_ERROR;
                }
                break;
                case 0:
                {
                    if(SYS_TMR_TickCountGet() - dnstimeout >= SYS_TMR_TickCounterFrequencyGet() * 2)
                    {
                        dnstimeout    = SYS_TMR_TickCountGet();
                        hasdnstimeout = true;
                        appData.state = APP_TTNGWC_FETCH_SERVER_IP;
                    }
                }
                break;
                case 1:
                {
                    TCPIP_Helper_IPAddressToString(&addr, serverip, 20);
                    dnstimeout    = SYS_TMR_TickCountGet();
                    appData.state = APP_TTNGWC_SOCKET_SETUP;
                }
                break;
            }
            break;
        }
        case APP_TTNGWC_SOCKET_SETUP:
            SYS_DEBUG(SYS_ERROR_WARNING, "MQTT: GOT IP: %s\r\n", serverip);
            err = NetworkConnect(network, serverip,
                                 appGWActivationData.configuration_gateway.ttn_servers[0].serv_port_up);

            if(err == 0)
            {
                retryTick     = SYS_TMR_TickCountGet();
                appData.state = APP_TTNGWC_SOCKET_CHECK;
            }
            else
            {
                SYS_DEBUG(SYS_ERROR_WARNING, "MQTT: SOCKET ERROR: %d\r\n", err);
                appData.state = APP_TTNGWC_ERROR;
                break;
            }
            break;
        case APP_TTNGWC_SOCKET_CHECK:
            err = Network_IsReady(network);
            if(err == 0)
            {
                if(appGWActivationData.configuration_gateway.ttn_servers[0].serv_tls)
                {
                    SYS_DEBUG(SYS_ERROR_INFO, "MQTT: Connection Opened: Starting TLS Negotiation\r\n");
                    if(NetworkTLS_Start(network) != 0)
                    {
                        SYS_DEBUG(SYS_ERROR_INFO, "MQTT: TLS Create Connection Failed - Aborting\r\n");
                        NetworkDisconnect(network);
                        appData.state = APP_TTNGWC_ERROR;
                    }
                    else
                    {
                        SYS_DEBUG(SYS_ERROR_INFO, "MQTT: Wait for SSL Connect\r\n");
                        appData.state = APP_TTNGWC_SOCKET_WAIT_TLS;
                    }
                }
                else
                {
                    SYS_DEBUG(SYS_ERROR_INFO, "MQTT: Socket good\r\n");
                    appData.state = APP_TTNGWC_CONNECT;
                }
            }
            else if(SYS_TMR_TickCountGet() - retryTick >= SYS_TMR_TickCounterFrequencyGet() * 1)
            {
                retryTick = SYS_TMR_TickCountGet();
                NetworkDisconnect(network);
                appData.state = APP_TTNGWC_ERROR;
                SYS_DEBUG(SYS_ERROR_INFO, "MQTT: Opening socket timed out, restarting\r\n");

                /*if (socket_setup_retry >= 3)
                {
                    socket_setup_retry = 0;
                    ttngwc_close_socket(appData.ttn);
                    appData.state = APP_TTNGWC_ERROR;
                }
                else
                {
                    socket_setup_retry++;
                    retryTick = SYS_TMR_TickCountGet();
                    ttngwc_close_socket(appData.ttn);
                    appData.state = APP_TTNGWC_SOCKET_SETUP;
                    SYS_DEBUG(SYS_ERROR_INFO,"MQTT: Opening socket timed out, restarting\r\n");
                }*/
            }
            break;
        case APP_TTNGWC_SOCKET_WAIT_TLS:
            if(NetworkTLS_IsStarting(network) == 1)
            {
                break;
            }
            if(NetworkTLS_IsSecure(network) != 0)
            {
                SYS_DEBUG(SYS_ERROR_INFO, "MQTT: TLS Connection Negotiation Failed - Aborting\r\n");
                NetworkDisconnect(network);
                appData.state = APP_TTNGWC_ERROR;
                break;
            }
            SYS_DEBUG(SYS_ERROR_INFO, "MQTT: TLS ready: Connect MQTT\r\n");
            appData.state = APP_TTNGWC_CONNECT;
            break;
        case APP_TTNGWC_CONNECT:
            retryTick = SYS_TMR_TickCountGet();
            err       = ttngwc_connect(
                appData.ttn, NULL, 0,
                appGWActivationData.configuration.key); // Leave host and port empty as we already connected the socket
            switch(err)
            {
                case 0:
                    SYS_DEBUG(SYS_ERROR_INFO, "MQTT: Connected\r\n");
                    mqttConnected = true;
                    appData.state = APP_TTNGWC_IDLE;
                    break;
                default:
                    // Connection failed, Keep connecting???
                    SYS_DEBUG(SYS_ERROR_WARNING, "MQTT: Connection failed\r\n");
                    appData.state = APP_TTNGWC_ERROR;
                    break;
            }
            break;
        case APP_TTNGWC_IDLE:
            if(Network_IsReady(network) != 0)
            {
                appData.state = APP_TTNGWC_ERROR;
                SYS_DEBUG(SYS_ERROR_WARNING, "MQTT: Connection lost\r\n");
                break;
            }

            retryTick = SYS_TMR_TickCountGet();
            if(hasLoraRXPacketInQueue())
            {
                // reset LoRa uplink timestamp for measuring idle time
                lastLoraUplinkTick = SYS_TMR_TickCountGet();

                APP_StatSet(LED_ACTIVITY, ON);
                err = sendUplink();
                APP_StatSet(LED_ACTIVITY, OFF);
                if(err != 0)
                {
                    appData.state = APP_TTNGWC_ERROR;
                    SYS_DEBUG(SYS_ERROR_WARNING, "MQTT: Sending UPLINK failed: %d\r\n", err);
                    break;
                }
                else
                    SYS_DEBUG(SYS_ERROR_INFO, "MQTT: Sending UPLINK OK\r\n");
            }
            else if(SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet() * STATUS_UPLOAD_INTERVAL)
            {
                startTick     = SYS_TMR_TickCountGet();
                appData.state = APP_TTNGWC_SEND_STATUS;
            }
            if(!loraUplinkIdleOneDaySent && (SYS_TMR_TickCountGet() - lastLoraUplinkTick >=
                                             SYS_TMR_TickCounterFrequencyGet() * LORA_UPLINK_IDLE_ONE_DAY))
            {
                lastLoraUplinkTick = SYS_TMR_TickCountGet();
                ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_LORA_IDLE_ONE_DAY);
                loraUplinkIdleOneDaySent = true;
            }
            if(!loraUplinkIdleOneWeekSent && (SYS_TMR_TickCountGet() - lastLoraUplinkTick >=
                                              SYS_TMR_TickCounterFrequencyGet() * LORA_UPLINK_IDLE_ONE_WEEK))
            {
                lastLoraUplinkTick = SYS_TMR_TickCountGet();
                ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_LORA_IDLE_ONE_WEEK);
                loraUplinkIdleOneWeekSent = true;
            }
            break;
        case APP_TTNGWC_SEND_STATUS:
            APP_StatSet(LED_ACTIVITY, ON);
            SYS_DEBUG(SYS_ERROR_INFO, "MQTT: Sending status packet\r\n");
            err = sendStatus();
            APP_StatSet(LED_ACTIVITY, OFF);
            if(err != 0)
            {
                appData.state = APP_TTNGWC_ERROR;
                SYS_DEBUG(SYS_ERROR_WARNING, "MQTT: Sending status failed\r\n");
                break;
            }
            else
            {
                SYS_DEBUG(SYS_ERROR_INFO, "MQTT: Sending status succeeded: %d\r\n", localStatusCounterSinceBoot);
                localStatusCounterSinceBoot++;
            }
            appData.state = APP_TTNGWC_IDLE;
            break;
        case APP_TTNGWC_ERROR:
            break;
    }
}

void APP_MQTT_Reset()
{
    if(appData.ttn != NULL)
    {
        if(mqttConnected)
        {
            ttngwc_disconnect(appData.ttn);
        }
        else if(network != NULL)
        {
            NetworkDisconnect(network);
        }
        ttngwc_cleanup(appData.ttn);
    }
    appData.state = APP_TTNGWC_INIT;
}

uint8_t APP_MQTT_GET_STATE()
{
    return appData.state;
}

int sendStatus()
{
    Gateway__Status status        = GATEWAY__STATUS__INIT;
    uint32_t        lastNtpUpdate = 0;
    TCPIP_SNTP_TimeStampGet(NULL, &lastNtpUpdate);
    if(lastNtpUpdate != 0)
    {
        status.has_time = 1;
        int64_t srvtime = ((int64_t)TCPIP_SNTP_UTCSecondsGet()) * 1000000000;
        status.time     = srvtime;
    }
    else
    {
        status.has_time = 0;
        status.time     = 0;
    }
    status.has_timestamp = 1;
    status.timestamp     = SYS_TMR_TickCountGet() / (SYS_TMR_TickCounterFrequencyGet() / 1000);
    status.has_rx_in     = 1;
    status.has_rx_ok     = 1;
    status.has_tx_in     = 1;
    status.has_tx_ok     = 1;
    status.rx_in         = rx_pkt_cnt;
    status.rx_ok         = rx_pkt_cnt;
    status.tx_in         = tx_pkt_cnt;
    status.tx_ok         = tx_pkt_cnt;

    const bootloader_version_t* pBootloaderVersion   = BootloaderVersion_Get();
    const char                  ts_fmt_str[]         = "%Y-%m-%dT%H:%M:%SZ";
    char                        platform_string[512] = {0};
    char                        ts_bl_str[100];
    char                        ts_fw_str[100];
    time_t                      ts_bl = pBootloaderVersion->timestamp;
    strftime(ts_bl_str, sizeof(ts_bl_str), ts_fmt_str, gmtime(&ts_bl));
    time_t ts_fw = VERSION_TIMESTAMP;
    strftime(ts_fw_str, sizeof(ts_fw_str), ts_fmt_str, gmtime(&ts_fw));
    snprintf(platform_string, sizeof(platform_string),
             "The Things Gateway v1 - BL r%u-%08x (%s) - Firmware v%d.%d.%d-%08x (%s)", pBootloaderVersion->revision,
             pBootloaderVersion->commit, ts_bl_str, VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_COMMIT,
             ts_fw_str);
    platform_string[sizeof(platform_string) - 1] = 0;
    status.platform                              = platform_string;
    status.frequency_plan                        = appGWActivationData.configuration.frequency_plan;

    Gateway__Status__OSMetrics metrics = GATEWAY__STATUS__OSMETRICS__INIT;
    metrics.has_memory_percentage      = 1;
    metrics.memory_percentage = ((float)1 - ((float)xPortGetFreeHeapSize() / (float)configTOTAL_HEAP_SIZE)) * 100.0f;
    status.os                 = &metrics;

    // Add error messages if any
    int   message_count = 0;
    char* messages[2];
    char  message0[ERROR_MESSAGES_PAYLOAD_SIZE * 2 + 1];
    char  message1[ERROR_MESSAGES_PAYLOAD_SIZE * 2 + 1];
    messages[0] = message0;
    messages[1] = message1;
    if(ErrorMessageFatal_BuildMessage(messages[message_count]))
    {
        SYS_PRINT("MQTT: Report reboot error: %s\r\n", messages[message_count]);
        message_count++;
    }
    if(ErrorMessageWarning_BuildMessage(messages[message_count]))
    {
        SYS_PRINT("MQTT: Report config error: %s\r\n", messages[message_count]);
        message_count++;
    }
    status.n_messages = message_count;
    status.messages   = messages;

    int err = Network_IsReady(network);
    if(err == 0)
        err = ttngwc_send_status(appData.ttn, &status);
    return err;
}

int sendUplink()
{
    Router__UplinkMessage up          = ROUTER__UPLINK_MESSAGE__INIT;
    loraRXPacket          recv_packet = {0};
    dequeueLoRaRX(&recv_packet);
    up.has_payload  = 1;
    up.payload.len  = recv_packet.payload_size;
    up.payload.data = recv_packet.payload;

    // Set protocol metadata
    Protocol__RxMetadata protocol = PROTOCOL__RX_METADATA__INIT;
    protocol.protocol_case        = PROTOCOL__RX_METADATA__PROTOCOL_LORAWAN;
    Lorawan__Metadata lorawan     = LORAWAN__METADATA__INIT;
    lorawan.has_modulation        = 1;
    char datarate[9]              = {0};
    char coderate[4]              = {0};
    if(recv_packet.modulation == LORA_MOD_LORA)
    {
        lorawan.modulation = LORAWAN__MODULATION__LORA;

        uint8_t offset = 3;
        datarate[0]    = 'S';
        datarate[1]    = 'F';

        if(recv_packet.datarate == 0x02)
            datarate[2] = '7';
        else if(recv_packet.datarate == 0x04)
            datarate[2] = '8';
        else if(recv_packet.datarate == 0x08)
            datarate[2] = '9';
        else if(recv_packet.datarate == 0x10)
        {
            datarate[2] = '1';
            datarate[3] = '0';
            offset++;
        }
        else if(recv_packet.datarate == 0x20)
        {
            datarate[2] = '1';
            datarate[3] = '1';
            offset++;
        }
        else
        {
            datarate[2] = '1';
            datarate[3] = '2';
            offset++;
        }
        datarate[offset] = 'B';
        offset++;
        datarate[offset] = 'W';
        offset++;
        if(recv_packet.bandwidth == 0x01)
        {
            datarate[offset]     = '5';
            datarate[offset + 1] = '0';
            datarate[offset + 2] = '0';
        }
        else if(recv_packet.bandwidth == 0x02)
        {
            datarate[offset]     = '2';
            datarate[offset + 1] = '5';
            datarate[offset + 2] = '0';
        }
        else
        {
            datarate[offset]     = '1';
            datarate[offset + 1] = '2';
            datarate[offset + 2] = '5';
        }
        datarate[offset + 3] = '\0';
        lorawan.data_rate    = datarate;

        coderate[0] = '4';
        coderate[1] = '/';
        coderate[2] = '4' + recv_packet.coderate;
        coderate[3] = '\0';

        lorawan.coding_rate  = coderate;
    } else {
        lorawan.modulation   = LORAWAN__MODULATION__FSK;
        lorawan.has_bit_rate = 1;
        lorawan.bit_rate     = recv_packet.datarate;
    }

    protocol.lorawan     = &lorawan;
    up.protocol_metadata = &protocol;

    // Set gateway metadata
    Gateway__RxMetadata gateway = GATEWAY__RX_METADATA__INIT;
    gateway.has_timestamp       = 1;
    gateway.timestamp           = recv_packet.timestamp;
    gateway.has_rf_chain        = 1;
    gateway.rf_chain            = recv_packet.rf_chain;
    gateway.has_frequency       = 1;
    gateway.frequency           = recv_packet.frequency;

    gateway.has_rf_chain = 1;
    gateway.rf_chain     = recv_packet.rf_chain;

    uint32_t lastNtpUpdate = 0;
    TCPIP_SNTP_TimeStampGet(NULL, &lastNtpUpdate);
    if(lastNtpUpdate != 0)
    {
        int64_t srvtime  = ((int64_t)TCPIP_SNTP_UTCSecondsGet()) * 1000000000;
        gateway.has_time = 1;
        gateway.time     = srvtime;
    }
    else
    {
        gateway.has_time = 0;
        gateway.time     = 0;
    }

    gateway.has_channel = 1;
    gateway.channel     = recv_packet.if_chain;

    gateway.has_rssi = 1;
    memcpy(&gateway.rssi, &recv_packet.rssi, 4);

    if(recv_packet.modulation == LORA_MOD_LORA)
    {
        gateway.has_snr = 1;
        memcpy(&gateway.snr, &recv_packet.snr_average, 4);
    }

    up.gateway_metadata = &gateway;

    // Send uplink message
    int err = Network_IsReady(network);
    if(err == 0)
        err = ttngwc_send_uplink(appData.ttn, &up);

    if(err != 0 && recv_packet.uploadretry < 3)
    {
        // Place the failed LoRa packet back in the queue
        recv_packet.uploadretry += 1;
        enqueueLoRaRX(&recv_packet);
        return err;
    }
    rx_pkt_cnt++;
    return err;
}

void handleDownlink(Router__DownlinkMessage* message, void* arg)
{
    tx_pkt_cnt++;
    loraTXPacket pkt = {0};

    Gateway__TxConfiguration *gateway_configuration = message->gateway_configuration;
    Lorawan__TxConfiguration *lorawan               = message->protocol_configuration->lorawan;

    pkt.frequency = gateway_configuration->frequency;
    pkt.tx_mode   = gateway_configuration->has_timestamp;

    if(gateway_configuration->has_timestamp)
    {
        pkt.timestamp = gateway_configuration->timestamp;
    }

    pkt.rf_chain = 0;
    pkt.tx_power = gateway_configuration->power;

    if(lorawan->modulation == LORAWAN__MODULATION__LORA)
    {
        pkt.modulation = LORA_MOD_LORA;

        uint8_t  spreadingfactor = 7;
        uint16_t bandwidth       = 0;

        char* sfbw = lorawan->data_rate;
        if(sfbw[2] == '1')
        {
            char sf[3];
            sf[0]           = sfbw[2];
            sf[1]           = sfbw[3];
            sf[2]           = '\0';
            spreadingfactor = atoi(sf);

            char bw[4];
            bw[0]     = sfbw[6];
            bw[1]     = sfbw[7];
            bw[2]     = sfbw[8];
            bw[3]     = '\0';
            bandwidth = atoi(bw);
        }
        else
        {
            char sf[2];
            sf[0]           = sfbw[2];
            sf[1]           = '\0';
            spreadingfactor = atoi(sf);

            char bw[4];
            bw[0]     = sfbw[5];
            bw[1]     = sfbw[6];
            bw[2]     = sfbw[7];
            bw[3]     = '\0';
            bandwidth = atoi(bw);
        }
        pkt.datarate = 0x02 << (spreadingfactor - 7);

        if(bandwidth == 500)
            pkt.bandwidth = LORA_BW_500K;
        else if(bandwidth == 250)
            pkt.bandwidth = LORA_BW_250K;
        else if(bandwidth == 125)
            pkt.bandwidth = LORA_BW_125K;
        else
            pkt.bandwidth = LORA_BW_250K;

        char*   cr       = lorawan->coding_rate;
        uint8_t coderate = ((cr[2] - 4) - 0x30);
        pkt.coderate     = coderate;

        if(gateway_configuration->has_polarization_inversion)
        {
            pkt.invert_polarity = gateway_configuration->polarization_inversion;
        }

        pkt.preamble  = 8;
        pkt.no_crc    = true;
    } else {
        uint32_t bit_rate            = lorawan->bit_rate;
        uint32_t frequency_deviation = bit_rate / 2;

        if(gateway_configuration->has_frequency_deviation)
        {
            frequency_deviation = gateway_configuration->frequency_deviation;
        }

        pkt.modulation          = LORA_MOD_FSK;
        pkt.bandwidth           = LORA_BW_125K;
        pkt.datarate            = bit_rate;
        pkt.frequency_deviation = frequency_deviation / 1000;
        pkt.preamble            = 5;
        pkt.no_crc              = false;
    }

    pkt.no_header = false;

    uint16_t it = 0;
    for(it = 0; it < message->payload.len; it++)
    {
        pkt.payload[it] = message->payload.data[it];
        if(it > 255)
            break;
    }

    pkt.payload_size = message->payload.len;
    enqueueLoRaTX(&pkt);

    SYS_DEBUG(SYS_ERROR_INFO, "MQTT: Received DOWNLINK\r\n");
}

void getPacketCount(uint32_t* pup, uint32_t* pdown)
{
    *pup   = rx_pkt_cnt;
    *pdown = tx_pkt_cnt;
    return;
}

static int8_t _pumpDNS(const char* hostname, IPV4_ADDR* ipv4Addr)
{
    IP_MULTI_ADDRESS mAddr;

    TCPIP_DNS_RESULT result = TCPIP_DNS_IsResolved(hostname, &mAddr, IP_ADDRESS_TYPE_IPV4);
    switch(result)
    {
        case TCPIP_DNS_RES_OK:
        {
            // We now have an IPv4 Address
            // Open a socket
            ipv4Addr->Val = mAddr.v4Add.Val;
            return 1;
        }
        case TCPIP_DNS_RES_PENDING:
            return 0;
        case TCPIP_DNS_RES_SERVER_TMO:
            return 0;
        case TCPIP_DNS_RES_NO_IP_ENTRY:
        default:
            SYS_DEBUG(SYS_ERROR_FATAL, "MQTT: TCPIP_DNS_IsResolved returned failure code %d\r\n", result);
            // SYS_CONSOLE_PRINT("status: %i\r\n", result);
            return -1;
    }
    // Should not be here!
    return -1;
}
