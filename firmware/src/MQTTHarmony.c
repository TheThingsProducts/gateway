/*******************************************************************************
 * Copyright (c) 2014, 2015 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Allan Stockdill-Mander - initial API and implementation and/or initial documentation
 *    Ian Craggs - convert to FreeRTOS
 *    Johan Stokking - convert to Microchip Harmony
 *******************************************************************************/

/* Copy of vendor/paho.mqtt.embedded-c/MQTTClient-C/src/Harmony/MQTTHarmony.c (submodule commit
 * 0c5d52b), built instead of the submodule file. Only change: the stack size of the MQTT read
 * task in ThreadStart(), see MQTT_TASK_STACK_WORDS (gateway issue #17). */

#include "Harmony/MQTTHarmony.h"

/* Stack of the MQTT read task in words (4 bytes). The submodule uses 4096 (16 KB), allocated at
 * every MQTT connect; on a fragmented heap that allocation failed after a network
 * re-initialisation and reset the gateway. Measured use: 564 words. */
#define MQTT_TASK_STACK_WORDS 1024

void MutexInit(Mutex *mutex) {
    mutex->sem = xSemaphoreCreateMutex();
}

int MutexLock(Mutex *mutex) {
    return xSemaphoreTake(mutex->sem, portMAX_DELAY);
}

int MutexUnlock(Mutex *mutex) {
    return xSemaphoreGive(mutex->sem);
}

int MutexDestroy(Mutex *mutex) {
    vSemaphoreDelete(mutex->sem);
    return 0;
}

void TimerCountdownMS(Timer *timer, unsigned int timeout_ms) {
    timer->xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
    vTaskSetTimeOutState(&timer->xTimeOut); /* Record the time at which this function was entered. */
}

void TimerCountdown(Timer *timer, unsigned int timeout) {
    TimerCountdownMS(timer, timeout * 1000);
}

int TimerLeftMS(Timer *timer) {
    xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait); /* updates xTicksToWait to the number left */
    return (timer->xTicksToWait < 0) ? 0 : (timer->xTicksToWait * portTICK_PERIOD_MS);
}

char TimerIsExpired(Timer *timer) {
    return xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait) == pdTRUE;
}

void TimerInit(Timer *timer) {
    timer->xTicksToWait = 0;
    timer->xTimeOut.xOverflowCount = 0;
    timer->xTimeOut.xTimeOnEntering = 0;
   // memset(&timer->xTimeOut->xOverflowCount, '\0', sizeof (timer->xTimeOut));
}

int Harmony_read(Network *n, unsigned char *buffer, int len, int timeout_ms) {
    TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
    TimeOut_t xTimeOut;
    int recvLen = 0;

    vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
    do {
        if (NET_PRES_SocketWasReset(n->my_socket)) {
            return 0;
        }
        uint16_t rc = NET_PRES_SocketRead(n->my_socket, buffer + recvLen, len - recvLen); // FIXME: Should be a blocking function... now the MQTT thread it busy-while waiting here
        if (rc > 0)
            recvLen += rc;
        else if (rc == 0) {
            recvLen = 0;
        }
    } while (recvLen < len && xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE);

    if(xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait)==pdTRUE) return -3;
    return recvLen;
}

int Harmony_write(Network *n, unsigned char *buffer, int len, int timeout_ms) {
    TickType_t xTicksToWait = timeout_ms / portTICK_PERIOD_MS; /* convert milliseconds to ticks */
    TimeOut_t xTimeOut;
    int sentLen = 0;

    vTaskSetTimeOutState(&xTimeOut); /* Record the time at which this function was entered. */
    do {
        int rc = NET_PRES_SocketWrite(n->my_socket, buffer + sentLen, len - sentLen);
        if (rc > 0)
            sentLen += rc;
        else if (rc == 0) {
            sentLen = 0;
            break;
        } else if (rc < 0) {
            sentLen = rc;
            break;
        }
    } while (sentLen < len && xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE);

    return sentLen;
}

void NetworkInit(Network *n) {
    n->my_socket = INVALID_SOCKET;
    n->mqttread = &Harmony_read;
    n->mqttwrite = &Harmony_write;
}

int NetworkConnect(Network *n, char *addr, int port) {
    IP_MULTI_ADDRESS remoteAddress;
    
    if (Network_IsReady(n) == 0)
        return 0; // Network is already ready (e.g. if a socket with TLS was set up externally)
    
    if (strlen(addr) == 0)
        return -1;

    if (!TCPIP_Helper_StringToIPAddress(addr, &remoteAddress.v4Add))
        return -1;

    SYS_PRINT("Connecting to: %s\r\n",addr);
    n->my_socket = NET_PRES_SocketOpen(0,
            NET_PRES_SKT_UNENCRYPTED_STREAM_CLIENT,
            IP_ADDRESS_TYPE_IPV4,
            port, 
            (NET_PRES_ADDRESS *)&remoteAddress,
            NULL);
    if (n->my_socket == INVALID_SOCKET)
        return INVALID_SOCKET;
    
    /*
     * We increased TCPIP_TCP_CLOSE_WAIT_TIMEOUT to 2 seconds (was 200ms)
     * To be able to reuse the socket within 2 seconds we
     * don't want to have graceful close (effectively it will be an abort).
     */
    TCP_OPTION_LINGER_DATA LData;
    NET_PRES_SocketOptionsGet(n->my_socket, TCP_OPTION_LINGER, &LData);
    LData.gracefulEnable = false;
    NET_PRES_SocketOptionsSet(n->my_socket, TCP_OPTION_LINGER, &LData);
    
    return 0;
}

void NetworkDisconnect(Network *n) {
    if (n->my_socket != INVALID_SOCKET) {
        NET_PRES_SocketDisconnect(n->my_socket);
        NET_PRES_SocketClose(n->my_socket);
        n->my_socket = INVALID_SOCKET;
    }
}

int Network_IsReady(Network *n)
{
   if (!NET_PRES_SocketIsConnected(n->my_socket))
    {
        return 1;
    }
   if (NET_PRES_SocketWasReset(n->my_socket))
    {
        return 1;
    }
    if (NET_PRES_SocketWriteIsReady(n->my_socket, 1, 1) == 0)
    {
        return 1;
    }  
   return 0;
}

int NetworkTLS_Start(Network *n)
{
    // (mbed)TLS requires RX and TX buffers to be 16384;
    return (NET_PRES_SocketOptionsSet(n->my_socket, TCP_OPTION_RX_BUFF, (void*)20000) &&
    NET_PRES_SocketOptionsSet(n->my_socket, TCP_OPTION_TX_BUFF, (void*)16384) &&
    NET_PRES_SocketEncryptSocket(n->my_socket)) ? 0 : -1;
}

int NetworkTLS_IsStarting(Network *n)
{
    return NET_PRES_SocketIsNegotiatingEncryption(n->my_socket) ? 1 : 0;
}

int NetworkTLS_IsSecure(Network *n)
{
   return NET_PRES_SocketIsSecure(n->my_socket) ? 0 : -1;
}


int ThreadStart(Thread *thread, void (*fn)(void *), void *arg) {
    uint16_t usTaskStackSize = MQTT_TASK_STACK_WORDS;
    UBaseType_t uxTaskPriority = 1; // Give lowest prio (busy whyle loop) FIXME: centralize prio config uxTaskPriorityGet(NULL); /* set the priority as the same as the calling task*/

    return xTaskCreate(fn, /* The function that implements the task. */
            "MQTTTask", /* Just a text name for the task to aid debugging. */
            usTaskStackSize, /* The stack size is defined in FreeRTOSIPConfig.h. */
            arg, /* The task parameter, not used in this case. */
            uxTaskPriority, /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
            &thread->task); /* The task handle is used for killing. */
}

int ThreadJoin(Thread *thread) {
    if (thread->task != NULL) {
        vTaskDelete(thread->task);
        thread->task = NULL;
    }
    return SUCCESS;
}

void ThreadExit() {
}

void QueueInit(Queue *q) {
    q->queue = xQueueCreate(1, sizeof (unsigned short));
}

int Enqueue(Queue *q, unsigned short item) {
    return xQueueSend(q->queue, &item, 0U) == pdTRUE ? SUCCESS : FAILURE;
}

int Dequeue(Queue *q, unsigned short *item, Timer *timer) {
    xTaskCheckForTimeOut(&timer->xTimeOut, &timer->xTicksToWait);
    return xQueueReceive(q->queue, item, timer->xTicksToWait) == pdTRUE ? SUCCESS : TIMEOUT;
}

int QueueDestroy(Queue *q) {
    if (q->queue != NULL) {
        vQueueDelete(q->queue);
        q->queue = NULL;
    }
    return SUCCESS;
}
