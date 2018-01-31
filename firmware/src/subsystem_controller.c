// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "subsystem_controller.h"
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

static bool wifiOn     = false;
static bool bleOn      = false;
static bool ethernetOn = false;
static bool loraOn     = false;
static bool sdOn       = false;
static bool gpsOn      = false;

static bool stat[5];

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

void sdSet(bool state)
{
    if(state)
    {
        TRISB = TRISB & (~0x0100); // SD card on
        PORTB = PORTB & (~0x0100);
        sdOn  = true;
    }
    else
    {
        TRISB = TRISB | 0x0100; // SD card off
        PORTB = PORTB | 0x0100;
        sdOn  = false;
    }
}

bool sdIsOn()
{
    return sdOn;
}

void wifiSet(bool state)
{
    if(state)
    {
        TRISB  = TRISB & (~0x0200); // Wifi on
        PORTB  = PORTB & (~0x0200);
        wifiOn = true;
    }
    else
    {
        TRISB  = TRISB | 0x0200; // Wifi off
        PORTB  = PORTB | 0x0200;
        wifiOn = false;
    }
}

bool wifiIsOn()
{
    return wifiOn;
}

void ethernetSet(bool state)
{
    if(state)
    {
        TRISB = TRISB & (~0x0400); // Ethernet on
        PORTB = PORTB & (~0x0400);

        ethernetOn = true;
    }
    else
    {
        TRISB      = TRISB | 0x0400; // Ethernet off
        PORTB      = PORTB | 0x0400;
        ethernetOn = false;
    }
}

bool ethernetIsOn()
{
    return ethernetOn;
}

void loraSet(bool state)
{
    if(state)
    {
        TRISE  = TRISE & (~0x02); // LoRa on
        PORTE  = PORTE & (~0x02);
        loraOn = true;
    }
    else
    {
        TRISE  = TRISE | 0x02; // LoRa off
        PORTE  = PORTE | 0x02;
        loraOn = false;
    }
}

bool loraIsOn()
{
    return loraOn;
}

void bleSet(bool state)
{
    if(state)
    {
        TRISH = TRISH & (~0x0800); // BLE on
        PORTH = PORTH & (~0x0800);
        bleOn = true;
    }
    else
    {
        TRISH = TRISH | 0x0800; // BLE off
        PORTH = PORTH | 0x0800;
        bleOn = false;
    }
}

bool bleIsOn()
{
    return bleOn;
}

void gpsSet(bool state)
{
    if(state)
    {
        TRISC = TRISC & (~0x08); // GPS on
        PORTC = PORTC & (~0x08);
        gpsOn = true;
    }
    else
    {
        TRISC = TRISC | 0x08; // GPS off
        PORTC = PORTC | 0x08;
        gpsOn = false;
    }
}

bool gpsIsOn()
{
    return gpsOn;
}

void statOn()
{
    statSet(0, ON);
    statSet(1, ON);
    statSet(2, ON);
    statSet(3, ON);
    statSet(4, ON);
}

void statOff()
{
    statSet(0, OFF);
    statSet(1, OFF);
    statSet(2, OFF);
    statSet(3, OFF);
    statSet(4, OFF);
}

void statSet(uint8_t stat_num, bool state)
{
    switch(stat_num)
    {
        case 0:
            if(state)
            {
                TRISF = TRISF & (~0x01); // Status LED 1 on
                PORTF = PORTF | 0x01;
            }
            else
            {
                TRISF = TRISF | 0x01; // Status LED 1 off
                PORTF = PORTF & (~0x01);
            }
            break;
        case 1:
            if(state)
            {
                TRISF = TRISF & (~0x02); // Status LED 2 on
                PORTF = PORTF | 0x02;
            }
            else
            {
                TRISF = TRISF | 0x02; // Status LED 2 off
                PORTF = PORTF & (~0x02);
            }
            break;
        case 2:
            if(state)
            {
                TRISK = TRISK & (~0x80); // Status LED 3 on
                PORTK = PORTK | 0x80;
            }
            else
            {
                TRISK = TRISK | 0x80; // Status LED 3 off
                PORTK = PORTK & (~0x80);
            }
            break;
        case 3:
            if(state)
            {
                TRISA = TRISA & (~0x40); // Status LED 4 on
                PORTA = PORTA | 0x40;
            }
            else
            {
                TRISA = TRISA | 0x40; // Status LED 4 off
                PORTA = PORTA & (~0x40);
            }
            break;
        case 4:
            if(state)
            {
                TRISA = TRISA & (~0x80); // Status LED 5 on
                PORTA = PORTA | 0x80;
            }
            else
            {
                TRISA = TRISA | 0x80; // Status LED 5 off
                PORTA = PORTA & (~0x80);
            }
            break;
        default:
            break;
    }
    stat[stat_num] = state;
}

void statToggle(uint8_t stat_num)
{
    if(stat_num > 4)
    {
        return;
    }
    statSet(stat_num, stat[stat_num] ^= ON);
}

bool statIsOn(uint8_t stat_num)
{
    if(stat_num > 4)
    {
        return false;
    }
    return stat[stat_num];
}

/* *****************************************************************************
 End of File
 */
