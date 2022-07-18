/**
 * @file      lr1110_hal.c
 *
 * @brief     HAL implementation for LR1110 radio chip
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lr1110_hal.h"
#include "lr1110_system.h"
#include "bcm2835.h"
#include <stdio.h>


//Raspberry pi bcm2835 configuration

// SNIFF LED on RPi pin GPIO 18
#define SNIFF 18

// BUSY on RPi pin GPIO 23
#define BUSY 23

// NSS on RPi pin GPIO 24
#define NSS 24

// NRESET on RPi pin GPIO 26  
#define RESET 26

// Dummy Byte
#define LR1110_NOP ( 0x00 )

/*
typedef struct
{
    int      nss;
    int      reset;
    int      irq;
    int      busy;
} radio_t;
*/

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_mode_t;

static volatile radio_mode_t radio_mode = RADIO_AWAKE;

void lr1110_hal_reset( const void* radio )
{
    //radio_t* radio_local = ( radio_t* ) radio;
    bcm2835_gpio_write(RESET, LOW);
    delayMicroseconds(500); 
    bcm2835_gpio_write(RESET, HIGH);
    radio_mode = RADIO_AWAKE;

    return LR1110_HAL_STATUS_OK;
}

lr1110_hal_status_t lr1110_hal_wakeup( const void* radio )
{
    lr1110_hal_check_device_ready(radio);
    return LR1110_HAL_STATUS_OK;
}

void lr1110_hal_check_device_ready( const void* radio )
{
    if( radio_mode != RADIO_SLEEP )
    {
        lr1110_hal_wait_on_busy();
    }
    else
    {
        // Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
        bcm2835_gpio_write(NSS, LOW);
        bcm2835_gpio_write(NSS, HIGH);
        lr1110_hal_wait_on_busy();
        radio_mode = RADIO_AWAKE;
    }
}

void lr1110_hal_wait_on_busy()
{
    while(bcm2835_gpio_lev(BUSY) == HIGH){};

}

lr1110_hal_status_t lr1110_hal_read( const void* radio, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                     uint8_t* rbuffer, const uint16_t rbuffer_length )
{
    lr1110_hal_check_device_ready( radio );

    /* 1st SPI transaction */

    bcm2835_gpio_write(NSS, LOW);
    bcm2835_spi_transfern(cbuffer, cbuffer_length);
    bcm2835_gpio_write(NSS, HIGH);

    lr1110_hal_check_device_ready( radio );

    /* 2nd SPI transaction */
    bcm2835_gpio_write(NSS, LOW);
    bcm2835_spi_transfer(LR1110_NOP);
    for (uint16_t i = 0; i < rbuffer_length; i++)
    {
        rbuffer[i] = bcm2835_spi_transfer(LR1110_NOP);
    }
    bcm2835_gpio_write(NSS, HIGH);

    return LR1110_HAL_STATUS_OK;
}


lr1110_hal_status_t lr1110_hal_write( const void* radio, const uint8_t* cbuffer, const uint16_t cbuffer_length,
                                      const uint8_t* cdata, const uint16_t cdata_length )
{
    lr1110_hal_check_device_ready( radio );

    bcm2835_gpio_write(NSS, LOW);
    bcm2835_spi_transfern(cbuffer, cbuffer_length);
    bcm2835_spi_transfern(cdata, cdata_length);
    bcm2835_gpio_write(NSS, HIGH);

    return LR1110_HAL_STATUS_OK;
}

lr1110_hal_status_t lr1110_hal_direct_read( const void* radio, uint8_t* buffer, const uint16_t length )
{
    lr1110_hal_check_device_ready( radio );

    bcm2835_gpio_write(NSS, LOW);
    bcm2835_spi_transfer(LR1110_NOP);
    for (uint16_t i = 0; i < buffer; i++)
    {
        buffer[i] = bcm2835_spi_transfer(LR1110_NOP);
    }
    bcm2835_gpio_write(NSS, HIGH);

    return LR1110_HAL_STATUS_OK;
    
}


/*
lr1110_hal_status_t lr1110_hal_write_read(const void* context, const uint8_t* command, uint8_t* data,
                                           const uint16_t data_length)
{


}
*/