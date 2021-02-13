/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
//#include <math.h>
#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_rtc.h"
//#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "lis2dh.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common address for LIS2DH */
#define LIS2DH_ADDR      0x18U

#define LIS2DH_CTRL_REG   0xA0U
#define LIS2DH_DATA_REG   0xA8U
#define WHO_AM_I          0x0FU
#define LIS2DH_CTRL_REG1  0x20U
#define LIS2DH_CTRL_REG4  0x23U
#define OUT_X_L           0x28U
#define OUT_X_H           0x29U

/* range */
#define LIS2DH_RANGE_2GA    0x00U
#define LIS2DH_RANGE_4GA    0x10U

#define MG_SCALE_VEL2     16

/* mode */
#define LOW_POWER_MODE    0x2FU // Low Power and ODR = 10Hz
#define LOW_POWER_MODE1   0x7FU // Low Power and ODR = 400Hz
#define LOW_POWER_MODE2   0x8FU // Low Power and ODR = 1620Hz

/* PPI */
#define PPI_TIMER1_INTERVAL   (1) // Timer interval in milliseconds, this is twi sampling rate. 
#define RTC_FREQUENCY     3200
#define RTC_CC_VALUE      8

/* buffer size */
#define TWIM_RX_BUF_WIDTH    6
#define TWIM_RX_BUF_LENGTH  100

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

/* Buffer for samples read from accelerometer */
typedef struct ArrayList
{
    uint8_t buffer[TWIM_RX_BUF_WIDTH];
} array_list_t;

static array_list_t p_rx_buffer[TWIM_RX_BUF_LENGTH];
static int16_t x;
static int16_t y;
static int16_t z;

static uint8_t m_sensorData[6];
static uint8_t m_dataReg[1] = {LIS2DH_DATA_REG};

/* ppi setting */
/*
static const nrf_drv_rtc_t   m_rtc1 = NRF_DRV_RTC_INSTANCE(1);
static const nrf_drv_rtc_t   m_rtc2 = NRF_DRV_RTC_INSTANCE(2);
*/
static const nrf_drv_timer_t m_timer1 = NRF_DRV_TIMER_INSTANCE(1);
static const nrf_drv_timer_t m_timer2 = NRF_DRV_TIMER_INSTANCE(2);

static nrf_ppi_channel_t     m_ppi_channel1;
static nrf_ppi_channel_t     m_ppi_channel2;

/* etc.. */
static uint32_t              m_evt_counter;

/* Function for setting LIS2DH accelerometer */
void LIS2DH_set_mode(void)
{
    ret_code_t err_code;
    
    /* Writing to LIS2DH_CTR_REG set range and Low Power mode */
    //uint8_t reg[7] = {LIS2DH_CTRL_REG, LOW_POWER_MODE1, 0x00, 0x00, LIS2DH_RANGE_2GA, 0x00, 0x00};
    uint8_t reg[2] = {LIS2DH_CTRL_REG, LOW_POWER_MODE1};
    //uint8_t reg[1] = {LIS2DH_CTRL_REG1};
    err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    m_xfer_done = false;

}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t* data)
{   
    x = (((int8_t)data[1]) << 8) + data[0];
    y = (((int8_t)data[3]) << 8) + data[2];
    z = (((int8_t)data[5]) << 8) + data[4];
    //NRF_LOG_INFO("x: %d", x);
    //NRF_LOG_INFO("y: %d", y);
    //NRF_LOG_INFO("z: %d", z);
    //norm = (int)sqrt((double)x*x + y*y + z*z);
    //xx = (int32_t)x*1000/(1024*MG_SCALE_VEL2);
    //yy = (int32_t)y*1000/(1024*MG_SCALE_VEL2);
    //zz = (int32_t)z*1000/(1024*MG_SCALE_VEL2);

    NRF_LOG_INFO("%d, %d, %d", x, y, z);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sensorData);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lis2dh_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lis2dh_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/* Function for reading data from accelerometer*/
static void read_accel_data()
{
    ret_code_t err_code;

    /* Writing to LIS2DH_DATA_REG */
    uint8_t reg[1] = {LIS2DH_DATA_REG};
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /* read x, y, z-axis data */
     m_xfer_done = false;
     err_code = nrf_drv_twi_rx(&m_twi, LIS2DH_ADDR, m_sensorData, sizeof(m_sensorData));
     APP_ERROR_CHECK(err_code);

}

/* for debug, reading who_am_i etc*/
static void read_who()
{
    ret_code_t err_code;
    
    uint8_t reg[1] = {LIS2DH_CTRL_REG1};
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, LIS2DH_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}

/* empty function */
static void timer1_handler(nrf_timer_event_t event_type, void * p_context)
{

}


/* TWIM counter handler */
static void timer2_handler(nrf_timer_event_t event_type, void * p_context)
{
    m_xfer_done = true;

    nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(LIS2DH_ADDR, m_dataReg, 
                                      sizeof(m_dataReg), (uint8_t*)p_rx_buffer, sizeof(p_rx_buffer) / TWIM_RX_BUF_LENGTH);

    uint32_t flags = NRF_DRV_TWI_FLAG_HOLD_XFER             |
                     NRF_DRV_TWI_FLAG_RX_POSTINC            |
                     NRF_DRV_TWI_FLAG_NO_XFER_EVT_HANDLER   |
                     NRF_DRV_TWI_FLAG_REPEATED_XFER;

    ret_code_t err_code = nrf_drv_twi_xfer(&m_twi, &xfer, flags);
    APP_ERROR_CHECK(err_code);

}

/** @brief Function starting the internal LFCLK XTAL oscillator.
 
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}
*/

// Function for Timer 1 initialization
static void timer1_init(void)
{
    nrf_drv_timer_config_t timer1_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer1_config.frequency = NRF_TIMER_FREQ_31250Hz;
    ret_code_t err_code = nrf_drv_timer_init(&m_timer1, &timer1_config, timer1_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_extended_compare(&m_timer1,
                                   NRF_TIMER_CC_CHANNEL0,
                                   nrf_drv_timer_ms_to_ticks(&m_timer1,
                                                             PPI_TIMER1_INTERVAL),
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);

}

// Function for Timer 2 initialization
static void timer2_init(void)
{
    nrf_drv_timer_config_t timer2_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer2_config.mode = NRF_TIMER_MODE_LOW_POWER_COUNTER;
    ret_code_t err_code = nrf_drv_timer_init(&m_timer2, &timer2_config, timer2_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_extended_compare(&m_timer2,
                                    NRF_TIMER_CC_CHANNEL0,
                                    TWIM_RX_BUF_LENGTH,
                                    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                    true);
}

/* initialize ppi channel */
static void twi_accel_ppi_init(void)
{
    ret_code_t err_code;
    
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
    /*
    // setup rtc1 
    nrf_drv_rtc_config_t rtc1_config = NRF_DRV_RTC_DEFAULT_CONFIG;
    rtc1_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&m_rtc1, &rtc1_config, rtc1_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&m_rtc1, 0, RTC_CC_VALUE, false);
    APP_ERROR_CHECK(err_code);

    //nrf_drv_rtc_enable(&m_rtc1);

    uint32_t rtc1_compare_event_addr = nrf_drv_rtc_event_address_get(&m_rtc1, NRF_RTC_EVENT_COMPARE_0);

    // setup rtc2 
    nrf_drv_rtc_config_t rtc2_config = NRF_DRV_RTC_DEFAULT_CONFIG;
    rtc2_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    err_code = nrf_drv_rtc_init(&m_rtc2, &rtc2_config, rtc2_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&m_rtc2, 0, RTC_CC_VALUE, false);
    APP_ERROR_CHECK(err_code);

    //nrf_drv_rtc_enable(&m_rtc2);

    uint32_t rtc2_compare_event_addr = nrf_drv_rtc_event_address_get(&m_rtc2, NRF_RTC_EVENT_COMPARE_0);
    */

    // setup timer1
    timer1_init();

    //setup timer2
    timer2_init();

    nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(LIS2DH_ADDR, m_dataReg, 
                                      sizeof(m_dataReg), (uint8_t*)p_rx_buffer, sizeof(p_rx_buffer)  / TWIM_RX_BUF_LENGTH);

    uint32_t flags = NRF_DRV_TWI_FLAG_HOLD_XFER             |
                     NRF_DRV_TWI_FLAG_RX_POSTINC            |
                     NRF_DRV_TWI_FLAG_NO_XFER_EVT_HANDLER   |
                     NRF_DRV_TWI_FLAG_REPEATED_XFER;

    err_code = nrf_drv_twi_xfer(&m_twi, &xfer, flags);
    
    // TWIM is now configured and ready to be started.
    if (err_code == NRF_SUCCESS)
    {   
        // set up PPI to trigger the transfer
        uint32_t twi_start_task_addr = nrf_drv_twi_start_task_get(&m_twi, xfer.type);
        err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel1);
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_ppi_channel_assign(m_ppi_channel1,
                                              nrf_drv_timer_event_address_get(&m_timer1,
                                                                              NRF_TIMER_EVENT_COMPARE0),
                                              twi_start_task_addr);
        APP_ERROR_CHECK(err_code);
        
        // set up PPI to count the number of transfers 
        uint32_t twi_stopped_event_addr = nrf_drv_twi_stopped_event_get(&m_twi);
        err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel2);
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_ppi_channel_assign(m_ppi_channel2,
                                              twi_stopped_event_addr,
                                              nrf_drv_timer_task_address_get(&m_timer2,
                                                                             NRF_TIMER_TASK_COUNT));
    }
}

static void twi_accel_ppi_enable(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel1);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel2);
    APP_ERROR_CHECK(err_code);
}

void twi_start(void)
{
    // enable the counter counting
    nrf_drv_timer_enable(&m_timer2);
    
    // enable timer triggering TWI transfer
    nrf_drv_timer_enable(&m_timer1);
}


/**
 * @brief Function for main application entry.
 */
int main(void)

{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example");
    NRF_LOG_FLUSH();
    twi_init();
    LIS2DH_set_mode();

    twi_accel_ppi_init();
    twi_accel_ppi_enable();
    twi_start();

    
    while (true)
    {
        /*
        nrf_delay_ms(500);
        
        do
        {
            __WFE();
        }while (m_xfer_done == false);
        */
        //read_accel_data();
        //read_who();
        //NRF_LOG_INFO("\r\n who am i = %d", m_sample);

        //uint8_t *data;
        //NRF_LOG_INFO("TWI event number: %d", (int)m_evt_counter);
        
        for (uint8_t j=0; j<TWIM_RX_BUF_LENGTH; j++)
        {
              x = (((int8_t)p_rx_buffer[j].buffer[1]) << 8) + p_rx_buffer[j].buffer[0];
              y = (((int8_t)p_rx_buffer[j].buffer[3]) << 8) + p_rx_buffer[j].buffer[2];
              z = (((int8_t)p_rx_buffer[j].buffer[5]) << 8) + p_rx_buffer[j].buffer[4];
              NRF_LOG_INFO("%d, %d, %d", x, y, z);
        }
       

        NRF_LOG_FLUSH();
        
        //m_evt_counter++;
        m_xfer_done = false;

    }
}

/** @} */
