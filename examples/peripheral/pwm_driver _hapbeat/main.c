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
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 */

#include <stdio.h>
#include <string.h>
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "bsp.h"
#include "app_timer.h"
//#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_twi.h"
#include <math.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define PWM_PIN  (3)
#define STBY_PIN (4)
#define IN1_PIN  (20)
#define IN2_PIN  (28)

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common address for LIS2DH */
#define LIS2DH_ADDR      0x18U

#define LIS2DH_CTRL_REG   0xA0U
#define LIS2DH_DATA_REG   0xA8U
#define WHO_AM_I          0x0FU
#define LIS2DH_CTRL_REG1  0x20U
#define LIS2DH_CTRL_REG4  0x23U
#define LIS2DH_CTRL_REG5  0x24U
#define FIFO_CTRL_REG     0x2EU
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

#define FIFO_ENABLE       0xC0U // FIFO enable
#define STREAM_MODE       0x80U // Stream mode
#define FIFO_MODE         0x40U //FIFO mode


/* PPI */
#define PPI_TIMER1_INTERVAL   (1) // Timer interval in milliseconds, this is twi sampling rate. 
#define PPI_TIMER3_INTERVAL   (1) // Timer interval in milliseconds, this is PWM Play interval. 

/* buffer size */
#define TWIM_RX_BUF_WIDTH    6
#define TWIM_RX_BUF_LENGTH  1

// pwm variable
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

static uint16_t const           m_motor_top  = 400;
static uint16_t                 m_motor_step1 = 400;
static uint16_t                 m_motor_step2 = 200;

static nrf_pwm_values_individual_t m_seq_values;
static nrf_pwm_sequence_t const    m_seq =
{
    .values.p_individual = &m_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};

// twi variable
 /* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from accelerometer */
typedef struct ArrayList
{
    uint8_t buffer[TWIM_RX_BUF_WIDTH];
} array_list_t;

static array_list_t p_rx_buffer[TWIM_RX_BUF_LENGTH];
static int16_t x;
static int16_t y;
static int16_t z;
static int32_t sum = 0;
static int32_t preSum = 0;
static int32_t tmp;

static uint8_t m_sensorData[6];
static uint8_t m_dataReg[1] = {LIS2DH_DATA_REG};

/* ppi setting */
static const nrf_drv_timer_t m_timer1 = NRF_DRV_TIMER_INSTANCE(1);
static const nrf_drv_timer_t m_timer2 = NRF_DRV_TIMER_INSTANCE(2);
static const nrf_drv_timer_t m_timer3 = NRF_DRV_TIMER_INSTANCE(3);

static nrf_ppi_channel_t     m_ppi_channel1;
static nrf_ppi_channel_t     m_ppi_channel2;
static nrf_ppi_channel_t     m_ppi_channel3;

/* filter setting */
static  float in1, in2, out1, out2;
static  float omega, alpha;
static  float a0, a1, a2, b0, b1, b2;
const   float freq = 0.01; //62.4                       // Cutoff frequency
const   float samplerate = 800;
const   float q = 0.71;                         //  q_value
const   float bw = 3.12;

static  float lin1, lin2, lout1, lout2;
static  float lomega, lalpha;
static  float la0, la1, la2, lb0, lb1, lb2;
const   float lfreq = 399.2;

static   float bandpass[] = {
             0.0000000000000000 ,
            -0.0000248598847283 ,
            0.0001516523246382 ,
            -0.0001834267075605 ,
            0.0019997002466150 ,
            0.0085820310275102 ,
            -0.0018208819783212 ,
            0.0228852711768900 ,
            -0.0180580186076670 ,
            -0.0542808170366290 ,
            -0.0099299349850613 ,
            -0.2702649781457600 ,
            0.0286165165403290 ,
            0.5959259259259300 ,
            0.0286165165403290 ,
            -0.2702649781457600 ,
            -0.0099299349850613 ,
            -0.0542808170366290 ,
            -0.0180580186076670 ,
            0.0228852711768900 ,
            -0.0018208819783212 ,
            0.0085820310275102 ,
            0.0019997002466150 ,
            -0.0001834267075605 ,
            0.0001516523246382 ,
            -0.0000248598847283 ,
            0.0000000000000000 ,

          };
const   int16_t bandsize = 27;

static int32_t bandIn[26];


//**************************************** etc functions *********************************//
// sqrt function
int32_t isqrt(int32_t num) {
    //assert(("sqrt input should be non-negative", num > 0));
    int32_t res = 0;
    int32_t bit = 1 << 30; // The second-to-top bit is set.
                           // Same as ((unsigned) INT32_MAX + 1) / 2.

    // "bit" starts at the highest power of four <= the argument.
    while (bit > num)
        bit >>= 2;

    while (bit != 0) {
        if (num >= res + bit) {
            num -= res + bit;
            res = (res >> 1) + bit;
        } else
            res >>= 1;
        bit >>= 2;
    }
    return res;
}

// initialize  BiQuad high-pass filter coefficient
void high_filter_set(void)
{
    in1 = 0;
    //in2 = 0;
    out1 = 0;
    //out2 = 0;
    //omega = 2.0f * 3.14159265f * freq / samplerate;
    //alpha = sin(omega) / (2.0f * q);
    
    a0 =   1;
    a1 =   -0.9691;
    //a2 =   1.0f - alpha;
    b0 =  0.9845;
    b1 = -0.9845;
    //b2 =  (1.0f + cos(omega)) / 2.0f;
}

void band_filter_set(void)
{
    for(int16_t i=0; i<bandsize-1; i++) bandIn[i] = 0;
    for(int16_t i=0; i<bandsize; i++){
        if(i==0) bandpass[i] += 1;
        else bandpass[i] *= 4;
    }
    
}

void low_filter_set(void)
{
    lin1 = 0;
    lin2 = 0;
    lout1 = 0;
    lout2 = 0;
    lomega = 2.0f * 3.14159265f * lfreq / samplerate;
    lalpha = sin(lomega) / (2.0f * q);
    
    la0 =   1.0f + lalpha;
    la1 =  -2.0f * cos(lomega);
    la2 =   1.0f - lalpha;
    lb0 =  (1.0f - cos(lomega)) / 2.0f;
    lb1 =   1.0f - cos(lomega);
    lb2 =  (1.0f - cos(lomega)) / 2.0f;
}

// apply filter
int32_t filter(int32_t input)
{
    float output;
    output = b0/a0 * (float)input + b1/a0 * in1 - a1/a0 * out1;
    in1 = input;
    out1 = output;
  
    return (int32_t)output;
}

int32_t lfilter(int32_t input)
{
    float output;
    output = lb0/la0 * (float)input + lb1/la0 * lin1 + lb2/la0 * lin2
                          - la1/la0 * lout1 - la2/la0 * lout2;
    lin2 = lin1;
    lin1 = input;

    lout2 = lout1;
    lout1 = output;
  
    return (int32_t)output;
}

int32_t bandFilter(int32_t input)
{
    float output = 0;
    for(int16_t i=0; i<bandsize; i++){
        if(i==0) output += bandpass[i] * input;
        else output += bandpass[i] * bandIn[bandsize-1-i];
    }

    int32_t temp1, temp2;

    for(int16_t i=0; i<bandsize-1; i++){
        if(i==0) {
            temp1 = bandIn[bandsize-2];
            //bandIn[bandsize-2] = (int32_t)output;
            bandIn[bandsize-2] = input;
        } else {
            temp2 = bandIn[bandsize-2-i];
            bandIn[bandsize-2-i] = temp1;
            temp1 = temp2;
        }
    }

    return (int32_t)output;
}
//**************************************** PWM functions *********************************//

/*
static void pwm_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
        uint16_t *p_channels = (uint16_t *)&m_seq_values;
        uint16_t value = p_channels[0];
        value += 2;
        if(value>=400) value = 0;
        p_channels[0] = value;
        p_channels[1] = value | 0x8000;
    }
}

static void pwm_start(void)
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            IN1_PIN, // channel 0
            IN2_PIN, // channel 1
            NRF_DRV_PWM_PIN_NOT_USED, // channel 2
            NRF_DRV_PWM_PIN_NOT_USED  // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = m_motor_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, pwm_handler));

    /*
    static nrf_pwm_values_individual_t seq_values[] = 
    {
        {201, 201 | 0x8000, 0, 0}
    };

    nrf_pwm_sequence_t const seq =
    {
        .values.p_individual = seq_values,
        .length              = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats             = 0,
        .end_delay           = 0
    };
    
    m_seq_values.channel_0 = m_motor_step1;
    m_seq_values.channel_1 = m_motor_step1 | 0x8000;
    m_seq_values.channel_2 = 0;
    m_seq_values.channel_3 = 0;


    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &m_seq, 40,
                                      NRF_DRV_PWM_FLAG_LOOP);

}

*/

static void motor_forward(void)
{
    nrf_gpio_pin_set(IN1_PIN);
    nrf_gpio_pin_clear(IN2_PIN);
}

static void motor_back(void)
{
    nrf_gpio_pin_set(IN2_PIN);
    nrf_gpio_pin_clear(IN1_PIN);
}


volatile bool flag = true;

/*
static void pwm_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
        uint16_t *p_channels = (uint16_t *)&m_seq_values;
        /*
        uint16_t value = p_channels[0];
        value += 1;
        if(value>=400) 
        {
            value = 0;
        }
        if(flag) motor_forward();
        else motor_back();
        flag = !flag;
        p_channels[0] = value;
        
        //x = (((int8_t)p_rx_buffer[0].buffer[1]) << 8) + p_rx_buffer[0].buffer[0];
        //y = (((int8_t)p_rx_buffer[0].buffer[3]) << 8) + p_rx_buffer[0].buffer[2];
        //z = (((int8_t)p_rx_buffer[0].buffer[5]) << 8) + p_rx_buffer[0].buffer[4];

        x = (int16_t)p_rx_buffer[0].buffer[1]; 
        y = (int16_t)p_rx_buffer[0].buffer[3];
        z = (int16_t)p_rx_buffer[0].buffer[5];
        sum = x * x + y * y + z * z;
        //sum = sqrt(sum);
        tmp = sum - preSum;
        if(tmp>=0) motor_forward();
        else{
            motor_back();
            tmp *= -1;
        }
        // 8192 x+y+z  32 x  
        uint16_t value = m_motor_top - m_motor_top * tmp / 8192;//8192;//32 x;
        //uint16_t value = m_motor_top - m_motor_top * sum / 8192;//8192;//32 x;
        if(value > m_motor_top) value = m_motor_top;
        else if(value < 0) value = 0;
        p_channels[0] = value;
        preSum = sum;
    }
}
*/

static void pwm_init(void)
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            PWM_PIN, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED, // channel 1
            NRF_DRV_PWM_PIN_NOT_USED, // channel 2
            NRF_DRV_PWM_PIN_NOT_USED  // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = m_motor_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };

    //APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, pwm_handler));
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
    
    m_seq_values.channel_0 = m_motor_step1;
    m_seq_values.channel_1 = 0;
    m_seq_values.channel_2 = 0;
    m_seq_values.channel_3 = 0;


    //(void)nrf_drv_pwm_simple_playback(&m_pwm0, &m_seq, 100,
    //                                  NRF_DRV_PWM_FLAG_LOOP);

}

/*
static void motor_pin_init(void)
{
    nrf_gpio_cfg_output(STBY_PIN);
    nrf_gpio_cfg_output(PWM_PIN);

    nrf_gpio_pin_set(STBY_PIN);
}
*/

static void smb_motor_pin_init(void)
{
    nrf_gpio_cfg_output(STBY_PIN);
    nrf_gpio_cfg_output(IN1_PIN);
    nrf_gpio_cfg_output(IN2_PIN);

    nrf_gpio_pin_set(STBY_PIN);
}

//********************************  twi functions ****************************//

/* Function for setting LIS2DH accelerometer */
void LIS2DH_set_mode(void)
{
    ret_code_t err_code;
    
    /* Writing to LIS2DH_CTR_REG set range and Low Power mode */
    //uint8_t reg[7] = {LIS2DH_CTRL_REG, LOW_POWER_MODE1, 0x00, 0x00, LIS2DH_RANGE_2GA, 0x00, 0x00};
    uint8_t reg[2] = {LIS2DH_CTRL_REG1, LOW_POWER_MODE2};
    //uint8_t reg[1] = {LIS2DH_CTRL_REG1};
    err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    m_xfer_done = false;

    reg[0] = LIS2DH_CTRL_REG5; reg[1] = FIFO_ENABLE;
    err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    m_xfer_done = false;

    reg[0] = FIFO_CTRL_REG; reg[1] = STREAM_MODE;
    err_code = nrf_drv_twi_tx(&m_twi, LIS2DH_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
    m_xfer_done = false;

}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    //nrf_gpio_cfg_input(ARDUINO_SCL_PIN, NRF_GPIO_PIN_PULLUP);
    //nrf_gpio_cfg_input(ARDUINO_SDA_PIN, NRF_GPIO_PIN_PULLUP);
    //nrf_delay_us(4);

    const nrf_drv_twi_config_t twi_lis2dh_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true
    };

    //nrf_gpio_cfg_input(ARDUINO_SCL_PIN, NRF_GPIO_PIN_PULLUP);
    //nrf_gpio_cfg_input(ARDUINO_SDA_PIN, NRF_GPIO_PIN_PULLUP);
    //nrf_delay_us(4);

    err_code = nrf_drv_twi_init(&m_twi, &twi_lis2dh_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


//********************************  ppi functions ****************************//

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

/* PWM play handler */
static void timer3_handler(nrf_timer_event_t event_type, void * p_context)
{
    uint16_t *p_channels = (uint16_t *)&m_seq_values;
        /*
        uint16_t value = p_channels[0];
        value += 1;
        if(value>=400) 
        {
            value = 0;
        }
        if(flag) motor_forward();
        else motor_back();
        flag = !flag;
        p_channels[0] = value;
        */
        //x = (((int8_t)p_rx_buffer[0].buffer[1]) << 8) + p_rx_buffer[0].buffer[0];
        //y = (((int8_t)p_rx_buffer[0].buffer[3]) << 8) + p_rx_buffer[0].buffer[2];
        //z = (((int8_t)p_rx_buffer[0].buffer[5]) << 8) + p_rx_buffer[0].buffer[4];

        x = (int8_t)p_rx_buffer[0].buffer[1]; 
        y = (int8_t)p_rx_buffer[0].buffer[3];
        z = (int8_t)p_rx_buffer[0].buffer[5];
        sum = x * x + y * y + z * z;
        sum = isqrt(sum);
        //printf("%d\n", sum);
        //printf("%d\n", sum);
        //sum -= 64;
        //printf("%d\n", sum);
        //printf("%d\n", sum);
        //sum = lfilter(sum);
        sum = bandFilter(sum);
        sum = filter(sum);
        //if(sum<0) sum *= -1;
        //NRF_LOG_INFO("%d", sum);
        //NRF_LOG_FLUSH();

        printf("%d\n", sum);


        //tmp = sum - preSum;
        //motor_forward();
        //printf("%d\n", tmp);
        
        if(sum>=0) motor_forward();
        else{
            motor_back();
            sum *= -1;
        }
        
        // 8192 x+y+z  32 x  
        uint16_t value = m_motor_top - 7.5 * sum;//8192;//32 x;
        //uint16_t value = m_motor_top - m_motor_top * sum / 60;//8192;//32 x;
        if(value > m_motor_top) value = m_motor_top;
        else if(value < 0) value = 0;
        p_channels[0] = value;
        preSum = sum;
}

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

// Function for Timer 2 initialization
static void timer3_init(void)
{
    nrf_drv_timer_config_t timer3_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer3_config.frequency = NRF_TIMER_FREQ_31250Hz;
    ret_code_t err_code = nrf_drv_timer_init(&m_timer3, &timer3_config, timer3_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_extended_compare(&m_timer3,
                                   NRF_TIMER_CC_CHANNEL0,
                                   nrf_drv_timer_ms_to_ticks(&m_timer3,
                                                             PPI_TIMER3_INTERVAL),
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
}

/* initialize twi & pwm ppi channel */
static void ppi_init(void)
{
    ret_code_t err_code;
    
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    // setup timer1
    timer1_init();

    //setup timer2
    timer2_init();

    // setup timer3
    timer3_init();

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
        APP_ERROR_CHECK(err_code);

        // set up PPI to play pwm
        uint32_t pwm_start_task_addr = nrf_drv_pwm_simple_playback(&m_pwm0, &m_seq,100, // more than 40
                                            NRF_DRV_PWM_FLAG_STOP | NRF_DRV_PWM_FLAG_START_VIA_TASK);
        err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel3);
        APP_ERROR_CHECK(err_code);
        err_code = nrf_drv_ppi_channel_assign(m_ppi_channel3,
                                              nrf_drv_timer_event_address_get(&m_timer3,
                                                                              NRF_TIMER_EVENT_COMPARE0),
                                              pwm_start_task_addr);
        APP_ERROR_CHECK(err_code);

    }
}


static void ppi_enable(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel1);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel2);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel3);
    APP_ERROR_CHECK(err_code);
}

void ppi_start(void)
{
    // enable timer triggering pwm
     nrf_drv_timer_enable(&m_timer3);

    // enable the counter counting
    nrf_drv_timer_enable(&m_timer2);
    
    // enable timer triggering TWI transfer
    nrf_drv_timer_enable(&m_timer1);
}




int main(void)
 {
   
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    //low_filter_set();
    high_filter_set();
    band_filter_set();

    smb_motor_pin_init();
    pwm_init();
    
    twi_init();
    LIS2DH_set_mode();

    ppi_init();
    ppi_enable();
    ppi_start();
    

    //nrf_gpio_pin_set(PWM_PIN);
  
    //bool flag = true;
    //int count = 0;
  
    for (;;)
    { 
        //nrf_delay_ms(500);
        //NRF_LOG_FLUSH();
        /*
        do
        {
            __WFE();
        }while (m_xfer_done == false);
        */
        //m_xfer_done = false;
        /*
        nrf_delay_ms(500);
        count++;
        NRF_LOG_INFO("%d", count);
        NRF_LOG_FLUSH();
        if(count==10)
        {
           nrf_gpio_pin_clear(PWM_PIN);
        }
        // Wait for an event.
        //__WFE();
        /*
        if(flag)
        {
            motor_forward();
        } else
        {
            motor_back();
        }
        flag = !flag;
        */
    }
}


/** @} */
