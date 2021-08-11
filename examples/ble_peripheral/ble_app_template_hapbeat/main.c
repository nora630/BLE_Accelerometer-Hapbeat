/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
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
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "ble_hpbs.h"

#include "boards.h"
#include "app_util_platform.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_pwm.h"
//#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_queue.h"
#include "app_scheduler.h"
#include "arm_const_structs.h"
#include "adpcm.h"


#define DEVICE_NAME                     "hapbeat"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION                18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define PWM_INTERVAL                    APP_TIMER_TICKS(1)
#define NOISE_CUT_INTERVAL              APP_TIMER_TICKS(128)

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (10 milliseconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (10 millisecond). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */
//#define APP_ADV_TIMEOUT_IN_SECONDS      180                                     /**< The advertising timeout in units of seconds. */
#define HPBS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */


#define PWM_PIN  (3)
#define STBY_PIN (4)
#define IN1_PIN  (20)
#define IN2_PIN  (28)


#define PPI_TIMER3_INTERVAL   (1) // Timer interval in milliseconds, this is PWM Play interval. 


/* buffer size */
#define BLE_BUF_WIDTH    20
#define BLE_BUF_LENGTH   5

#define NOISE_CUT_LENGTH    128

static struct ADPCMstate state;

// pwm variable
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static const nrf_drv_timer_t m_timer3 = NRF_DRV_TIMER_INSTANCE(3);
static nrf_ppi_channel_t     m_ppi_channel3;


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

static int32_t sum = 0;
static int16_t x;
static int16_t y;
static int16_t z;

static uint8_t code;
static bool decode_flag = false;


BLE_HPBS_DEF(m_hpbs);                                                             // Hapbeat Service instance
APP_TIMER_DEF(m_pwm_timer_id);                                                    /**< PWM timer. */
APP_TIMER_DEF(m_noise_cut_id);                                                    // noise cut timer


static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

/* filter setting */
static  float in1, in2, out1, out2;
static  float a0, a1, a2, b0, b1, b2;

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

static float32_t maxH[] = {
350.724976,
276.459717,
261.41864,
233.233215,
234.194702,
270.225037,
226.98143,
269.698853,
268.461151,
262.180939,
267.006134,
233.544617,
278.873047,
176.518646,
233.387527,
232.008636,
257.125549,
188.226166,
222.72966,
276.903442,
182.567688,
175.039169,
225.598541,
171.719086,
147.93544,
158.23204,
146.057648,
163.926163,
126.691414,
146.816483,
131.059311,
126.536362,
149.482437,
98.119461,
104.944954,
107.108101,
111.009338,
101.710991,
89.290054,
83.71656,
73.245529,
74.694931,
86.293861,
64.300171,
61.497021,
46.326847,
53.369846,
66.486992,
79.633041,
55.820625,
77.234825,
55.628502,
72.608147,
71.01947,
72.920654,
67.440834,
58.589024,
69.17672,
77.956528,
73.891441,
85.439651,
76.184097,
75.869949,
78.797493,
150,
78.797485,
75.869949,
76.184097,
85.439651,
73.891441,
77.956528,
69.176712,
58.589024,
67.440849,
72.920654,
71.019478,
72.608147,
55.628498,
77.234825,
55.820633,
79.633041,
66.486992,
53.369835,
46.326847,
61.497021,
64.300171,
86.293861,
74.694946,
73.245529,
83.71656,
89.290054,
101.710991,
111.009338,
107.108101,
104.944954,
98.119461,
149.482437,
126.536362,
131.059311,
146.816498,
126.691422,
163.926163,
146.057648,
158.23204,
147.93544,
171.71907,
225.598495,
175.039169,
182.567688,
276.903442,
222.72966,
188.226166,
257.125549,
232.008636,
233.387558,
176.518646,
278.873047,
233.544617,
267.006134,
262.180939,
268.461151,
269.698853,
226.98143,
270.225037,
234.194702,
233.233231,
261.41864,
276.459717,
          
          };

const   int16_t bandsize = 27;

static int32_t bandIn[26];

static uint8_t aData[20];
const  uint8_t alen = 20;

//uint16_t dat = 0;

NRF_QUEUE_DEF(uint8_t, m_byte_queue, 4096, NRF_QUEUE_MODE_NO_OVERFLOW);
NRF_QUEUE_DEF(int32_t, m_play_queue, 2048, NRF_QUEUE_MODE_NO_OVERFLOW);

/* Buufer for samples read from a smartphone */
typedef struct ArrayList
{
    uint8_t buffer[BLE_BUF_WIDTH];
} array_list_t;

static array_list_t ble_buffer[BLE_BUF_LENGTH];

static uint8_t index = 0;

volatile static flag = false;

static arm_rfft_fast_instance_f32   fft_inst;


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

    // cut f = 0.001 * 500 Hz
    a0 =   1;
    a1 =   -0.9969;
    //a2 =   1.0f - alpha;
    b0 =  0.9984;
    b1 = -0.9984;
    //b2 =  (1.0f + cos(omega)) / 2.0f;

    /*
    // cut f = 0.01 * 500 Hz
    a0 =   1;
    a1 =   -0.9691;
    //a2 =   1.0f - alpha;
    b0 =  0.9845;
    b1 = -0.9845;
    //b2 =  (1.0f + cos(omega)) / 2.0f;
    */

    /*
    // cut f = 0.1 * 500 Hz
    a0 = 1;
    a1 = -0.7265;
    b0 = 0.8633;
    b1 = -0.8633;
    */
}

void band_filter_set(void)
{
    for(int16_t i=0; i<bandsize-1; i++) bandIn[i] = 0;
    for(int16_t i=0; i<bandsize; i++){
        if(i==0) bandpass[i] += 1;
        else bandpass[i] *= 8;
    }
    
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

/* PWM functions */
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

static void smb_motor_pin_init(void)
{
    nrf_gpio_cfg_output(STBY_PIN);
    nrf_gpio_cfg_output(IN1_PIN);
    nrf_gpio_cfg_output(IN2_PIN);

    nrf_gpio_pin_set(STBY_PIN);
}

/* PWM play handler */
static void timer3_handler(nrf_timer_event_t event_type, void * p_context)
{
    uint16_t *p_channels = (uint16_t *)&m_seq_values;
        //if (!flag) return;
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
        

        //sum = aData[index];
        

        //sum = isqrt(sum);
        
        if(!nrf_queue_is_empty(&m_byte_queue))
        {
            //ret_code_t err_code = nrf_queue_pop(&m_byte_queue, &dat);
            //APP_ERROR_CHECK(err_code);
            //sum = dat;
            //sum = bandFilter(sum);
            //sum = filter(sum);
        }

        //sum = dat;


        //printf("%d\n", sum);
        //sum -= 64;
        //printf("%d\n", sum);
        //printf("%d\n", sum);
        //sum = lfilter(sum);

        
        
        //sum = bandFilter(sum);
        //sum = filter(sum);
       


        //if(sum<0) sum *= -1;
        //NRF_LOG_INFO("%d", sum);
        //NRF_LOG_FLUSH();

        //printf("%d\n", sum);

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
        /*
        index++;
        if(index>=20) 
        {
            index = 0;
            //flag = false;
        }
        */
        //if(index>=12) index = 0;
}

// Function for Timer 3 initialization
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

    // setup timer3
    timer3_init();

    
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


static void ppi_enable(void)
{
    ret_code_t err_code;
    
    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel3);
    APP_ERROR_CHECK(err_code);
}

void ppi_start(void)
{
    // enable timer triggering pwm
     nrf_drv_timer_enable(&m_timer3);

}

/*******************************************************************************************************/

static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    //{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    {HPBS_UUID_SERVICE, HPBS_SERVICE_UUID_TYPE}
};



static void advertising_start(bool erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}

static void pwm_update(void)
{
    uint16_t *p_channels = (uint16_t *)&m_seq_values;

    if(decode_flag){
        sum = ADPCMDecoder(code & 0x0f, &state);
        sum = filter(sum);
        decode_flag = false;
    } else {
        if(!nrf_queue_is_empty(&m_byte_queue)){
            ret_code_t err_code = nrf_queue_pop(&m_byte_queue, &code);
            sum = ADPCMDecoder((code >> 4) & 0x0f, &state);
            sum = filter(sum);
            decode_flag = true;
        }
    }
    /*
    if(!nrf_queue_is_empty(&m_byte_queue))
    {
        ret_code_t err_code = nrf_queue_pop(&m_byte_queue, &code);
        //APP_ERROR_CHECK(err_code);
        sum = dat;
        //sum = bandFilter(sum);
        sum = filter(sum);
    }*/
    
    /*
    if(!nrf_queue_is_empty(&m_play_queue))
    {
        ret_code_t err_code = nrf_queue_pop(&m_play_queue, &sum);
        //APP_ERROR_CHECK(err_code);
        sum = bandFilter(sum);
        //sum = filter(sum);
    } */

    //printf("%d\n", sum);
    
    if(sum>=0) motor_forward();
    else{
        motor_back();
        //sum *= -1;
    }
    
    // 8192 x+y+z  32 x  
    uint16_t value = m_motor_top - abs(sum);//8192;//32 x;
    //uint16_t value = m_motor_top - m_motor_top * sum / 60;//8192;//32 x;
    if(value > m_motor_top) value = m_motor_top;
    else if(value < 0) value = 0;
    p_channels[0] = value;

    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &m_seq, 40, NRF_DRV_PWM_FLAG_LOOP);
       
}

static void pwm_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    pwm_update();
}

static void noise_cut_update(void)
{
    float32_t fft_in[NOISE_CUT_LENGTH*2], fft_out[NOISE_CUT_LENGTH*2], fft_mag[NOISE_CUT_LENGTH], h[NOISE_CUT_LENGTH];
    uint8_t dat;
    int32_t d;
    if(nrf_queue_utilization_get(&m_byte_queue)>=NOISE_CUT_LENGTH/2)
    {
        for(uint16_t i=0; i<NOISE_CUT_LENGTH/2; i++)
        {
            ret_code_t err_code = nrf_queue_pop(&m_byte_queue, &dat);
            d = ADPCMDecoder((dat >> 4) & 0x0f, &state);
            //printf("%d\n", d);
            d = filter(d);
            //printf("%d\n", d);
            fft_in[4*i] = d;
            fft_in[4*i+1] = 0;

            d = ADPCMDecoder(dat & 0x0f, &state);
            //printf("%d\n", d);
            d = filter(d);
            //printf("%d\n", d);
            fft_in[4*i+2] = d;
            fft_in[4*i+3] = 0;
            //APP_ERROR_CHECK(err_code);
            /*
            d = (dat << 8) & 0xff00;
            err_code = nrf_queue_pop(&m_byte_queue, &dat);
            d |= dat & 0xff;
            d = filter(d);
            fft_in[2*i] = d;
            fft_in[2*i+1] = 0;
            */
        }
        
        
        arm_rfft_fast_f32(&fft_inst, fft_in, fft_out, 0);
        arm_cmplx_mag_f32(fft_out, fft_mag, NOISE_CUT_LENGTH);

        for(uint16_t i=0; i<NOISE_CUT_LENGTH; i++)
        {
            h[i] = 1 - (maxH[i]) / fft_mag[i];
            if(h[i]<0) h[i] = 0;
            fft_out[2*i] = h[i] * fft_out[2*i];
            fft_out[2*i+1] = h[i] * fft_out[2*i+1];
        }
        arm_rfft_fast_f32(&fft_inst, fft_out, fft_in, 1);
        
        for(uint16_t i=0; i<NOISE_CUT_LENGTH; i++)
        {
            int32_t dd;
            if(!nrf_queue_is_full(&m_play_queue))
            {
                dd = fft_in[2*i];
                ret_code_t err_code = nrf_queue_push(&m_play_queue, &dd);
                //APP_ERROR_CHECK(err_code);
            }
        }
    }    
}

static void noise_cut_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    noise_cut_update();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_pwm_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                pwm_timeout_handler);
    APP_ERROR_CHECK(err_code);

    /*
    err_code = app_timer_create(&m_noise_cut_id,
                                APP_TIMER_MODE_REPEATED,
                                noise_cut_timeout_handler);
    APP_ERROR_CHECK(err_code);
    */
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(0);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

void adpcm_state_init(void)
{
    state.prevsample = 0;
    state.previndex = 0;
}

/* Function for receiving the data from the smartphone */
static void accel_read_handler(ble_hpbs_evt_t * p_evt)
{
    if(p_evt->type == BLE_HPBS_EVT_RX_DATA)
    {
         ret_code_t err_code;
         NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

         for(uint16_t i=0; i<p_evt->params.rx_data.length; i++)
         {
            //printf("%d\n", p_evt->params.rx_data.p_data[i]);

            //aData[i] = p_evt->params.rx_data.p_data[i];
            
            //ble_buffer[0].buffer[i] = p_evt->params.rx_data.p_data[i];
            
            //NRF_LOG_INFO("%d: %d", i, aData[i]);

            uint8_t data = p_evt->params.rx_data.p_data[i];
            if(!nrf_queue_is_full(&m_byte_queue))
            {
                err_code = nrf_queue_push(&m_byte_queue, &data);
                //APP_ERROR_CHECK(err_code);
            }

         }
         //flag = true;
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    ble_hpbs_init_t hpbs_init;

    memset(&hpbs_init, 0, sizeof(hpbs_init));

    hpbs_init.accel_read_handler = accel_read_handler;

    err_code = ble_hpbs_init(&m_hpbs, &hpbs_init);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(m_pwm_timer_id, PWM_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    /*
    err_code = app_timer_start(m_noise_cut_id, NOISE_CUT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    */
}

static void application_timers_stop(void)
{
    ret_code_t err_code;
    err_code = app_timer_stop(m_pwm_timer_id);
    APP_ERROR_CHECK(err_code);
    /*
    err_code = app_timer_stop(m_noise_cut_id);
    APP_ERROR_CHECK(err_code);
    */
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    sum = 0;
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            application_timers_stop();
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            application_timers_start();
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;


    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    pwm_init();
    //arm_rfft_fast_init_f32(&fft_inst, NOISE_CUT_LENGTH*2);
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    //advertising_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();

    high_filter_set();
    band_filter_set();

    smb_motor_pin_init();
    //pwm_init();

    //ppi_init();
    //ppi_enable();
    //ppi_start();

    // Start execution.
    NRF_LOG_INFO("Template example started.");
    //NRF_LOG_FLUSH();
    //application_timers_start();
    adpcm_state_init();
    advertising_start(erase_bonds);
    NRF_LOG_FLUSH();

    
    // Enter main loop.
    do {
      //app_sched_execute();
      idle_state_handle();
    } while (true);

}


/**
 * @}
 */
