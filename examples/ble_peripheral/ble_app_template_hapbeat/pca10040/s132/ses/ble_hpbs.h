#ifndef BLE_HPBS_H__
#define BLE_HPBS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

/* Macro for defining a ble_hpbs instance */
#define BLE_HPBS_DEF(_name)                                      \
static ble_hpbs_t _name;                                         \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                             \
                     2,                                         \
                     ble_hpbs_on_ble_evt, &_name)

/* define original UUID */
#define HPBS_UUID_BASE         {0x8D, 0xEA, 0x3B, 0x2B, 0x2E, 0x01, 0x4B, 0xE9,  \
                               0xA6, 0x40, 0x86, 0xB6, 0x94, 0xB4, 0x3E, 0xAD}
                               
#define HPBS_UUID_SERVICE    0xBCDE
#define HPBS_UUID_CHAR       0xBCDF

/* Hapbeat Service event type */
typedef enum
{
    BLE_HPBS_EVT_RX_DATA,
} ble_hpbs_evt_type_t;


/* Forward declaration of the ble_hpbs_t type */
typedef struct ble_hpbs_s ble_hpbs_t;

/* Hapbeat Service event data */
typedef struct
{
    uint8_t const * p_data;           /**< A pointer to the buffer with received data. */
    uint16_t        length;           /**< Length of received data. */
} ble_hpbs_evt_rx_data_t;

/* Hapbeat Service event */
typedef struct
{
    ble_hpbs_evt_type_t type;
    ble_hpbs_t * p_hpbs;
    union
    {
        ble_hpbs_evt_rx_data_t rx_data;
    } params;
} ble_hpbs_evt_t;


/* Hapbeat Service event handler type */
typedef void (*ble_hpbs_accel_read_handler_t) (ble_hpbs_evt_type_t * p_evt);

/* 
   Hapbeat Service initialization structure 

   This structure contains status information related to the service.
*/
typedef struct
{
    ble_hpbs_accel_read_handler_t accel_read_handler;
} ble_hpbs_init_t;

/* Hapbeat Service structure */
struct ble_hpbs_s
{
    uint8_t                       uuid_type;
    uint16_t                      service_handle;
    ble_gatts_char_handles_t      rx_handles;
    uint16_t                      conn_handle;
    ble_hpbs_accel_read_handler_t accel_read_handler;
};

/* Function for initializing Hapbeat Service */
uint32_t ble_hpbs_init(ble_hpbs_t * p_hpbs, ble_hpbs_init_t const * p_hpbs_init);

/* Function for handling Hapbeat Service BLE  events*/
void ble_hpbs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/* Function for receiving the accel data from the smartphone */
//uint32_t ble_hpbs_accel_data_receive(ble_hpbs_t * p_hpbs, uint8_t * p_data, uint16_t * p_length);

#endif