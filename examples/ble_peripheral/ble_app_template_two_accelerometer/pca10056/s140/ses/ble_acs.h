#ifndef BLE_ACS_H__
#define BLE_ACS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

/* Macro for defining a ble_acs instance */
#define BLE_ACS_DEF(_name)                                      \
static ble_acs_t _name;                                         \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                             \
                     2,                                         \
                     ble_acs_on_ble_evt, &_name)

/* define original UUID */
#define ACS_UUID_BASE         {0xEA, 0x95, 0xE0, 0x84, 0x98, 0x13, 0x11, 0xEA,  \
                               0xBB, 0x37, 0x02, 0x42, 0xAC, 0x13, 0x00, 0x02}
                               
#define ACS_UUID_SERVICE    0xABCD
#define ACS_UUID_CHAR       0xABCE

/* Accelerometer Service event type */
typedef enum
{
    BLE_ACS_NOTIFICATION_ENABLED,
    BLE_ACS_NOTIFICATION_DISABLED
} ble_acs_evt_type_t;

/* Accelerometer Service event */
typedef struct
{
    ble_acs_evt_type_t evt_type;
} ble_acs_evt_t;

/* Forward declaration of the ble_acs_t type */
typedef struct ble_acs_s ble_acs_t;

/* Accelerometer Service event handler type */
typedef void (*ble_acs_accel_write_handler_t) (ble_acs_t * p_acs, ble_acs_evt_type_t * p_evt);

/* Accelerometer Service initialization structure 

   This structure contains status information related to the service.
 */
typedef struct
{
    ble_acs_accel_write_handler_t accel_write_handler;
    bool                          notification_enabled;
} ble_acs_init_t;

/* Accelerometer Service structure */
struct ble_acs_s
{
    uint8_t                       uuid_type;
    uint16_t                      service_handle;
    ble_gatts_char_handles_t      tx_handles;
    uint16_t                      conn_handle;
    ble_acs_accel_write_handler_t accel_write_handler;
    bool                          is_notification_enabled;
};

/* Function for initializing Accelerometer Service */
uint32_t ble_acs_init(ble_acs_t * p_acs, ble_acs_init_t const * p_acs_init);

/* Function for handling Accelerometer Service BLE  events*/
void ble_acs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/* Function for sends the accel data to the smartphone */
uint32_t ble_acs_accel_data_send(ble_acs_t * p_acs, uint8_t * p_data, uint16_t * p_length);

#endif