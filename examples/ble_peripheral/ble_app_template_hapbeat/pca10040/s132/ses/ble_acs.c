#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_ACS)
#include "ble_acs.h"
#include "ble_srv_common.h"


/* Function for handling the Connect event */
static void on_connect(ble_acs_t * p_acs, ble_evt_t const * p_ble_evt)
{
    p_acs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/* Function for handling the Disconnect event from the SoftDevice */
static void on_disconnect(ble_acs_t * p_acs, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_acs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/* Function for handling the write event from the SoftDevice */
// now this function is not needed. but if i want to receive data from smartphone, maybe need to change
static void on_write(ble_acs_t * p_acs, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (    (p_evt_write->handle == p_acs->tx_handles.cccd_handle)
        &&  (p_evt_write->len == 2))
    {
        if (p_acs->accel_write_handler == NULL)
        {
            return;
        }
        // maybe not needed
        ble_acs_evt_t evt;

        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            evt.evt_type = BLE_ACS_NOTIFICATION_ENABLED;
        }
        else
        {
            evt.evt_type = BLE_ACS_NOTIFICATION_DISABLED;
        }
        
        // CCCD written, call application event handler
        p_acs->accel_write_handler(p_acs, &evt);
    }
}

void ble_acs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL)) return;

    ble_acs_t * p_acs = (ble_acs_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
          on_connect(p_acs, p_ble_evt);
          break;

        case BLE_GAP_EVT_DISCONNECTED:
          on_disconnect(p_acs, p_ble_evt);
          break;

        case BLE_GATTS_EVT_WRITE:
          on_write(p_acs, p_ble_evt);
          break;

        default:
          // No implementation needed.
          break;
    }

}

/* Function for adding tx accelerometer characteristic */
static uint32_t tx_char_add(ble_acs_t * p_acs, const ble_acs_init_t * p_acs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_acs->uuid_type;
    ble_uuid.uuid = ACS_UUID_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t); // maybe change?
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_GATTS_FIX_ATTR_LEN_MAX;

    return sd_ble_gatts_characteristic_add(p_acs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_acs->tx_handles);

}


uint32_t ble_acs_init(ble_acs_t * p_acs, ble_acs_init_t const * p_acs_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t acs_base_uuid = ACS_UUID_BASE;

    VERIFY_PARAM_NOT_NULL(p_acs);
    VERIFY_PARAM_NOT_NULL(p_acs_init);

    // Initialize the service structure
    p_acs->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_acs->accel_write_handler     = p_acs_init->accel_write_handler;
    p_acs->is_notification_enabled = p_acs_init->notification_enabled;

    // Add a custom base UUID
    err_code = sd_ble_uuid_vs_add(&acs_base_uuid, &p_acs->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_acs->uuid_type;
    ble_uuid.uuid = ACS_UUID_SERVICE;

    // Add the service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_acs->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the accelerometer characteristic
    err_code = tx_char_add(p_acs, p_acs_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t ble_acs_accel_data_send(ble_acs_t * p_acs, uint8_t * p_data, uint16_t * p_length)
{
    ble_gatts_hvx_params_t hvx_params;
     
    VERIFY_PARAM_NOT_NULL(p_acs);

    if ((p_acs->conn_handle == BLE_CONN_HANDLE_INVALID)|| (!p_acs->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }
      
    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_acs->tx_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.p_data = p_data;
    hvx_params.p_len  = p_length;

    return  sd_ble_gatts_hvx(p_acs->conn_handle, &hvx_params);
    
}

#endif