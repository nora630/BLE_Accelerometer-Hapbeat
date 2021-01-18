#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_HPBS)
#include "ble_hpbs.h"
#include "ble_srv_common.h"


/* Function for handling the Connect event */
static void on_connect(ble_hpbs_t * p_hpbs, ble_evt_t const * p_ble_evt)
{
    p_hpbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/* Function for handling the Disconnect event from the SoftDevice */
static void on_disconnect(ble_hpbs_t * p_hpbs, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_hpbs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/* Function for handling the write event from the SoftDevice */
static void on_write(ble_hpbs_t * p_hpbs, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    ble_hpbs_evt_t evt;
    evt.p_hpbs = p_hpbs;

   if (   (p_evt_write->handle == p_hpbs->rx_handles.value_handle)
             && (p_hpbs->accel_read_handler != NULL))
    {
        evt.params.rx_data.p_data = p_evt_write->data;
        evt.params.rx_data.length = p_evt_write->len;
        evt.type = BLE_HPBS_EVT_RX_DATA;
        p_hpbs->accel_read_handler(&evt);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}

void ble_hpbs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL)) return;

    ble_hpbs_t * p_hpbs = (ble_hpbs_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
          on_connect(p_hpbs, p_ble_evt);
          break;

        case BLE_GAP_EVT_DISCONNECTED:
          on_disconnect(p_hpbs, p_ble_evt);
          break;

        case BLE_GATTS_EVT_WRITE:
          on_write(p_hpbs, p_ble_evt);
          break;

        default:
          // No implementation needed.
          break;
    }

}

/* Function for adding tx accelerometer characteristic */
static uint32_t rx_char_add(ble_hpbs_t * p_hpbs, const ble_hpbs_init_t * p_hpbs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;
    
    ble_uuid.type = p_hpbs->uuid_type;
    ble_uuid.uuid = HPBS_UUID_CHAR;
    
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
    attr_char_value.init_len  = 1; // maybe change?
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_GATTS_FIX_ATTR_LEN_MAX;

    return sd_ble_gatts_characteristic_add(p_hpbs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_hpbs->rx_handles);

}


uint32_t ble_hpbs_init(ble_hpbs_t * p_hpbs, ble_hpbs_init_t const * p_hpbs_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t hpbs_base_uuid = HPBS_UUID_BASE;

    VERIFY_PARAM_NOT_NULL(p_hpbs);
    VERIFY_PARAM_NOT_NULL(p_hpbs_init);

    // Initialize the service structure
    p_hpbs->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_hpbs->accel_read_handler     = p_hpbs_init->accel_read_handler;

    // Add a custom base UUID
    err_code = sd_ble_uuid_vs_add(&hpbs_base_uuid, &p_hpbs->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_hpbs->uuid_type;
    ble_uuid.uuid = HPBS_UUID_SERVICE;

    // Add the service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_hpbs->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the accelerometer characteristic
    err_code = rx_char_add(p_hpbs, p_hpbs_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


#endif