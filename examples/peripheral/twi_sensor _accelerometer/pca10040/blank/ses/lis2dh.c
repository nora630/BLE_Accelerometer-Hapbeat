#include "lis2dh.h"

#define RETURN_IF_ERR(err)  \
    if (err != NRF_SUCCESS) \
    {                       \
        return err;         \
    }

ret_code_t lis2dh_init(lis2dh_instance_t * p_inst)
{
    ASSERT(p_inst != NULL);
    p_inst->ctrl1     = 0x8F; // Low Power and ODR = 1620Hz
    p_inst->ctrl5     = 0xC0; // FIFO enable
    p_inst->fifo_ctrl = 0x80; // Stream mode
    return lis2dh_cfg_commit(p_inst);
}

ret_code_t lis2dh_cfg_commit(lis2dh_instance_t * p_inst)
{
    ASSERT(p_inst != NULL);
    ret_code_t err;
   

    uint8_t ctrl1_msg[] = {
        LIS2DH_REG_CTRL_REG1,
        p_inst->ctrl1
    };
    err = nrf_twi_sensor_write(p_inst->p_sensor_data,
                               p_inst->sensor_addr,
                               ctrl1_msg,
                               ARRAY_SIZE(ctrl1_msg),
                               true);
    RETURN_IF_ERR(err);

    uint8_t ctrl5_msg[] = {
        LIS2DH_REG_CTRL_REG5,
        p_inst->ctrl5
    };
    err = nrf_twi_sensor_write(p_inst->p_sensor_data,
                               p_inst->sensor_addr,
                               ctrl5_msg,
                               ARRAY_SIZE(ctrl5_msg),
                               true);
    RETURN_IF_ERR(err);


    uint8_t fifo_msg[] = {
        LIS2DH_REG_FIFO_CTRL,
        p_inst->fifo_ctrl
    };

    err = nrf_twi_sensor_write(p_inst->p_sensor_data,
                               p_inst->sensor_addr,
                               fifo_msg,
                               ARRAY_SIZE(fifo_msg),
                               true);

    return err;
}

ret_code_t lis2dh_data_read(lis2dh_instance_t * p_inst,
                              lis2dh_data_cb_t    user_cb,
                              lis2dh_data_t *     p_data,
                              uint8_t               samples)
{
    ASSERT(p_inst != NULL);
    return nrf_twi_sensor_reg_read(p_inst->p_sensor_data,
                                   p_inst->sensor_addr,
                                   LIS2DH_REG_OUT_X_L | LIS2DH_AUTO_INCR_MASK,
                                   (nrf_twi_sensor_reg_cb_t) user_cb,
                                   (uint8_t *) p_data,
                                   samples * LIS2DH_BYTES_PER_SAMPLE);
}

