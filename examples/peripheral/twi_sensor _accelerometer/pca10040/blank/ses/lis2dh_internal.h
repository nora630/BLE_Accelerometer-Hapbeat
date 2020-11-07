/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
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
#ifndef LIS2DH_INTERNAL_H
#define LIS2DH_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif


#define LIS2DH_AUTO_INCR_MASK         0x80

#define LIS2DH_BYTES_PER_SAMPLE       6

/**
 * @brief LIS2DH sensor registers.
 */

#define LIS2DH_REG_CTRL_REG1          0x20
#define LIS2DH_REG_CTRL_REG2          0x21
#define LIS2DH_REG_CTRL_REG3          0x22
#define LIS2DH_REG_CTRL_REG4          0x23
#define LIS2DH_REG_CTRL_REG5          0x24
#define LIS2DH_REG_CTRL_REG6          0x25

#define LIS2DH_REG_OUT_X_L            0x28
#define LIS2DH_REG_OUT_X_H            0x29
#define LIS2DH_REG_OUT_Y_L            0x2A
#define LIS2DH_REG_OUT_Y_H            0x2B
#define LIS2DH_REG_OUT_Z_L            0x2C
#define LIS2DH_REG_OUT_Z_H            0x2D

#define LIS2DH_REG_FIFO_CTRL          0x2E
#define LIS2DH_REG_FIFO_SRC           0x2F



/**
 * @brief Control register 1 bitmasks
 */

// Bitmasks for ODR.
#define LIS2DH_ODR_POS                4
#define LIS2DH_ODR_MASK               (0x08 << LIS2DH_ODR_POS)

// Bitmasks for LP_EN
#define LIS2DH_LP_EN_POS              3
#define LIS2DH_LP_EN_MASK             (1 << LIS2DH_LP_EN_POS)

// Bitmasks for Z_EN
#define LIS2DH_Z_EN_POS               2
#define LIS2DH_Z_EN_MASK              (1 << LIS2DH_Z_EN_POS)

// Bitmasks for Y_EN
#define LIS2DH_Y_EN_POS               1
#define LIS2DH_Y_EN_MASK              (1 << LIS2DH_Y_EN_POS)

// Bitmasks for X_EN
#define LIS2DH_X_EN_POS               0
#define LIS2DH_X_EN_MASK              (1 << LIS2DH_X_EN_POS)







/**
 * @brief Control register 5 bitmasks.
 */

// Bitmasks for BOOT.
#define LIS2DH_BOOT_POS               7
#define LIS2DH_BOOT_MASK              (1 << LIS2DH_BOOT_POS)

// Bitmasks for FIFO_EN.
#define LIS2DH_FIFO_EN_POS            6
#define LIS2DH_FIFO_EN_MASK           (1 << LIS2DH_FIFO_EN_POS)



/**
 * @brief FIFO control register bitmasks.
 */

// Bitmasks for FM.
#define LIS2DH_FM_POS                 6
#define LIS2DH_FM_MASK                (2 << LIS2DH_FM_POS)

// Bitmasks for TR.
#define LIS2DH_TR_POS                 5
#define LIS2DH_TR_MASK                (0 << LIS2DH_TR_POS)

// Bitmasks for FTH.
#define LIS2DH_FTH_POS                0
#define LIS2DH_FTH_MASK               (0x00 << LIS2DH_FTH_POS)


/**
 * @brief Structure holding sensor instance
 */
typedef struct
{
    nrf_twi_sensor_t * const p_sensor_data;
    uint8_t const            sensor_addr;

    uint8_t ctrl1;
    uint8_t ctrl2;
    uint8_t ctrl3;
    uint8_t ctrl4;
    uint8_t ctrl5;
    uint8_t ctrl6;
    uint8_t fifo_ctrl;

} lis2dh_instance_t;

/**
 * @brief Macro for defining sensor instance.
 */
#define LIS2DH_INTERNAL_INSTANCE_DEF(_lis2dh_inst_name, _p_twi_sensor, _sensor_address)       \
    static lis2dh_instance_t _lis2dh_inst_name =                                              \
    {                                                                                             \
        .p_sensor_data = _p_twi_sensor,                                                           \
        .sensor_addr   = _sensor_address                                                          \
    }


/**
 * @brief Macro for setting data acquisition configuration.
 */
#define LIS2DH_INTERNAL_DATA_CFG(_s, _odr, _lp, _z_en, _y_en, _x_en)                   \
    NRF_TWI_SENSOR_REG_SET(_s.ctrl1, LIS2DH_ODR_MASK,   LIS2DH_ODR_POS,   _odr);               \
    NRF_TWI_SENSOR_REG_SET(_s.ctrl1, LIS2DH_LP_EN_MASK, LIS2DH_LP_EN_POS, _lp);                \
    NRF_TWI_SENSOR_REG_SET(_s.ctrl1, LIS2DH_Z_EN_MASK,  LIS2DH_Z_EN_POS,  _z_en);              \
    NRF_TWI_SENSOR_REG_SET(_s.ctrl1, LIS2DH_Y_EN_MASK,  LIS2DH_Y_EN_POS,  _y_en);              \
    NRF_TWI_SENSOR_REG_SET(_s.ctrl1, LIS2DH_X_EN_MASK,  LIS2DH_X_EN_POS,  _x_en);


/**
 * @brief Macro for setting FIFO configuration.
 */
#define LIS2DH_INTERNAL_FIFO_CFG(_s, _en, _mode, _t_sel, _t_thr)                                          \
    NRF_TWI_SENSOR_REG_SET(_s.fifo_ctrl, LIS2DH_FM_MASK,      LIS2DH_FM_POS,      _mode);      \
    NRF_TWI_SENSOR_REG_SET(_s.fifo_ctrl, LIS2DH_TR_MASK,      LIS2DH_TR_POS,      _t_sel);     \
    NRF_TWI_SENSOR_REG_SET(_s.fifo_ctrl, LIS2DH_FTH_MASK,     LIS2DH_FTH_POS,     _t_thr);     \
    NRF_TWI_SENSOR_REG_SET(_s.ctrl5,     LIS2DH_FIFO_EN_MASK, LIS2DH_FIFO_EN_POS, _en)


#ifdef __cplusplus
}
#endif

#endif // LIS2DH_INTERNAL_H
