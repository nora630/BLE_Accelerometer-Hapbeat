#ifndef LIS2DH_H
#define LIS2DH_H

#include "nrf_twi_sensor.h"
#include "lis2dh_internal.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Possible sensor addresses.
 */
#define LIS2DH_BASE_ADDRESS_LOW       0x18U
#define LIS2DH_BASE_ADDRESS_HIGH      0x19U

// WHO_AM_I register value.
#define LIS2DH_WHO_AM_I               0x33


/**
 * @brief Output data rate settings.
 */
typedef enum
{
    LIS2DH_ODR_POWERDOWN,
    LIS2DH_ODR_1HZ,
    LIS2DH_ODR_10HZ,
    LIS2DH_ODR_25HZ,
    LIS2DH_ODR_50HZ,
    LIS2DH_ODR_100HZ,
    LIS2DH_ODR_200HZ,
    LIS2DH_ODR_400HZ,
    LIS2DH_ODR_1620HZ,
    LIS2DH_ODR_1344_5376HZ
} lis2dh_odr_t;

/**
 * @brief Fifo mode settings.
 */
typedef enum
{
    LIS2DH_BYPASS,
    LIS2DH_FIFO,
    LIS2DH_STREAM,
    LIS2DH_STREAM_TO_FIFO
} lis2dh_fifo_mode_t;


/**
 * @brief Structure containing accelerometer data.
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} lis2dh_data_t;

/**
 * @brief Data callback prototype.
 *
 * @param[in] result Result of operation (NRF_SUCCESS on success,
 *                   otherwise a relevant error code).
 * @param[in] p_data Pointer to raw sensor data structure.
 */
typedef void (* lis2dh_data_cb_t)(ret_code_t result, lis2dh_data_t * p_data);

/**
 * @brief Macro for defining sensor instance.
 *
 * @param[in] _lis2dh_inst_name   Sensor instance name.
 * @param[in] _p_twi_sensor         Pointer to common TWI sensor instance.
 * @param[in] _sensor_address       Sensor base address.
 */
#define LIS2DH_INSTANCE_DEF(_lis2dh_inst_name, _p_twi_sensor, _sensor_address) \
    LIS2DH_INTERNAL_INSTANCE_DEF(_lis2dh_inst_name, _p_twi_sensor, _sensor_address)

/**
 * @brief Macro for setting data acquisition configuration.
 *
 * @param[in] _s        Sensor instance.
 * @param[in] _odr      Data rate. @ref LIS2DH_odr_t
 * @param[in] _lp       Power mode. True if low power mode is enabled.
 * @param[in] _z_en     Enable measure in z-axis. True if enabled.
 * @param[in] _y_en     Enable measure in y-axis. True if enabled.
 * @param[in] _x_en     Enable measure in x-axis. True if enabled.
 */
#define LIS2DH_DATA_CFG(_s, _odr, _lp, _z_en, _y_en, _x_en) \
    LIS2DH_INTERNAL_DATA_CFG(_s, _odr, _lp, _z_en, _y_en, _x_en)


/**
 * @brief Macro for setting FIFO configuration.
 *
 * @param[in] _s     Sensor instance.
 * @param[in] _en    Enables FIFO. True if enabled. False clears FIFO setting.
 * @param[in] _mode  FIFO mode. @ref LIS2DH_fifo_mode_t
 * @param[in] _t_sel Trigger event pin selection. True if int2 pin, false if int1 pin.
 * @param[in] _t_thr Trigger threshold.
 */
#define LIS2DH_FIFO_CFG(_s, _en, _mode, _t_sel, _t_thr) \
    LIS2DH_INTERNAL_FIFO_CFG(_s, _en, _mode, _t_sel, _t_thr)

/**
 * @brief Function for initializing LIS2DH instance.
 *
 * @param[in] p_inst Pointer to sensor instance defined by macro. @ref lis2dh_insTANCE_DEF
 *
 * @return  Return error code from nrf_twi_sensor @ref nrf_twi_sensor_write
 */
ret_code_t lis2dh_init(lis2dh_instance_t * p_inst);

/**
 * @brief Function for writing configuration to sensor.
 *
 * @param[in] p_inst Pointer to sensor instance.
 *
 * @return  Return error code from nrf_twi_sensor @ref nrf_twi_sensor_write
 */
ret_code_t lis2dh_cfg_commit(lis2dh_instance_t * p_inst);

/**
 * @brief Function for reading accelerometer data.
 *
 * @param[in] p_inst  Pointer to sensor instance.
 * @param[in] user_cb Function to be called after data read is complete.
 * @param[in] p_data  Pointer to data structure.
 * @param[in] samples Number of samples to read.
 *
 * @note When trying to read more than one sample and FIFO is disabled,
 *       current output value will be copied to all read samples.
 *       When trying to read more samples than there is currently in FIFO,
 *       excess samples will be equal to 0.
 *
 * @return  Return error code from nrf_twi_sensor @ref nrf_twi_sensor_reg_read
 */
ret_code_t lis2dh_data_read(lis2dh_instance_t * p_inst,
                              lis2dh_data_cb_t    user_cb,
                              lis2dh_data_t *     p_data,
                              uint8_t               samples);


#ifdef __cplusplus
}
#endif

#endif // LIS2DH_H
