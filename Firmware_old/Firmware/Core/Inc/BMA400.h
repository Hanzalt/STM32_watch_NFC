/*
 * BMA400.h
 *
 *  Created on: Jan 9, 2026
 *      Author: vakhaib
 */

#ifndef INC_BMA400_H_
#define INC_BMA400_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief BMA400 orientation-change (orientch) driver context.
 */
typedef struct
{
    I2C_HandleTypeDef *hi2c;   /**< STM32 HAL I2C handle */
    uint8_t i2c_addr_7bit;     /**< 7-bit I2C address (usually 0x14 or 0x15) */
} BMA400_Orient_t;

/**
 * @brief Axis mask selection for orientch feature.
 */
typedef enum
{
    BMA400_AXIS_X = (1u << 0),
    BMA400_AXIS_Y = (1u << 1),
    BMA400_AXIS_Z = (1u << 2),
} BMA400_AxisMask_t;

/**
 * @brief Init struct for orientation-change interrupt feature.
 */
typedef struct
{
    uint8_t threshold_lsb;     /**< 8 mg/LSB (e.g. 32 => 256 mg) */
    uint8_t duration_lsb;      /**< 10 ms/LSB (e.g. 5 => 50 ms) */
    bool use_lowpass_1hz;      /**< false => use acc_filt2 (fast), true => acc_filt_lp (slower, 1Hz LPF) */
    bool int1_active_high;     /**< true => INT1 active-high, false => active-low */
    bool int1_push_pull;       /**< true => push-pull, false => open-drain */
    bool non_latched;          /**< true => pulse/non-latched, false => latched */

    uint8_t axis_mask;         /**< OR of BMA400_AXIS_X/Y/Z. Example: BMA400_AXIS_Z */
} BMA400_OrientCfg_t;

/**
 * @brief Which axis & direction defines "face-to-me".
 */
typedef enum
{
    BMA400_FACE_AXIS_X_POS,
    BMA400_FACE_AXIS_X_NEG,
    BMA400_FACE_AXIS_Y_POS,
    BMA400_FACE_AXIS_Y_NEG,
    BMA400_FACE_AXIS_Z_POS,
    BMA400_FACE_AXIS_Z_NEG,
} BMA400_FaceAxis_t;

/**
 * @brief Initialize the driver context.
 * @param dev Pointer to device context
 * @param hi2c STM32 I2C handle
 * @param i2c_addr_7bit 7-bit address (0x14 or 0x15)
 */
void BMA400_Orient_Init(BMA400_Orient_t *dev, I2C_HandleTypeDef *hi2c, uint8_t i2c_addr_7bit);

/**
 * @brief Configure BMA400 to generate "orientation change" interrupts on INT1.
 * @param dev Pointer to device context
 * @param cfg Configuration parameters (threshold/duration/int pin behavior)
 * @return HAL_OK on success, otherwise HAL_ERROR/HAL_TIMEOUT/etc.
 */
HAL_StatusTypeDef BMA400_Orient_EnableInt1(BMA400_Orient_t *dev, const BMA400_OrientCfg_t *cfg);

/**
 * @brief Read INT status and report if an orientation-change interrupt occurred.
 *        Calling this will also "acknowledge" the status by reading the status register.
 * @param dev Pointer to device context
 * @param out_orientch true if orientation-change status bit is set
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMA400_Orient_ReadIntStatus(BMA400_Orient_t *dev, bool *out_orientch);

/**
 * @brief Re-arm/refresh the reference used by the orientation-change logic.
 *        Useful after you handle an interrupt to make the new position the baseline.
 * @param dev Pointer to device context
 * @param use_lowpass_1hz must match the mode you configured
 * @return HAL_OK on success
 */
HAL_StatusTypeDef BMA400_Orient_RearmReference(BMA400_Orient_t *dev, bool use_lowpass_1hz);

/**
 * @brief Read raw 12-bit signed acceleration samples (X/Y/Z).
 *        Returned units are "raw codes". Scale depends on range. In ±2g mode,
 *        typical is ~1024 LSB/g.
 */
HAL_StatusTypeDef BMA400_ReadXYZ_Raw12(BMA400_Orient_t *dev, int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief Decide if current pose matches "face-to-me" based on one axis threshold.
 *
 * @param x,y,z raw 12-bit values from BMA400_ReadXYZ_Raw12()
 * @param face_axis which axis/sign is face-to-me
 * @param threshold_raw absolute threshold in raw codes (e.g. 768 ~ 0.75g at 1024 LSB/g)
 *
 * @return true if pose is considered face-to-me
 */
bool BMA400_IsFaceToMe(int16_t x, int16_t y, int16_t z,
                       BMA400_FaceAxis_t face_axis,
                       int16_t threshold_raw);

#ifdef __cplusplus
}
#endif


#endif /* INC_BMA400_H_ */
