/**
 * @file BNO055.h
 * @brief Interface to the Adafruit BNO055 IMU for orientation sensing.
 *
 * This file contains initialization and reading functions for
 * the BNO055 inertial measurement unit (IMU), as well as constants
 * and data structures for storing and analyzing flight data.
 *
 * @author Aaron
 * @date May 17, 2025
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "main.h" ///< Include for STM32 HAL types and definitions

/**
 * @brief Initializes the BNO055 sensor in NDOF mode.
 */
void BNO_Init(void);

/**
 * @brief Reads Euler angles from the BNO055 sensor.
 *
 * @param roll Pointer to variable storing the roll angle.
 * @param pitch Pointer to variable storing the pitch angle.
 * @param yaw Pointer to variable storing the yaw angle.
 */
void BNO_Read(int32_t *roll, int32_t *pitch, int32_t *yaw);

#define BNO055_I2C_ADDR       (0x28 << 1) ///< 7-bit I2C address shifted for STM32 HAL
#define BNO055_OPR_MODE_ADDR  0x3D        ///< Operation mode register
#define BNO055_EULER_LSB      0x1A        ///< Start of Euler angle registers
#define BNO055_CALIB_STAT     0x35        ///< Calibration status register
#define BNO055_OFFSET_X       0x55        //offset x LSB
#define MAX_SAMPLES           5000        ///< Maximum samples for blackbox logging
#define blackboxFreq          2           ///< Logging frequency (Hz)
#define true 1                           ///< Boolean true
#define false 0                          ///< Boolean false

/**
 * @brief Buffer storing raw IMU data samples.
 */
extern uint8_t ndof_buf;

/**
 * @struct IMUSample
 * @brief Structure holding IMU roll/pitch data and their setpoints.
 */
typedef struct {
    int32_t pitch;     ///< Measured pitch
    int32_t roll;      ///< Measured roll
    int32_t pitchSet;  ///< Desired pitch
    int32_t rollSet;   ///< Desired roll
} IMUSample;

extern IMUSample blackbox[MAX_SAMPLES]; ///< Flight data buffer
extern uint16_t sample_index;           ///< Index for blackbox samples
extern int counter;                     ///< Sample counter or general use variable

#endif /* INC_BNO055_H_ */
