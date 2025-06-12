/**
  ******************************************************************************
  * @file    BNO055.c
  * @author  Aaron Lubinsky
  * @brief   BNO055 IMU (Inertial Measurement Unit) driver for attitude sensing
  * @version 1.0
  * @date    2025
  *
  * @details This driver provides interface functions for the Bosch BNO055
  *          9-axis absolute orientation sensor. It handles I2C communication,
  *          sensor initialization, calibration monitoring, and Euler angle
  *          data acquisition with integrated flight data logging.
  *
  * @note The BNO055 combines a triaxial 14-bit accelerometer, a triaxial
  *       16-bit gyroscope, a triaxial geomagnetic sensor, and a 32-bit
  *       cortex M0+ microcontroller running Bosch Sensortec sensor fusion software.
  *
  ******************************************************************************
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  1. Ensure I2C1 peripheral is properly configured and initialized
  2. Connect BNO055 reset pin to GPIOB Pin 14
  3. Connect status LED to GPIOA Pin 0 for calibration indication
  4. Call BNO_Init() to initialize and calibrate the IMU
  5. Call BNO_Read() periodically to get current orientation data
  6. Access blackbox[] array for flight data analysis

  @warning Ensure proper I2C pull-up resistors are installed
  @warning Allow sufficient time for IMU calibration before flight
  */

#include "BNO055.h"
#include "stm32f4xx_hal.h"   // Needed for HAL types

/* External I2C Handle */
extern I2C_HandleTypeDef hi2c1; ///< I2C1 handle for BNO055 communication

/* Static Variables */
static uint8_t calibData; ///< Calibration status data from BNO055
extern int32_t roll_set;        ///< Roll setpoint (desired roll angle)
extern int32_t pitch_set;        ///< Roll setpoint (desired roll angle)

/* Flight Data Logging */
IMUSample blackbox[MAX_SAMPLES]; ///< Flight data buffer for post-flight analysis
uint16_t sample_index = 0;       ///< Current index in blackbox buffer
int counter = 0;                 ///< Counter for blackbox data sampling
int pitch_offset = 18000;
int roll_offset = 0;

/**
 * @brief Initializes the BNO055 IMU sensor
 *
 * @details This function performs a complete initialization sequence:
 *          1. Verifies I2C communication with the BNO055
 *          2. Resets the sensor and I2C interface if needed
 *          3. Configures the sensor to NDOF (Nine Degrees of Freedom) mode
 *          4. Waits for full system calibration before returning
 *
 * The function implements robust error handling with automatic retry logic.
 * It toggles the hardware reset pin and reinitializes I2C if communication fails.
 *
 * @note NDOF mode provides absolute orientation by fusing all 9 sensor axes
 * @note Calibration can take 30-60 seconds depending on movement patterns
 * @note Red LED (PA0) remains on during calibration process
 *
 * @warning This function blocks until calibration is complete
 * @warning Ensure sensor is moved through various orientations for proper calibration
 *
 * @see BNO_Read()
 */
void BNO_Init(){
    uint8_t ndof_mode = 0x0C;      ///< NDOF operation mode value
    uint8_t config_mode = 0x00;    ///< Configuration mode value
    uint8_t clear_reg = 0x00;
    uint8_t successfulRead = false; ///< Flag for successful I2C communication
    uint8_t sampleData = 0x00;     ///< Data read from chip ID register
    int calibrated = false;        ///< Calibration completion flag

    /* ===== COMMUNICATION VERIFICATION ===== */
    while(successfulRead == false){
        // Wait to verify IMU connection
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // Reset BNO055 via hardware pin
        HAL_Delay(1000);                       // Wait for BNO055 boot sequence
        HAL_I2C_DeInit(&hi2c1);                // Reset I2C1 peripheral
        HAL_I2C_Init(&hi2c1);                  // Reinitialize I2C1

        if (hi2c1.State == HAL_I2C_STATE_READY) {
            // Once I2C is reset, attempt to read chip ID
            HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR, 0x00, 1, &sampleData, 1, 100);
        }

        // Verify chip ID (should be 0xA0 for BNO055)
        if (sampleData == 0xa0){
            successfulRead = true;
        }
    }

    /* ===== MODE CONFIGURATION ===== */
    // Set to CONFIG mode to allow register writes
    HAL_I2C_Mem_Write(&hi2c1, BNO055_I2C_ADDR, BNO055_OPR_MODE_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &config_mode, 1, 100);
    HAL_Delay(25);

    // Set to NDOF mode for full sensor fusion
    HAL_I2C_Mem_Write(&hi2c1, BNO055_I2C_ADDR, BNO055_OPR_MODE_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &ndof_mode, 1, 100);
    HAL_Delay(25);

    /* ===== CALIBRATION MONITORING ===== */

    for (int i = 0; i < 18; i++)
    {
        HAL_I2C_Mem_Write(&hi2c1,
                          BNO055_I2C_ADDR,
                          BNO055_OFFSET_X + i,
                          I2C_MEMADD_SIZE_8BIT,
                          &clear_reg,
                          1,
                          100);
    }

    while(calibrated == false){
        // Wait for IMU to calibrate
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // Red LED on during calibration

        // Read calibration status register
        HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR, BNO055_CALIB_STAT, 1, &calibData, 1, 100);

        // Check if system calibration is complete (bits 7:6 == 0b11)
        if (((calibData >> 6) & 0x03) == 0x03){
            calibrated = true;
        }

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);   // Red LED off
    }
}

/**
 * @brief Reads current Euler angles from the BNO055 sensor
 *
 * @param[out] roll  Pointer to store roll angle (in millidegrees)
 * @param[out] pitch Pointer to store pitch angle (in millidegrees)
 * @param[out] yaw   Pointer to store yaw angle (in millidegrees)
 *
 * @details This function reads 6 bytes of Euler angle data from the BNO055
 *          and converts them to 32-bit signed integers in millidegrees.
 *          The raw 16-bit values are scaled by 1000/16 to convert from
 *          the BNO055's native 1/16 degree resolution to millidegrees.
 *
 * Additionally, this function implements flight data logging by storing
 * pitch and roll data in the blackbox buffer at a configurable rate.
 *
 * @note Euler angles are returned in millidegrees (1/1000 of a degree)
 * @note Yaw range: 0° to 360° (0 to 360000 millidegrees)
 * @note Roll/Pitch range: -180° to +180° (-180000 to +180000 millidegrees)
 * @note Data logging frequency controlled by blackboxFreq variable
 *
 * @warning Pointers must be valid and point to allocated memory
 * @warning Function should be called at regular intervals for smooth control
 *
 * @see BNO_Init()
 */
void BNO_Read(int32_t *roll, int32_t *pitch, int32_t *yaw){
    uint8_t eulerData[6];  ///< Raw Euler angle data buffer (6 bytes)
    int32_t rawYaw16;      ///< Raw 16-bit yaw value
    int32_t rawPitch16;    ///< Raw 16-bit pitch value
    int32_t rawRoll16;     ///< Raw 16-bit roll value

    /* ===== READ RAW EULER DATA ===== */
    // Read 6 bytes starting from Euler LSB register
    HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR, BNO055_EULER_LSB,
                     I2C_MEMADD_SIZE_8BIT, eulerData, 6, 100);

    /* ===== DATA CONVERSION ===== */
    // Combine LSB and MSB bytes to form 16-bit signed values
    rawYaw16   = (int16_t)((eulerData[1] << 8) | eulerData[0]);  // Bytes 0-1: Yaw
    rawRoll16  = (int16_t)((eulerData[3] << 8) | eulerData[2]);  // Bytes 2-3: Roll
    rawPitch16 = (int16_t)((eulerData[5] << 8) | eulerData[4]);  // Bytes 4-5: Pitch

    // Convert from 1/16 degree resolution to millidegrees
    *yaw   = ((int32_t)rawYaw16 * 1000) / 16;
    *roll  = (((int32_t)rawRoll16 * 1000) / 16) - roll_offset;
    *pitch = (((int32_t)rawPitch16 * 1000) / 16) - pitch_offset;

    /* ===== FLIGHT DATA LOGGING ===== */
    // Log data to blackbox at specified frequency
    if (counter++ == blackboxFreq) {
        if (sample_index < MAX_SAMPLES) {
            blackbox[sample_index].pitch = *pitch;
            blackbox[sample_index].roll  = *roll;
            blackbox[sample_index].pitchSet = pitch_set;
            blackbox[sample_index].rollSet  = roll_set;
            sample_index++;
        }
        counter = 0;
    }
}
