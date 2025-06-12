/**
  ******************************************************************************
  * @file    HC05.c
  * @author  Aaron Lubinsky
  * @brief   HC-05 Bluetooth module driver for wireless communication and control
  * @version 1.0
  * @date    2025
  *
  * @details This driver provides Bluetooth communication interface using the HC-05
  *          module for remote control of the quadcopter. It handles joystick input
  *          parsing, flight parameter adjustment, and flight data transmission.
  *
  *          The driver expects comma-separated values in the format:
  *          #LjoyX,LjoyY,RjoyX,LT,RT,ENTER
  *
  *          Where:
  *          - LjoyX/LjoyY: Left joystick for roll/pitch control
  *          - RjoyX: Right joystick X for yaw control
  *          - LT/RT: Left/Right triggers for throttle control
  *          - ENTER: Button for blackbox data dump
  *
  ******************************************************************************
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  1. Ensure UART2 is configured for HC-05 communication (typically 9600 baud)
  2. Pair HC-05 module with control device (phone/computer)
  4. Call processInput() with received data to parse control commands
  5. Call dumpBlackbox() to transmit flight data for analysis

  @note Input format must start with '#' character for validation
  @note All control values are scaled appropriately for flight control
  @warning Ensure proper UART configuration before use
  @warning Invalid input formats will increment error counter
  */

#include "HC05.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "stm32f4xx_hal.h" // Needed for HAL types
#include "BNO055.h"

/* Communication Statistics */
int badBTcount = 0;  ///< Counter for invalid Bluetooth transmissions
int effortRate = 10; ///< Rate of effort change per control input

/* External UART Handle */
extern UART_HandleTypeDef huart2; ///< UART2 handle for HC-05 communication

/* External PID Constants */
extern int32_t Kp_roll;   ///< Proportional gain for roll control
extern int32_t Ki_roll;   ///< Integral gain for roll control
extern int32_t Kd_roll;   ///< Derivative gain for roll control
extern int32_t Kp_pitch;  ///< Proportional gain for pitch control
extern int32_t Ki_pitch;  ///< Integral gain for pitch control
extern int32_t Kd_pitch;  ///< Derivative gain for pitch control
extern int32_t Kp_yaw;    ///< Proportional gain for yaw control
extern int32_t Ki_yaw;    ///< Integral gain for yaw control
extern int32_t Kd_yaw;    ///< Derivative gain for yaw control

/* External Sensor Data */
extern int32_t pitch_true; ///< Current pitch angle from IMU (millidegrees)
extern int32_t roll_true;  ///< Current roll angle from IMU (millidegrees)
extern int32_t yaw_true;   ///< Current yaw angle from IMU (millidegrees)

extern int stopFlag;

/**
 * @brief Processes incoming control input from Bluetooth connection
 *
 * @param[in]  charBuf   Input string buffer containing control data
 * @param[out] roll      Pointer to store calculated roll setpoint (millidegrees)
 * @param[out] pitch     Pointer to store calculated pitch setpoint (millidegrees)
 * @param[out] yaw       Pointer to store calculated yaw setpoint (millidegrees)
 * @param[out] effort    Pointer to store/update throttle effort value
 * @param[out] dumpFlag  Pointer to store blackbox dump request flag
 *
 * @details Parses comma-separated joystick and button data from the control device.
 *          Expected input format: "#LjoyX,LjoyY,RjoyX,LT,RT,ENTER"
 *
 *          Control mapping:
 *          - Left joystick X/Y → Roll/Pitch commands (±20° range)
 *          - Right joystick X → Yaw rate command (relative to current heading)
 *          - Left/Right triggers → Throttle increase/decrease
 *          - Enter button → Trigger blackbox data dump
 *
 * @note Input must start with '#' character for validation
 * @note Roll/Pitch scaled from joystick ±1000 range to ±20° (±20000 millidegrees)
 * @note Yaw command is relative to current heading with wraparound
 * @note Effort is constrained between 0 and 1000, 0 is transformed to large negative number, ensuring no spin due to PID loop
 * @note Invalid inputs increment badBTcount and are rejected
 *
 * @warning Pointers must be valid and point to allocated memory
 * @warning Function modifies input buffer (uses strtok)
 *
 * @see InitializeBT()
 * @see dumpBlackbox()
 */
void processInput(char *charBuf, int32_t *roll, int32_t *pitch, int32_t *yaw, int32_t *effort, int *dumpFlag){
    int32_t LjoyX, LjoyY, RjoyX, LT, RT, ENTER; ///< Parsed joystick and button values

    /* ===== INPUT VALIDATION ===== */
    // Skip pound sign if present
    if (charBuf[0] == '#') {
        charBuf++; // Move pointer to the next character
    } else {
        badBTcount++;
        printf("Bad BT input (did not process): %d\r\n", badBTcount);
        return;
    }

    /* ===== DATA PARSING ===== */
    // Parse comma-separated values using strtok
    char *token = strtok(charBuf, ",");
    if (token) LjoyX = (int32_t)strtol(token, NULL, 10);

    token = strtok(NULL, ",");
    if (token) LjoyY = (int32_t)strtol(token, NULL, 10);

    token = strtok(NULL, ",");
    if (token) RjoyX = (int32_t)strtol(token, NULL, 10);

    token = strtok(NULL, ",");
    if (token) LT = (int32_t)strtol(token, NULL, 10);

    token = strtok(NULL, ",");
    if (token) RT = (int32_t)strtol(token, NULL, 10);

    token = strtok(NULL, ",");
    if (token) ENTER = (int32_t)strtol(token, NULL, 10);

    /* ===== CONTROL MAPPING ===== */
    // Convert joystick values to flight control commands

    // Roll/Pitch: Scale joystick input to ±20° (assuming joystick range ±1000)
    *roll = LjoyX * 180 / 9;   // Scale to millidegrees (±20000)
    *pitch = LjoyY * 180 / 9;  // Scale to millidegrees (±20000)

    // Yaw: Relative control - add rate command to current heading
    *yaw = yaw_true + (RjoyX) / 10;

    // Handle yaw wraparound (0-360°)
    if (*yaw < 0) {
        *yaw = *yaw + 360000;
    } else if (*yaw > 360000) {
        *yaw = *yaw - 360000;
    }

    // Throttle: Differential trigger control (RT increases, LT decreases)
    *effort = *effort + (RT - LT) * effortRate / 1000;

    // Constrain effort to safe limits
    stopFlag = false;
    if (*effort <= 0) {
        *effort = 0;
        stopFlag = true;
    } else if (*effort > 1000) {
        *effort = 1000;
    }

    // Blackbox dump trigger
    if (ENTER == 1) {
        *dumpFlag = 1;
    }
}

/**
 * @brief Transmits flight data blackbox over Bluetooth connection
 *
 * @details Sends all recorded flight data from the blackbox buffer via UART
 *          to the connected Bluetooth device. Data is transmitted in CSV format
 *          with one sample per line containing pitch, pitch setpoint, roll, and
 *          roll setpoint values.
 *
 *          Output format per line: "pitch,pitchSet,roll,rollSet\r\n"
 *          All values are in millidegrees.
 *
 * @note Function transmits all samples up to current sample_index
 * @note 200ms delay added to allow receiver to process data
 * @note Data transmission is blocking (waits for completion)
 * @note Currently only transmits pitch and roll data (yaw commented out)
 *
 * @warning Large blackbox buffers may take significant time to transmit
 * @warning Ensure receiver can handle the data rate
 * @warning Function blocks until all data is transmitted
 *
 * @see processInput()
 * @see InitializeBT()
 */
void dumpBlackbox(void)
{
    // Optional: Send PID parameters first (currently commented)
    //char paramMsg[128];
    //snprintf(paramMsg, sizeof(paramMsg), "Kp_pitch: %ld, Ki_pitch: %ld, Kd_pitch: %ld, Kp_roll: %ld, Ki_roll: %ld, Kd_roll: %ld \r\n",Kp_pitch, Ki_pitch, Kd_pitch, Kp_roll, Ki_roll, Kd_roll);
    //HAL_UART_Transmit(&huart2, (uint8_t*)paramMsg, strlen(paramMsg), HAL_MAX_DELAY);


    char msg[64]; ///< Message buffer for each data line

    // Transmit all blackbox samples
    for (uint16_t i = 0; i < sample_index; i++) {
        snprintf(msg, sizeof(msg), "%ld,%ld,%ld,%ld\r\n",
                blackbox[i].pitch,
                blackbox[i].pitchSet,
                blackbox[i].roll,
                blackbox[i].rollSet);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY/200);
    }
}


void sendATCommand(const char* cmd) {
HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
HAL_Delay(500); // Small delay to give HC-05 time to process
}


void configure_HC05() {
// Optional: Reset to defaults
// sendATCommand("AT+ORGL\r\n");

sendATCommand("AT+NAME=MECHA16 ROBOT BT\r\n"); // Module Name
//sendATCommand("AT+PSWD=0614\r\n");
sendATCommand("AT+UART=115200,0,0\r\n"); // Set Baud Rate
sendATCommand("AT+RESET\r\n"); // Apply Changes
}
/* USER CODE END 0 */
