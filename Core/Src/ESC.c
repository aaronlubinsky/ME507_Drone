/**
  ******************************************************************************
  * @file    ESC.c
  * @author  Aaron Lubinsky
  * @brief   ESC (Electronic Speed Controller) driver for quadcopter motor control
  * @version 1.0
  * @date    2025
  *
  * @details This driver implements PID-based motor control for a quadcopter ESC system.
  *          It provides functionality to arm ESCs and update motor speeds based on
  *          roll, pitch, and yaw control inputs using PID control algorithms.
  *
  ******************************************************************************
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  1. Call armESC() to initialize and arm all ESC motors
  2. Call update_Motors() periodically to update motor speeds based on PID control
  3. Ensure all external variables (PID constants, setpoints, etc.) are properly configured

  @note This driver requires STM32 HAL library and Timer 3 configured for PWM output
  @warning Motor safety: Always ensure proper calibration of motor offsets before flight
  */

#include "ESC.h"
#include "stm32f4xx_hal.h"   // Needed for HAL types
#include <stdint.h>
#include <stdio.h>

/* External Timer Handle */
extern TIM_HandleTypeDef htim3; ///< Timer handle for PWM generation (Timer 3)

/* External Control Variables */
extern int effort_set;    ///< Base throttle effort setting (0-1000)
extern int K_effort;      ///< Effort scaling constant
extern int effortRate;    ///< Rate of effort change
extern int stopFlag;

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

/* External PID State Variables */
extern int32_t roll_set;        ///< Roll setpoint (desired roll angle)
extern int32_t roll_true;       ///< Current roll angle (from sensors)
extern int32_t roll_error;      ///< Roll error (setpoint - actual)
extern int32_t roll_integral;   ///< Roll integral term accumulator
extern int32_t roll_derivative; ///< Roll derivative term
extern int32_t last_roll_error; ///< Previous roll error for derivative calculation

extern int32_t pitch_set;        ///< Pitch setpoint (desired pitch angle)
extern int32_t pitch_true;       ///< Current pitch angle (from sensors)
extern int32_t pitch_error;      ///< Pitch error (setpoint - actual)
extern int32_t pitch_integral;   ///< Pitch integral term accumulator
extern int32_t pitch_derivative; ///< Pitch derivative term
extern int32_t last_pitch_error; ///< Previous pitch error for derivative calculation

extern int32_t yaw_set;        ///< Yaw setpoint (desired yaw angle)
extern int32_t yaw_true;       ///< Current yaw angle (from sensors)
extern int32_t yaw_error;      ///< Yaw error (setpoint - actual)
extern int32_t yaw_integral;   ///< Yaw integral term accumulator
extern int32_t yaw_derivative; ///< Yaw derivative term
extern int32_t last_yaw_error; ///< Previous yaw error for derivative calculation

extern int32_t roll_effort;  ///< Roll control output effort
extern int32_t pitch_effort; ///< Pitch control output effort
extern int32_t yaw_effort;   ///< Yaw control output effort

/**
 * @brief PID scaling factor for fixed-point arithmetic
 * @details Used to maintain precision in PID calculations while using integer math
 */
#define PID_SCALE 100000

/**
 * @brief Motor offset calibration values
 * @details These offsets compensate for individual motor/ESC variations
 * @{
 */
int motA_offset = 960; ///< Motor A PWM offset (front-right motor)
int motB_offset = 960; ///< Motor B PWM offset (rear-right motor)
int motC_offset = 960; ///< Motor C PWM offset (rear-left motor)
int motD_offset = 960; ///< Motor D PWM offset (front-left motor)
/** @} */

/**
 * @brief Motor PWM compare values
 * @details Individual motor PWM compare values sent to timer
 * @{
 */
int A = 0; ///< Motor A PWM compare value
int B = 0; ///< Motor B PWM compare value
int C = 0; ///< Motor C PWM compare value
int D = 0; ///< Motor D PWM compare value
/** @} */

int armCompare = 0;     ///< PWM compare value used during ESC arming sequence
int max_integral = 400000; ///< Maximum integral windup limit


// Add these global variables for rolling average derivative calculation

#define DERIVATIVE_SAMPLES 20

// Rolling average arrays for each axis (no index needed)
int roll_delta_history[DERIVATIVE_SAMPLES] = {0};
int pitch_delta_history[DERIVATIVE_SAMPLES] = {0};
int yaw_delta_history[DERIVATIVE_SAMPLES] = {0};

/**
 * @brief Arms all ESC motors by sending initialization sequence
 *
 * @details This function sends the proper PWM signals to arm all four ESC motors.
 *          It starts PWM generation on all channels and sends a specific pulse width
 *          (approximately 1000μs) to arm the ESCs. The function continues until
 *          the roll_set value reaches a threshold, indicating the system is ready.
 *
 * @note This function blocks execution until arming is complete
 * @warning Ensure motors are properly secured before calling this function
 *
 * @see update_Motors()
 */
void armESC()
{
    effort_set = 1000;

    while(roll_set < 10000){
        // Start PWM generation on all timer channels
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

        // Calculate arming PWM value (approximately 1000μs pulse width)
        armCompare = effort_set*4 - 2000;

        // Clamp arming value to safe range
        if (armCompare < 960) armCompare = 960;
        if (armCompare > 2000) armCompare = 2000;

        // Set PWM compare values for all motors
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, armCompare);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, armCompare);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, armCompare);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, armCompare);

        // Toggle status LED during arming
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
        HAL_Delay(125);
    }

    // Reset effort and compare values after arming
    effort_set = 0;
    armCompare = 0;
}

/**
 * @brief Updates all motor speeds based on PID control calculations
 *
 * @details This function performs the main control loop operation:
 *          1. Calculates PID errors for roll, pitch, and yaw axes
 *          2. Computes PID control efforts for each axis
 *          3. Mixes control efforts to determine individual motor speeds
 *          4. Applies safety limits and updates PWM outputs
 *
 * The motor mixing follows a standard quadcopter X-configuration:
 * - Motor A (front-right): +pitch, -roll, -yaw
 * - Motor B (rear-right): -pitch, -roll, +yaw
 * - Motor C (rear-left): -pitch, +roll, -yaw
 * - Motor D (front-left): +pitch, +roll, +yaw
 *
 * @note PWM range: 960 (0% throttle) to 2000 (100% throttle)
 * @note For safety, maximum throttle is limited to 1500 (≈50%)
 * @note Function should be called at regular intervals (typically 1kHz)
 *
 * @warning
 *
 * @see armESC()
 */

void update_Motors()
{
    // PWM Mapping: Compare 960 = 1ms (0%), Compare 2000 = 2ms (100%)

    /* ===== ROLL PID CALCULATION ===== */
    roll_error = -roll_set + roll_true;

    roll_integral += roll_error/1000; //Wind up protection for integral
    if (roll_integral > max_integral) {
        roll_integral = max_integral;
    } else if (roll_integral < -max_integral) {
        roll_integral = -max_integral;
    }


    // Shift array left and add new delta at end
    float roll_delta = roll_error - last_roll_error;
    for (int i = 0; i < DERIVATIVE_SAMPLES - 1; i++) {
        roll_delta_history[i] = roll_delta_history[i + 1];
    }
    roll_delta_history[DERIVATIVE_SAMPLES - 1] = roll_delta;

    // Calculate rolling average derivative
    float roll_derivative_sum = 0;
    for (int i = 0; i < DERIVATIVE_SAMPLES; i++) {
        roll_derivative_sum += roll_delta_history[i];
    }
    roll_derivative = roll_derivative_sum / DERIVATIVE_SAMPLES;

    // TESTING ONLY - Roll effort disabled
    roll_effort = -(Kp_roll * roll_error + Ki_roll * roll_integral + Kd_roll * roll_derivative) / PID_SCALE;
    last_roll_error = roll_error;

    /* ===== PITCH PID CALCULATION ===== */
    pitch_error = -pitch_set + pitch_true;
    pitch_integral += pitch_error/1000; //Wind up protection for integral
    if (pitch_integral > max_integral) {
        pitch_integral = max_integral;
    } else if (pitch_integral < -max_integral) {
        pitch_integral = -max_integral;
    }

    // Shift array left and add new delta at end
    float pitch_delta = pitch_error - last_pitch_error;
    for (int i = 0; i < DERIVATIVE_SAMPLES - 1; i++) {
        pitch_delta_history[i] = pitch_delta_history[i + 1];
    }
    pitch_delta_history[DERIVATIVE_SAMPLES - 1] = pitch_delta;

    // Calculate rolling average derivative
    float pitch_derivative_sum = 0;
    for (int i = 0; i < DERIVATIVE_SAMPLES; i++) {
        pitch_derivative_sum += pitch_delta_history[i];
    }
    pitch_derivative = pitch_derivative_sum / DERIVATIVE_SAMPLES;

    pitch_effort = -(Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_derivative) / PID_SCALE;
    last_pitch_error = pitch_error;

    /* ===== YAW PID CALCULATION ===== */
    yaw_error = -yaw_set + yaw_true;
    yaw_integral += yaw_error;

    // Shift array left and add new delta at end
    float yaw_delta = yaw_error - last_yaw_error;
    for (int i = 0; i < DERIVATIVE_SAMPLES - 1; i++) {
        yaw_delta_history[i] = yaw_delta_history[i + 1];
    }
    yaw_delta_history[DERIVATIVE_SAMPLES - 1] = yaw_delta;

    // Calculate rolling average derivative
    float yaw_derivative_sum = 0;
    for (int i = 0; i < DERIVATIVE_SAMPLES; i++) {
        yaw_derivative_sum += yaw_delta_history[i];
    }
    yaw_derivative = yaw_derivative_sum / DERIVATIVE_SAMPLES;

    yaw_effort = -(Kp_yaw * yaw_error + Ki_yaw * yaw_integral + Kd_yaw * yaw_derivative) / PID_SCALE;
    last_yaw_error = yaw_error;

    /* ===== BASE THROTTLE CALCULATION ===== */
    // Start with base throttle plus individual motor offsets
    A = effort_set * K_effort/PID_SCALE + motA_offset;
    B = effort_set * K_effort/PID_SCALE + motB_offset;
    C = effort_set * K_effort/PID_SCALE + motC_offset;
    D = effort_set * K_effort/PID_SCALE + motD_offset;

    /* ===== CONTROL MIXING ===== */
    // Apply pitch control (affects front/rear motor pairs)
    if (pitch_effort > 0) {
        A += pitch_effort; // Front motors get more power
        D += pitch_effort;
    }
    else if (pitch_effort < 0) {
        B -= pitch_effort; // Rear motors get more power
        C -= pitch_effort;
    }

    // Apply roll control (affects left/right motor pairs)
    if (roll_effort > 0) {
        A += roll_effort; // Left motors get more power
        B += roll_effort;
    }
    else if (roll_effort < 0) {
        C -= roll_effort; // Right motors get more power
        D -= roll_effort;
    }

    // Apply yaw control (affects diagonal motor pairs)
    if (yaw_effort > 0) {
        B += yaw_effort; // CW motors get more power
        D += yaw_effort;
    }
    else if (yaw_effort < 0) {
        A -= yaw_effort; // CCW motors get more power
        C -= yaw_effort;
    }


    /* ===== SAFETY LIMITS ===== */
    // Clamp all motors between 960 (0%) and 1500 (≈50% for safety)
    if ((A < 960) | (stopFlag == true)) A = 960;
    if (A > 1600) A = 1700;

    if ((B < 960) | (stopFlag == true)) B = 960;
    if (B > 1700) B = 1700;

    if ((C < 960) | (stopFlag == true)) C = 960;
    if (C > 1700) C = 1700;

    if ((D < 960) | (stopFlag == true)) D = 960;
    if (D > 1700) D = 1700;

    // Debug output (commented)
    //printf("%d,%d,%d,%d \r\n", pitch_true, 0, 0, 0);

    /* ===== PWM OUTPUT UPDATE ===== */
    // Update timer compare registers to set motor speeds
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, A);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, B);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, C);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, D);
}
