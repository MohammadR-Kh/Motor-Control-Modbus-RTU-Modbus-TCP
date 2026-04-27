/*
 * motor.h
 *
 *  Created on: Feb 24, 2026
 *      Author: Mohammadreza
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f4xx_hal.h"

#define MAX_DUTY 799
#define ENCODER_CPR   1600.0f
#define ACCEL_RPM_PER_SEC  4000.0f
#define DECEL_RPM_PER_SEC  8000.0f
#define CONTROL_HZ         500.0f
#define MAX_POSITION_SPEED_RPM  3600.0f
#define KP_POSITION             0.035f
#define POSITION_TOLERANCE_COUNTS  10

typedef enum {
    MOTOR_MODE_IDLE = 0,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION
} MotorMode_t;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

    float out_min;
    float out_max;
} PID_t;

extern PID_t motor_pid;
extern MotorMode_t motor_mode;

void Encoder_Start(void);
//void Motor_SetDuty(uint16_t duty);
//void Motor_SetDirection(MotorDirection_t dir);
float PID_Update(PID_t *pid, float setpoint, float measurement);
void Ramp_Update(void);
//void Motor_CommandSpeed(float rpm);
//float Position_Controller(int32_t target, int32_t current);
void PID_Reset(PID_t *pid);

void Motor_SetPWM(int16_t pwm);
//void Motor_SetOutput(MotorDirection_t dir, uint16_t duty);
void Position_Controller(int32_t target, int32_t current);

#endif /* INC_MOTOR_H_ */
