/*
 * motor.c
 *
 *  Created on: Feb 24, 2026
 *      Author: Mohammadreza
 */

#include "motor.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

volatile int32_t encoder_position = 0;
volatile uint32_t encoder_last_cnt = 0;
volatile float motor_rpm = 0;
volatile float pwm = 0.0f;
volatile float target_rpm_cmd = 0.0f;
volatile float target_rpm_ramped = 0.0f;
volatile int32_t target_position = 0;
volatile float position_speed_cmd = 0.0f;
volatile uint8_t position_reached = 0;

MotorMode_t motor_mode;

PID_t motor_pid =
{
    .kp = 1.8f,
    .ki = 1.4f,
    .kd = 0.0f,

    .integral = 0.0f,
    .prev_error = 0.0f,

    .out_min = -MAX_DUTY,
    .out_max = MAX_DUTY
};
void Encoder_Start(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    encoder_last_cnt = TIM2->CNT;
}
void Motor_SetPWM(int16_t pwm)
{
    if (pwm > MAX_DUTY) pwm = MAX_DUTY;
    if (pwm < -MAX_DUTY) pwm = -MAX_DUTY;

    if (pwm > 0)
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);

    }
    else if (pwm < 0)
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, -pwm);
    }
    else
    {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    }
}


float PID_Update(PID_t *pid, float setpoint, float measurement)
{
    float dt = 1.0f / CONTROL_HZ;

    float error = setpoint - measurement;

    pid->integral += error * dt;

    float integral_limit = pid->out_max / pid->ki;

    if (pid->integral > integral_limit)
        pid->integral = integral_limit;
    if (pid->integral < -integral_limit)
        pid->integral = -integral_limit;

    float output =
        pid->kp * error +
        pid->ki * pid->integral;

    if (output > pid->out_max)
        output = pid->out_max;
    if (output < pid->out_min)
        output = pid->out_min;

    return output;
}

void Ramp_Update(void)
{
	float accel_step = ACCEL_RPM_PER_SEC / CONTROL_HZ;
	float decel_step = DECEL_RPM_PER_SEC / CONTROL_HZ;

	if (target_rpm_ramped < target_rpm_cmd)
	{
		target_rpm_ramped += accel_step;
		if (target_rpm_ramped > target_rpm_cmd)
			target_rpm_ramped = target_rpm_cmd;
	}
	else if (target_rpm_ramped > target_rpm_cmd)
	{
		target_rpm_ramped -= decel_step;
		if (target_rpm_ramped < target_rpm_cmd)
			target_rpm_ramped = target_rpm_cmd;
	}

}

void Position_Controller(int32_t target, int32_t current)
{
    int32_t error = target - current;
    float speed_cmd = KP_POSITION * error;

    if (speed_cmd > MAX_POSITION_SPEED_RPM){
        speed_cmd = MAX_POSITION_SPEED_RPM;}
    else if (speed_cmd < -MAX_POSITION_SPEED_RPM){
        speed_cmd = -MAX_POSITION_SPEED_RPM;}

    target_rpm_cmd = speed_cmd;

}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
}
