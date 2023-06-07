/*
 * motors.c
 *
 *  Created on: 7 Jun. 2023
 *      Author: Reilly
 */

#include "motors.h"
#include "main.h"
#include <math.h>

#define MOTOR_MAX 1.0f

Motor_t motor = {};

void Motors_SetDuty(float a, float b, float c)
{
	const uint32_t scaler = MOTOR_MAX * MOTOR_TIM_FREQ_MHZ * 1000 / MOTOR_PWM_KHZ;

	TIM1->CCR1 = c * scaler;
	TIM1->CCR2 = b * scaler;
	TIM1->CCR3 = a * scaler;
}

void Motors_SetDirection(bool a, bool b, bool c)
{
	motor.direction_a = a;
	motor.direction_b = b;
	motor.direction_c = c;

	// TODO: Can porbably just write to the ports and skip the HAL calls
	HAL_GPIO_WritePin(MOTOR_A_DIRN_GPIO_Port, MOTOR_A_DIRN_Pin, a);
	HAL_GPIO_WritePin(MOTOR_B_DIRN_GPIO_Port, MOTOR_B_DIRN_Pin, b);
	HAL_GPIO_WritePin(MOTOR_C_DIRN_GPIO_Port, MOTOR_C_DIRN_Pin, c);

	// Second set of
	HAL_GPIO_WritePin(MOTOR_AN_DIRN_GPIO_Port, MOTOR_AN_DIRN_Pin, !a);
	HAL_GPIO_WritePin(MOTOR_BN_DIRN_GPIO_Port, MOTOR_BN_DIRN_Pin, !b);
	HAL_GPIO_WritePin(MOTOR_CN_DIRN_GPIO_Port, MOTOR_CN_DIRN_Pin, !c);
}


void Motors_SetMotors(float a, float b, float c)
{
	Motors_SetDirection(a < 0.0f, b < 0.0f, c < 0.0f);
	Motors_SetDuty(fabs(a), fabs(b), fabs(c));
}

void Motors_SetMovement(float throttle, float angle, float rotation)
{

#define sqrt_3 1.73205
	float a = throttle * (sin(angle) + cos (angle) / (2.0f + sqrt_3)) + rotation;
	float c = - throttle * (sin(angle) - cos (angle) / (2.0f + sqrt_3)) + rotation;
    float b = a + c - 3*rotation;

    // Scale in case any motor > 1.0
    if (a > 1.0f)
    {
    	a = 1.0f;
    	b /= a;
    	c /= a;
    }
    if (b > 1.0f)
    {
    	b = 1.0f;
    	a /= b;
    	c /= b;
    }
    if (c > 1.0f)
    {
    	c = 1.0f;
    	b /= c;
    	a /= c;
    }

    Motors_SetMotors(a, b, c);
}
