/*
 * motors.h
 *
 *  Created on: 7 Jun. 2023
 *      Author: Reilly
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "tim.h"
#include <stdbool.h>

typedef struct Motor
{
	bool direction_a;
	bool direction_b;
	bool direction_c;
} Motor_t;


extern Motor_t motors;

void Motors_SetDuty(float a, float b, float c);
void Motors_SetDirection(bool a, bool b, bool c);
void Motors_SetMotors(float a, float b, float c);
void Motors_SetMovement(float throttle, float angle, float rotation);
#endif /* INC_MOTORS_H_ */
