/*
 * sensor_state.h
 *
 *  Created on: Jun 14, 2020
 *      Author: tyler
 */

#include <message_queue.h>
#include <softwareTimer.h>
#include <servo_PWM.h>
#include <stdio.h>
#include <string.h>

#define IDLE_MODE           0
#define SERVO_SETUP_MODE    1
#define PICK_UP_MODE        2
#define DROP_OFF_MODE       3
#define TIMER_EXPIRED       22

#ifndef ROBOT_ARM_FSM_H_
#define ROBOT_ARM_FSM_H_

Motor_Angles angle;

static int status0;
static int status1;
static int status2;
static int status3;

static int baseAngle;
static int extendAngle;
static int liftAngle;
static int clawAngle;

static int firstRead;

// Subroutine declarations.
void FSMsubroutine();
//void printNum(double num);

#endif /* SENSOR_STATE_H_ */
