/*
 * timerone.h
 *
 *  Created on: Jun 11, 2020
 *      Author: Kevin Kleinegger
 */

#include <message_queue.h>
#include <stddef.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include "ti_drivers_config.h"
#include "task.h"
#include <ti/drivers/PWM.h>

//The following defines are based on the specs of the MG90 servos pulse width and pulse cycle
#define PWM_PERIOD                  20000   //20 ms or 20000 us
#define DEFAULT_BASE_DUTY_CYCLE     200 //1.15 us = 90, 2.66 us = 0, 0.4 us = 180
#define DEFAULT_EXTEND_DUTY_CYCLE   700 // 1.15 us = Min, 2.5 us = Max
#define DEFAULT_LIFT_DUTY_CYCLE     1500 // 0.5 us = Min, 2000 us = Max
#define DEFAULT_CLAW_DUTY_CYCLE     400 //  0.4 us = Min Closed, 2 us = Max Open

#define INITIAL_BASE_DUTY_CYCLE     1190
#define INITIAL_EXTEND_DUTY_CYCLE   700
#define INITIAL_LIFT_DUTY_CYCLE     1500 //1700
#define INITIAL_CLAW_DUTY_CYCLE     400

#define CONTINUING_CONFIG   -5
#define CONFIG_COMPLETE     5


#ifndef SERVO_PWM_H_
#define SERVO_PWM_H_

static PWM_Handle pwm_Base;
static int dutyCycle_Base; //increment by 10 us to represent 1 degree angle changes

static PWM_Handle pwm_Extend;
static int dutyCycle_Extend;

static PWM_Handle pwm_Lift;
static int dutyCycle_Lift;

static PWM_Handle pwm_Claw;
static int dutyCycle_Claw;

PWM_Params params;

void configurePWM();
int setPWM_Base(int angle_Base);
int setPWM_Extend(int angle_Extend);
int setPWM_Lift(int angle_Lift);
int setPWM_Claw(int angle_Claw);
void stopPWMs();



#endif /* SERVO_PWM_H_ */
