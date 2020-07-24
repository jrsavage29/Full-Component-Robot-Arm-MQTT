/*
 * timertwo.h
 *
 *  Created on: Jun 11, 2020
 *      Author: Kevin Kleinegger
 */



#include <stddef.h>
#include <math.h>
#include <message_queue.h>
#include <servo_PWM.h>
#include <timers.h>
#include <FreeRTOS.h>


/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
/* Board Header file */
#include "ti_drivers_config.h"

#ifndef SOFTWARE_TIMER_H_
#define SOFTWARE_TIMER_H_

#define AUTO_RELOAD_TIMER_PERIOD   pdMS_TO_TICKS(20) //in ms
#define CONTINUING_CONFIG   -5
#define CONFIG_COMPLETE     5
#define IDLE_MODE           0
#define SERVO_SETUP_MODE    1
#define PICK_UP_MODE        2
#define DROP_OFF_MODE       3
#define TIMER_EXPIRED       22

static TimerHandle_t xAutoReloadTimer;
static Timer_Handle HardwareTimer;
static Timer_Params Params;

static int first;

void configureSTimer();
void startSoftwareTimer();
void restartSoftwareTimer();
void stopSoftwareTimer();
BaseType_t checkIfTimerActive();
void softwareTimerCallback(TimerHandle_t xAutoReloadTimer);




#endif /* SOFTWARE_TIMER_H_ */
