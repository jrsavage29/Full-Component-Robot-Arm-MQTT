/*
 * timertwo.h
 *
 *  Created on: Jun 11, 2020
 *      Author: Kevin Kleinegger
 */

#ifndef DATATIMER_H_
#define DATATIMER_H_


#include <stddef.h>
#include <stdio.h>
/* Driver Header files */
#include <ti/drivers/Timer.h>
#include "debugIntercomponent.h"
/* Board Header file */
#include "ti_drivers_config.h"
#include "client_cbs.h"
#include "uart_term.h"

#define BUFFER_SIZE 500

extern int32_t MQTT_SendMsgToQueue(struct msgQueue *queueElement);

void timerCallback(Timer_Handle myHandle, int_fast16_t status);
void configureDataTimer();
void formatDataForQueue();

#endif /* DATATIMER_H_ */
