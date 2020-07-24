/*
 * sensor_queue.h
 *
 *  Created on: Jun 11, 2020
 *      Author: jrsav
 */

#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"


/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>

/* Board Header file */
#include "ti_drivers_config.h"

/* Sensor queue routine*/
#ifndef SENSOR_QUEUE_H_
#define SENSOR_QUEUE_H_

#define READ_FROM_QUEUE_SUCCESS 18
#define READ_FROM_QUEUE_FAIL -18
#define SEND_TIME_TO_QUEUE_SUCCESS 40
#define SEND_TIME_TO_QUEUE_FAIL -40
#define TIME_CONTEXT_SWITCH_TRUE 45
#define TIME_CONTEXT_SWITCH_FALSE -45
#define SEND_SENSOR_TO_QUEUE_FAIL -71
#define SENSOR_CONTEXT_SWITCH_TRUE 81
#define SENSOR_CONTEXT_SWITCH_FALSE -81

#define IDLE_MODE           0
#define SERVO_SETUP_MODE    1
#define PICK_UP_MODE        2
#define DROP_OFF_MODE       3

#define TRUE_VAL                1
#define FALSE_VAL               0
    typedef struct
    {
        int angle_Base; //The camera sends in coordinates of where the object is located
        char* object_color;

        //These will probably only be hard coded in for the complex angles. (Extend is based off how far away the rover is from the object)
        int angle_Extend, angle_Lift, angle_Claw;
        int timer_expired;




    }Motor_Angles;


    typedef struct
    {
        char* rover_Status; //The rover will send a signal for when it's moving or not

        int arm_status; //The current status of the robot arm

        char* color; // The camera also sends in the color it has detected
        char* position; //The camera sends the direction where the object is located

    }Task_Status;


    //static QueueHandle_t       Timer_Status_Queue;
    static QueueHandle_t       Task_Done_Queue;
    static QueueHandle_t       Read_Data_Queue;

    Motor_Angles ReadData;
    Task_Status currentStatus;
    int timerStatus;

    int sendTimerStatusfromISR( Motor_Angles timer_status);

    int sendToReadDataQueue(Motor_Angles data);

    //Any variable passed in is by reference
    int readDataReadQueueBlocking( Motor_Angles * data);

    int setStatusofConfiguration(Task_Status status);
    int getStatusofConfiguration(Task_Status * status);

    void createQueue();


#endif /* SENSOR_QUEUE_H_ */
