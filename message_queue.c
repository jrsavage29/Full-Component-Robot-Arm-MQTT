/*
 * sensor_queue.c
 *
 *  Created on: Jun 11, 2020
 *      Author: jrsav
 */
#include <message_queue.h>

int sendToReadDataQueue(Motor_Angles data)
{

    if(xQueueSend(Read_Data_Queue, &data, portMAX_DELAY) == pdPASS)
    {
        //This means that it worked!
        //dbgOutputLoc(DBG_MESSAGE_QUEUE_SEND_SERVO_ANGLES);
        return SEND_TIME_TO_QUEUE_SUCCESS;
    }

    else //sending to queue failed
    {
        //dbgOutputLoc(DBG_MESSAGE_QUEUE_SEND_FAILURE);
        return SEND_TIME_TO_QUEUE_FAIL;
    }

}

//Non blocking timer status update from ISR
int sendTimerStatusfromISR( Motor_Angles timer_status)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    if(xQueueSendFromISR(Read_Data_Queue, &timer_status, &xHigherPriorityTaskWoken))
    {
        //dbgOutputLoc(DBG_MESSAGE_QUEUE_SEND_TIMER_EXPIRED);
        //This means that it worked!
    }

    if(xHigherPriorityTaskWoken)
    {
        //dbgOutputLoc();
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    else
    {
        //dbgOutputLoc();
        return SEND_TIME_TO_QUEUE_SUCCESS;
    }

    //dbgOutputLoc(DBG_MESSAGE_QUEUE_SEND_FAILURE);
    return SEND_TIME_TO_QUEUE_FAIL;
}

//Send a blocking status update of whether or not motor configuration is complete
int setStatusofConfiguration(Task_Status status)
{

    if(xQueueSend(Task_Done_Queue, &status, portMAX_DELAY) == pdPASS)
    {
        //This means that it worked!
        //dbgOutputLoc(DBG_MESSAGE_QUEUE_SEND_STATUS_OF_CONFIG);
        return SEND_TIME_TO_QUEUE_SUCCESS;
    }

    else //sending to queue failed
    {
        //dbgOutputLoc(DBG_MESSAGE_QUEUE_SEND_FAILURE);
        return SEND_TIME_TO_QUEUE_FAIL;
    }

}

int getStatusofConfiguration(Task_Status * status)
{
    if(xQueueReceive(Task_Done_Queue, &currentStatus, portMAX_DELAY) == pdPASS)
    {
        //dbgOutputLoc(DBG_MESSAGE_QUEUE_RECEIVE_STATUS_OF_CONFIG);
        *status = currentStatus; //This means we successfully read from the queue
        return READ_FROM_QUEUE_SUCCESS;
    }
    else
    {
        //Could not receive from the queue.
        //dbgOutputLoc(DBG_MESSAGE_QUEUE_RECEIVE_FAILURE);
        return READ_FROM_QUEUE_FAIL; //This means we were not able to read from the queue
    }
}

//Blocking version
int readDataReadQueueBlocking( Motor_Angles * data)
{
    if( xQueueReceive(Read_Data_Queue, &ReadData, portMAX_DELAY) )
    {
        //dbgOutputLoc(DBG_MESSAGE_QUEUE_RECEIVE_SERVO_ANGLES);
        *data = ReadData;
        return READ_FROM_QUEUE_SUCCESS;
    }

    else
    {
        //Could not receive from the queue.
        //dbgOutputLoc(DBG_MESSAGE_QUEUE_RECEIVE_FAILURE);
        return READ_FROM_QUEUE_FAIL; //This means we were not able to read from the queue
    }

}


void createQueue()
{
    //Timer_Status_Queue = xQueueCreate(1, sizeof(int)); //create a queue that updates the status of timer when it expires
    Read_Data_Queue = xQueueCreate(1, sizeof(Motor_Angles)); //create a queue that contains incremental updates about the angle changes
    Task_Done_Queue = xQueueCreate(1, sizeof(Task_Status)); //create a queue that's used for confirming task completion
}

