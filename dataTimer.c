/*
 * timertwo.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Kevin Kleinegger
 */

#include "dataTimer.h"


void configureDataTimer()
{
    Timer_Handle timer;
    Timer_Params params;

    Timer_Params_init(&params);
    params.period = 1000000; //1s
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer = Timer_open(CONFIG_TIMER_0, &params);
    Timer_start(timer);
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    dbgOutputLoc(ENTER_TIMER_ISR);
    formatDataForQueue();
    dbgOutputLoc(EXIT_TIMER_ISR);
}

void formatDataForQueue()
{
        struct msgQueue queueElement;
        queueElement.event = TESTING_PUBLISH;

        char jsonObject[BUFFER_SIZE];
        static int sequenceNumber = 0;

        static char * arm_status = "READY";
        static char * rover_status = "STOPPED";
        static int counter = 0;

        if(counter >= 25 && counter < 40)
        {
            arm_status = "HOLDING";
        }

        else if(counter  >= 40 && counter < 45)
        {
            arm_status = "DROP OFF";
        }

        else if(counter == 45)
        {
            arm_status = "READY";
            counter = 0;
        }

        counter++;

//        char* topic_name = "Summer/Team2/Arm/PickupStatus";
//        snprintf(jsonObject, BUFFER_SIZE, "{\"Sequence Number\":\"%d\", \"Arm Status\":\"%s\", \"Topic\":\"%s\"}", sequenceNumber, arm_status, topic_name);

        char* topic_name = "Summer/Testing/Signal";
        snprintf(jsonObject, BUFFER_SIZE, "{\"Topic\":\"%s\", \"Sequence Number\":\"%d\", \"Rover Status\":\"%s\"}", topic_name, sequenceNumber, rover_status );
        queueElement.msgPtr = jsonObject;
        sequenceNumber++;

        /* write message indicating publish message                             */
        if(MQTT_SendMsgToQueue(&queueElement))
        {
            dbgOutputLoc(ERROR_QUEUE_FULL);
            //UART_PRINT("\n\n\rQueue is full\n\n\r");
        }
}
