/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*****************************************************************************

   Application Name     -   MQTT Client
   Application Overview -   The device is running a MQTT client which is
                           connected to the online broker. Three LEDs on the
                           device can be controlled from a web client by
                           publishing msg on appropriate topics. Similarly,
                           message can be published on pre-configured topics
                           by pressing the switch buttons on the device.

   Application Details  - Refer to 'MQTT Client' README.html

*****************************************************************************/
//*****************************************************************************
//
//! \addtogroup mqtt_server
//! @{
//
//*****************************************************************************
/* Standard includes                                                         */
#include <stdlib.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include "queue.h"


/* TI-Driver includes                                                        */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Simplelink includes                                                       */
#include <ti/drivers/net/wifi/simplelink.h>

/* SlNetSock includes                                                        */
#include <ti/drivers/net/wifi/slnetifwifi.h>

/* MQTT Library includes                                                     */
#include <ti/net/mqtt/mqttclient.h>

/* Common interface includes                                                 */
#include "network_if.h"
#include "uart_term.h"

/* TI-DRIVERS Header files */
#include "ti_drivers_config.h"

/* Application includes                                                      */
#include "client_cbs.h"
#include "dataTimer.h"
#include "debugIntercomponent.h"
#include <jsmn.h>
#include <message_queue.h>
#include <softwareTimer.h>
#include <servo_PWM.h>

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
/* enables secured client                                                    */
//#define SECURE_CLIENT

/* enables client authentication by the server                               */
#define CLNT_USR_PWD

#define CLIENT_INIT_STATE        (0x01)
#define MQTT_INIT_STATE          (0x04)

#define APPLICATION_VERSION      "1.1.1"
#define APPLICATION_NAME         "MQTT client"

#define SLNET_IF_WIFI_PRIO       (5)

/* Operate Lib in MQTT 3.1 mode.                                             */
#define MQTT_3_1_1               false
#define MQTT_3_1                 true

#define WILL_TOPIC               "Client"
#define WILL_MSG                 "Client Stopped"
#define WILL_QOS                 MQTT_QOS_0
#define WILL_RETAIN              false

/* Defining Broker IP address and port Number                                */
//#define SERVER_ADDRESS           "messagesight.demos.ibm.com"
#define SERVER_ADDRESS           "m16.cloudmqtt.com"
#define SERVER_IP_ADDRESS        "m16.cloudmqtt.com"
#define PORT_NUMBER              15663
#define SECURED_PORT_NUMBER      8883
#define LOOPBACK_PORT            1882

/* Clean session flag                                                        */
#define CLEAN_SESSION            true

/* Retain Flag. Used in publish message.                                     */
#define RETAIN_ENABLE            1

/* Defining Number of subscription topics                                    */
#define SUBSCRIPTION_TOPIC_COUNT 2

/* Defining Subscription Topic Values                                        */
#define SUBSCRIPTION_TOPIC0         "Summer/Team2/Camera"
//#define SUBSCRIPTION_TOPIC1         "Summer/Team2/Arm/PickupStatus"
//#define SUBSCRIPTION_TOPIC1         "Summer/Team2/Rover/StopSignal"
#define SUBSCRIPTION_TOPIC1         "Summer/Testing/Signal"


/* Defining Publish Topic Values                                             */
//#define PUBLISH_TOPIC1           "Summer/Team2/Arm/PickupStatus"
#define PUBLISH_TOPIC1           "Summer/Testing/Signal"
#define PUBLISH_TOPIC2           "Summer/Team2/Statistics/Jamahl"

/* Spawn task priority and Task and Thread Stack Size                        */
#define TASKSTACKSIZE            2048
#define RXTASKSIZE               4096
#define MQTTTHREADSIZE           2048
#define RTHREADSIZE              1024
#define SPAWN_TASK_PRIORITY      9

#define THREADSTACKSIZE2         1024
#define THREADSTACKSIZE3         1024

/* secured client requires time configuration, in order to verify server     */
/* certificate validity (date).                                              */

/* Day of month (DD format) range 1-31                                       */
#define DAY                      1
/* Month (MM format) in the range of 1-12                                    */
#define MONTH                    5
/* Year (YYYY format)                                                        */
#define YEAR                     2017
/* Hours in the range of 0-23                                                */
#define HOUR                     12
/* Minutes in the range of 0-59                                              */
#define MINUTES                  33
/* Seconds in the range of 0-59                                              */
#define SEC                      21

/* Number of files used for secure connection                                */
#define CLIENT_NUM_SECURE_FILES  1

/* Expiration value for the timer that is being used to toggle the Led.      */
//#define TIMER_EXPIRATION_VALUE   100 * 1000000

#define TRUE_VAL    1
#define FALSE_VAL   0

#define IDLE_MODE           0
#define SERVO_SETUP_MODE    1
#define PICK_UP_MODE        2
#define DROP_OFF_MODE       3


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void DisplayBanner(char * AppName);
void * MqttClient(void *pvParameters);
void * ReceiveClient(void *pvParameters);
void *robotArmThread(void *pvParameters);
void *readDataThread(void *pvParameters);
//void readFromMQTT(char* position, char* color, char* rover_status);
void pickUp(Motor_Angles data, int * robot_arm_status);
void dropOff(int * robot_arm_status, char* object_color);
void Mqtt_ClientStop(uint8_t disconnect);
void Mqtt_ServerStop();
void Mqtt_Stop();
void Mqtt_start();
void Mqtt_start2();
int32_t Mqtt_IF_Connect();
int32_t MqttServer_start();
int32_t MqttClient_start();
int32_t MQTT_SendMsgToQueue(struct msgQueue *queueElement);
int32_t MQTT_SendMsgToQ(struct msgQueue *queueElement);

//****************************************************************************
//                         EXTERNAL FUNTIONS
//****************************************************************************
extern int32_t ti_net_SlNet_initConfig();

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************

/* Connection state: (0) - connected, (negative) - disconnected              */
int32_t gApConnectionState = -1;
uint32_t gInitState = 0;
uint32_t memPtrCounterfree = 0;
bool gResetApplication = false;
static MQTTClient_Handle gMqttClient;
MQTTClient_Params MqttClientExmple_params;
unsigned short g_usTimerInts;

/* Receive task handle                                                       */
pthread_t g_rx_task_hndl = (pthread_t) NULL;
uint32_t gUiConnFlag = 0;

/* AP Security Parameters                                                    */
SlWlanSecParams_t SecurityParams = { 0 };

/* Client ID                                                                 */
/* If ClientId isn't set, the MAC address of the device will be copied into  */
/* the ClientID parameter.                                                   */
char ClientId[13] = {'\0'};

/* Client User Name and Password                                             */
const char *ClientUsername = "zkrbbakj";
const char *ClientPassword = "tDWwUUrBl3lH";

/* Subscription topics and qos values                                        */
char *topic[SUBSCRIPTION_TOPIC_COUNT] =
{ SUBSCRIPTION_TOPIC0, SUBSCRIPTION_TOPIC1 };

unsigned char qos[SUBSCRIPTION_TOPIC_COUNT] =
{ MQTT_QOS_0, MQTT_QOS_0 };

/* Publishing topics and messages                                            */
const char *publish_topic1 = { PUBLISH_TOPIC1 };
const char *publish_topic2 = { PUBLISH_TOPIC2 };

/* Message Queue                                                             */
mqd_t g_PBQueue;
mqd_t g_JBQueue;
pthread_t mqttThread = (pthread_t) NULL;
pthread_t appThread = (pthread_t) NULL;
timer_t g_timer;

/* Printing new line                                                         */
char lineBreak[] = "\n\r";

//*****************************************************************************
//                 Banner VARIABLES
//*****************************************************************************
#ifdef  SECURE_CLIENT

char *Mqtt_Client_secure_files[CLIENT_NUM_SECURE_FILES] = {"ca-cert.pem"};

/*Initialization structure to be used with sl_ExtMqtt_Init API. In order to  */
/*use secured socket method, the flag MQTTCLIENT_NETCONN_SEC, cipher,        */
/*n_files and secure_files must be configured.                               */
/*certificates also must be programmed  ("ca-cert.pem").                     */
/*The first parameter is a bit mask which configures server address type and */
/*security mode.                                                             */
/*Server address type: IPv4, IPv6 and URL must be declared with The          */
/*corresponding flag.                                                        */
/*Security mode: The flag MQTTCLIENT_NETCONN_SEC enables the security (TLS)  */
/*which includes domain name verification and certificate catalog            */
/*verification, those verifications can be disabled by adding to the bit mask*/
/*MQTTCLIENT_NETCONN_SKIP_DOMAIN_NAME_VERIFICATION and                       */
/*MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION flags             */
/*Example: MQTTCLIENT_NETCONN_IP6 | MQTTCLIENT_NETCONN_SEC |                 */
/*MQTTCLIENT_NETCONN_SKIP_CERTIFICATE_CATALOG_VERIFICATION                   */
/*For this bit mask, the IPv6 address type will be in use, the security      */
/*feature will be enable and the certificate catalog verification will be    */
/*skipped.                                                                   */
/*Note: The domain name verification requires URL Server address type        */
/*      otherwise, this verification will be disabled.                       */
MQTTClient_ConnParams Mqtt_ClientCtx =
{
    MQTTCLIENT_NETCONN_IP4 | MQTTCLIENT_NETCONN_SEC,
    SERVER_IP_ADDRESS,  //SERVER_ADDRESS,
    SECURED_PORT_NUMBER, //  PORT_NUMBER
    SLNETSOCK_SEC_METHOD_SSLv3_TLSV1_2,
    SLNETSOCK_SEC_CIPHER_FULL_LIST,
    CLIENT_NUM_SECURE_FILES,
    Mqtt_Client_secure_files
};

void setTime()
{
    SlDateTime_t dateTime = {0};
    dateTime.tm_day = (uint32_t)DAY;
    dateTime.tm_mon = (uint32_t)MONTH;
    dateTime.tm_year = (uint32_t)YEAR;
    dateTime.tm_hour = (uint32_t)HOUR;
    dateTime.tm_min = (uint32_t)MINUTES;
    dateTime.tm_sec = (uint32_t)SEC;
    sl_DeviceSet(SL_DEVICE_GENERAL, SL_DEVICE_GENERAL_DATE_TIME,
                 sizeof(SlDateTime_t), (uint8_t *)(&dateTime));
}

#else
MQTTClient_ConnParams Mqtt_ClientCtx =
{
    MQTTCLIENT_NETCONN_URL,
    SERVER_ADDRESS,
    PORT_NUMBER, 0, 0, 0,
    NULL
};
#endif

/* Initialize the will_param structure to the default will parameters        */
MQTTClient_Will will_param =
{
    WILL_TOPIC,
    WILL_MSG,
    WILL_QOS,
    WILL_RETAIN
};

//*****************************************************************************
//
//! MQTT_SendMsgToQueue - Utility function that receive msgQueue parameter and
//! tries to push it the queue with minimal time for timeout of 0.
//! If the queue isn't full the parameter will be stored and the function
//! will return 0.
//! If the queue is full and the timeout expired (because the timeout parameter
//! is 0 it will expire immediately), the parameter is thrown away and the
//! function will return -1 as an error for full queue.
//!
//! \param[in] struct msgQueue *queueElement
//!
//! \return 0 on success, -1 on error
//
//*****************************************************************************
int32_t MQTT_SendMsgToQueue(struct msgQueue *queueElement)
{
    struct timespec abstime = {0};

    clock_gettime(CLOCK_REALTIME, &abstime);

    if(g_PBQueue)
    {
        /* send message to the queue                                        */
        if(mq_timedsend(g_PBQueue, (char *) queueElement,
                        sizeof(struct msgQueue), 0, &abstime) == 0)
        {
            return(0);
        }
    }
    return(-1);
}

int32_t MQTT_SendMsgToQ(struct msgQueue *queueElement)
{
    struct timespec abstime = {0};

    clock_gettime(CLOCK_REALTIME, &abstime);

    if(g_JBQueue)
    {
        dbgOutputLoc(AFTER_SEND_MSG_Q);

        /* send message to the queue                                        */
        if(mq_timedsend(g_JBQueue, (char *) queueElement,
                        sizeof(struct msgQueue), 0, &abstime) == 0)
        {
            return(0);
        }
    }
    return(-1);
}

static void DisplayBanner(char * AppName)
{
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t    CC32xx %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}

void * MqttClientThread(void * pvParameters)
{
    struct msgQueue queueElement;
    struct msgQueue queueElemRecv;

    MQTTClient_run((MQTTClient_Handle)pvParameters);

    queueElement.event = LOCAL_CLIENT_DISCONNECTION;
    queueElement.msgPtr = NULL;

    /*write message indicating disconnect Broker message.                   */
    if(MQTT_SendMsgToQueue(&queueElement))
    {
        UART_PRINT(
            "\n\n\rQueue is full, throw first msg and send the new one\n\n\r");
        mq_receive(g_PBQueue, (char*) &queueElemRecv, sizeof(struct msgQueue),
                   NULL);
        MQTT_SendMsgToQueue(&queueElement);
    }

    pthread_exit(0);

    return(NULL);
}

//*****************************************************************************
//
//! Task implementing MQTT Server plus client bridge
//!
//! This function
//!    1. Initializes network driver and connects to the default AP
//!    2. Initializes the mqtt client ans server libraries and set up MQTT
//!       with the remote broker.
//!    3. set up the button events and their callbacks(for publishing)
//!    4. handles the callback signals
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void * MqttClient(void *pvParameters)
{
    struct msgQueue queueElemRecv;
    long lRetVal = -1;
    //char *tmpBuff;

    /*Initializing Client and Subscribing to the Broker.                     */
    if(gApConnectionState >= 0)
    {
        lRetVal = MqttClient_start();
        if(lRetVal == -1)
        {
            UART_PRINT("MQTT Client lib initialization failed\n\r");
            pthread_exit(0);
            return(NULL);
        }
    }

    /*handling the signals from various callbacks including the push button  */
    /*prompting the client to publish a msg on PUB_TOPIC OR msg received by  */
    /*the server on enrolled topic(for which the on-board client ha enrolled)*/
    /*from a local client(will be published to the remote broker by the      */
    /*client) OR msg received by the client from the remote broker (need to  */
    /*be sent to the server to see if any local client has subscribed on the */
    /*same topic).                                                           */
    for(;; )
    {

        /*waiting for signals                                                */
        mq_receive(g_PBQueue, (char*) &queueElemRecv, sizeof(struct msgQueue),
                   NULL);

        switch(queueElemRecv.event)
        {
        case LOCAL_CLIENT_DISCONNECTION:
            UART_PRINT("\n\rOn-board Client Disconnected\n\r\r\n");
            gUiConnFlag = 0;
            break;

        case THREAD_TERMINATE_REQ:
            gUiConnFlag = 0;
            pthread_exit(0);
            return(NULL);

        case TESTING_PUBLISH:
        {
            lRetVal =
                MQTTClient_publish(gMqttClient, (char*) publish_topic1, strlen(
                                      (char*)publish_topic1),
                                  (char*)queueElemRecv.msgPtr,
                                  strlen((char*) queueElemRecv.msgPtr), MQTT_QOS_0 |
                                  ((RETAIN_ENABLE) ? MQTT_PUBLISH_RETAIN : 0));

            break;
        }
        case SEND_DATA_TO_STATS:
        {
            lRetVal =
                MQTTClient_publish(gMqttClient, (char*) publish_topic2, strlen(
                                      (char*)publish_topic2),
                                  (char*)queueElemRecv.msgPtr,
                                  strlen((char*) queueElemRecv.msgPtr), MQTT_QOS_0 |
                                  ((RETAIN_ENABLE) ? MQTT_PUBLISH_RETAIN : 0));
        }
        default:
            //send error
            break;
        }
    }
}

//*****************************************************************************

void * ReceiveClient(void *pvParameters)
{
    dbgOutputLoc(INSIDE_RECV_CLIENT);

    struct msgQueue queueElemRecv;
    char *tmpBuff;
    static char* color = "LIGHT";
    static char* position = "60";
    static char* rover_status = "MOVING";
    Task_Status    data;


    /*handling the signals from various callbacks including the push button  */
    /*prompting the client to publish a msg on PUB_TOPIC OR msg received by  */
    /*the server on enrolled topic(for which the on-board client ha enrolled)*/
    /*from a local client(will be published to the remote broker by the      */
    /*client) OR msg received by the client from the remote broker (need to  */
    /*be sent to the server to see if any local client has subscribed on the */
    /*same topic).                                                           */
    for(;; )
    {
        /*waiting for signals                                                */
        mq_receive(g_JBQueue, (char*) &queueElemRecv, sizeof(struct msgQueue),
                   NULL);
        printString("message received from queue\n\r");

        dbgOutputLoc(INSIDE_RECV_CLIENT_FOR);

        switch(queueElemRecv.event)
        {
        case MSG_RECV_BY_CLIENT:
        {
            printString("Data: ");
            tmpBuff = (char *) ((char *) queueElemRecv.msgPtr + 13 + queueElemRecv.topLen );
            printString(tmpBuff);
            printString("\n\r");

            struct msgQueue queueElemStats;
            queueElemStats.event = SEND_DATA_TO_STATS;
            queueElemStats.msgPtr = tmpBuff;

            if(MQTT_SendMsgToQueue(&queueElemStats))
            {
//
            }

            jsmn_parser p;
            jsmntok_t tokens[25];

            jsmn_init(&p);
            jsmn_parse(&p, tmpBuff, strlen(tmpBuff), tokens, 25);

            jsmntok_t key = tokens[5];
            unsigned int length = key.end - key.start;
            char keyString[length + 1];
            memcpy(keyString, &tmpBuff[key.start], length);
            keyString[length] = '\0';
            printString("Identifying keyString: ");
            printString(keyString);
            printString("\n\r");
            char* keyStringTemp = keyString;

            if( strcmp(keyStringTemp, "Rover Status") == 0 )
            {
                jsmntok_t keyR = tokens[6];
                unsigned int lengthR = keyR.end - keyR.start;
                char keyStringR[length + 1];
                memcpy(keyStringR, &tmpBuff[keyR.start], lengthR);
                keyStringR[lengthR] = '\0';
                printString("keyStringRover Status: ");
                printString(keyStringR);
                printString("\n\r");
                rover_status = keyStringR;

            }

            if( strcmp(keyStringTemp, "Color") == 0 )
            {
                //color = "DARK";
                //position = "180";

                jsmntok_t keyC = tokens[6];
                unsigned int lengthC = keyC.end - keyC.start;
                char keyStringC[lengthC + 1];
                memcpy(keyStringC, &tmpBuff[keyC.start], lengthC);
                keyStringC[lengthC] = '\0';
                printString("keyStringCamera Color: ");
                printString(keyStringC);
                printString("\n\r");
                color = keyStringC;


                keyC = tokens[8];
                lengthC = keyC.end - keyC.start;
                memcpy(keyStringC, &tmpBuff[keyC.start], lengthC);
                keyStringC[lengthC] = '\0';
                printString("keyStringCamera Position: ");
                printString(keyStringC);
                printString("\n\r");
                position = keyStringC;

            }

            data.rover_Status = rover_status;
            data.color = color;
            data.position = position;

            setStatusofConfiguration(data);

            break;
        }
        default:
            //send error
            break;
        }
    }
}

void *robotArmThread(void *pvParameters)
{
    dbgOutputLoc(DBG_MAINTHREAD_ENTER);
    Motor_Angles data;
    static int robot_arm_status = PICK_UP_MODE;
    static int valid_color = FALSE_VAL;
    static int initial_check = TRUE_VAL;
    static char* rover_status;
    static char* object_color;

    while(1)
    {
        dbgOutputLoc(DBG_MAINTHREAD_READ_DATA);
        readDataReadQueueBlocking(&data);


        //if( strcmp(data.rover_Status, "STOPPED") == 0 || data.timer_expired == TRUE_VAL)
        //{
        if(initial_check == TRUE_VAL) //Check if we initially receive a valid color
        {
            initial_check = FALSE_VAL;
            rover_status = data.rover_Status;

            if( ( strcmp(data.object_color, "LIGHT") == 0 || strcmp(data.object_color, "DARK") == 0 ) && robot_arm_status == PICK_UP_MODE)
            {
                object_color = data.object_color;
                valid_color = TRUE_VAL;
            }

            else if( !(strcmp(data.object_color, "LIGHT") == 0 || strcmp(data.object_color, "DARK") == 0) )
            {
                valid_color = FALSE_VAL;
            }
        }

        if(valid_color == TRUE_VAL && strcmp(rover_status, "STOPPED") == 0)
        {
            if( robot_arm_status == PICK_UP_MODE )
            {
                //data.timer_expired = TRUE_VAL;
                pickUp(data, &robot_arm_status);

                if(robot_arm_status == DROP_OFF_MODE)
                {
                    initial_check = TRUE_VAL;
                }

            }

            else if( robot_arm_status == DROP_OFF_MODE )
            {
                dropOff(&robot_arm_status, object_color);

                if(robot_arm_status == PICK_UP_MODE)
                {
                    initial_check = TRUE_VAL;
                }
            }
        }

        else if( strcmp(rover_status, "MOVING") == 0 )
        {
            initial_check = TRUE_VAL;
        }
        //}
        //UART_PRINT("Received Data!");

        dbgOutputLoc(DBG_MAINTHREAD_EXIT);
    }

    //dbgOutputLoc(DBG_MAINTHREAD_EXIT);



}

//create another thread that reads from the the mqtt and stores the data in the DetectedAngles queue
void *readDataThread(void *pvParameters)
{
    dbgOutputLoc(DBG_READTHREAD_ENTER);
    Task_Status status;
    Motor_Angles sending_data;
    static int sent = false;
    //status.arm_status = IDLE_MODE;
    //setStatusofConfiguration(status);

    while(1)
    {
        //UART_PRINT("IN WHILE LOOP OF READ DATA THREAD \r\n");
        dbgOutputLoc(DBG_READTHREAD_READ_FROM_MQTT);
        getStatusofConfiguration( &status );


        //If the rover has stopped moving, we should send the data being seen by the camera
        //to the robot arm control thread so it can pick up
        /*if( strcmp(status.rover_Status, "STOPPED") == 0 && sent == false)
        {
            //UART_PRINT("SENDING DATA! \r\n");
            sending_data.object_color = status.color;
            sending_data.angle_Base = atoi(status.position);
            sending_data.timer_expired = true;

            dbgOutputLoc(DBG_READTHREAD_SEND_DATA);
            sendToReadDataQueue(sending_data);
            sent = true;
        }

        else if(strcmp(status.rover_Status, "MOVING") == 0 && sent == true)
        {
            sent = false;
        }*/

        sending_data.object_color = status.color;
        sending_data.angle_Base = atoi(status.position);
        sending_data.rover_Status = status.rover_Status;
        //sending_data.timer_expired = TRUE_VAL;

        dbgOutputLoc(DBG_READTHREAD_SEND_DATA);
        sendToReadDataQueue(sending_data);

        dbgOutputLoc(DBG_READTHREAD_EXIT);
    }

}

//void readFromMQTT(char* position, char* color, char* rover_status)
//{
//    Task_Status    data;
//
//    data.color = color;
//    data.position = position;
//    data.rover_Status = rover_status;
//
//    setStatusofConfiguration(data);
//}

void pickUp(Motor_Angles data, int * robot_arm_status)
{
    static int firstRead = TRUE_VAL;
    static int firstRun = TRUE_VAL;
    static int servo_config = TRUE_VAL;
    static int claw_pickup = FALSE_VAL;
    static int post_claw_pickup = FALSE_VAL;
    static int holding_mode = FALSE_VAL;

    static int status0;
    static int status1;
    static int status2;
    static int status3;

    static int baseAngle;
    static int extendAngle;
    static int liftAngle;
    static int clawAngle;

    dbgOutputLoc(DBG_PICKUP_ENTER);

    if(checkIfTimerActive() == pdFALSE)
    {
        //printString("TIMMMMMMMMMERRRRRRRRRRRRRRRRRRRRRRRRRRR EXXXXXXXXPPPPPPPIRRRRRRRRRRRRRREEEEEEEEDDDDDDDDDDDDD PPPPIICCCCCCCCCCCKKKKKKKUUUUUPPPPPPP!!!!!!!!!!!!!!");
        if(firstRead == TRUE_VAL)
        {
            //Read the angles once for servo configuration

            //dbgOutputLoc(DBG_MAINTHREAD_READ_ANGLES);
            baseAngle = data.angle_Base;
            extendAngle = 50; //Might need to change to be read in from the camera
            liftAngle = 0;
            clawAngle = 60;

            firstRead = FALSE_VAL;
            firstRun = TRUE_VAL;

            servo_config = TRUE_VAL;
            claw_pickup = FALSE_VAL;
            post_claw_pickup = FALSE_VAL;
            holding_mode = FALSE_VAL;

            status0 = CONTINUING_CONFIG;
            status1 = CONTINUING_CONFIG;
            status2 = CONTINUING_CONFIG;
            status3 = CONTINUING_CONFIG;
        }

        //increment/ decrement the duty cycle/ degree of the pwm by 1
        //PWM formulas for each motor will return a completion status on it's progress to configuration.
        status0 = setPWM_Base(baseAngle);
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_EXIT_BASE);

        status2 = setPWM_Lift(liftAngle);
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_EXIT_LIFT);

        status1 = setPWM_Extend(extendAngle);
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_EXIT_EXTEND);

        status3 = setPWM_Claw(clawAngle);
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_EXIT_CLAW);


        if(servo_config == TRUE_VAL && claw_pickup == FALSE_VAL && post_claw_pickup == FALSE_VAL && holding_mode == FALSE_VAL)
        {
            dbgOutputLoc(DBG_PICKUP_SERVO_CONFIG_ENTER);
            //Now that all the configurations are done, we should use the claw to
            //pick up the object
            if(status0 == CONFIG_COMPLETE && status1 == CONFIG_COMPLETE && status2 == CONFIG_COMPLETE && status3 == CONFIG_COMPLETE)
            {
                //printString("MOVINNNNNNNNNNNNNGGGGGG TOOOOOOOO CLAWWWWWWW PPPIIIICCCCCCCCCCKKKKKUUUUPPPPPPPPP!!!!");
                dbgOutputLoc(DBG_PICKUP_SERVO_CONFIG_EXIT);

                servo_config = FALSE_VAL;
                claw_pickup = TRUE_VAL;

                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
            }

            //We must continue to increment the PWMs
            firstRun = TRUE_VAL;
            firstRead = FALSE_VAL;
        }

        else if(servo_config == FALSE_VAL && claw_pickup == TRUE_VAL && post_claw_pickup == FALSE_VAL && holding_mode == FALSE_VAL)
        {
            dbgOutputLoc(DBG_PICKUP_CLAW_PICKUP_ENTER);

            static bool temp = false;
            if(temp == false)
            {
                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
                temp = true;
            }
            extendAngle = 120;

            if(status0 == CONFIG_COMPLETE && status1 == CONFIG_COMPLETE && status2 == CONFIG_COMPLETE && status3 == CONFIG_COMPLETE)
            {
                //printString("MOVINNNNNNNNNNNNNGGGGGG TOOOOOOOO HHHHHOOOOOOOOOOOOOOOOOOLLLLLLLLLLDDDDDDDIIIINNNNGGGGGGGG!!!!");

                dbgOutputLoc(DBG_PICKUP_CLAW_PICKUP_EXIT);

                claw_pickup = FALSE_VAL;
                post_claw_pickup = TRUE_VAL;

                temp = false;

                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
            }

            //We must continue to increment the PWMs
            firstRun = TRUE_VAL;
            firstRead = FALSE_VAL;
        }

        else if(servo_config == FALSE_VAL && claw_pickup == FALSE_VAL && post_claw_pickup == TRUE_VAL && holding_mode == FALSE_VAL)
        {
            static bool temp = false;
            if(temp == false)
            {
                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
                temp = true;
            }

            clawAngle = 0;
            //extendAngle = 50;

            if(status0 == CONFIG_COMPLETE && status1 == CONFIG_COMPLETE && status2 == CONFIG_COMPLETE && status3 == CONFIG_COMPLETE)
            {
                //printString("GGGGGGGGOOOOOOOOOIIIIIIIIIIIINNNNNGGGGGGGG TTTOOOOOOOO IIIIIIIIIIDDDDDDDDDDDDLLLLLLLLLLLEEEEEEEEE");


                post_claw_pickup = FALSE_VAL;
                holding_mode = TRUE_VAL;

                temp = false;

                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
            }

            //We must continue to increment the PWMs
            firstRun = TRUE_VAL;
            firstRead = FALSE_VAL;
        }

        else if(servo_config == FALSE_VAL && claw_pickup == FALSE_VAL && post_claw_pickup == FALSE_VAL && holding_mode == TRUE_VAL)
        {
            static bool temp = false;
            if(temp == false)
            {
                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
                temp = true;
            }

            dbgOutputLoc(DBG_PICKUP_HOLDING_MODE_ENTER);
            //baseAngle = 90;
            extendAngle = 50;
            //liftAngle = 40;

            //We're done configuring the PWMs if this if statement is true
            if(status0 == CONFIG_COMPLETE && status1 == CONFIG_COMPLETE && status2 == CONFIG_COMPLETE && status3 == CONFIG_COMPLETE)
            {
                dbgOutputLoc(DBG_PICKUP_HOLDING_MODE_EXIT);

    //            if( strcmp(data.object_color, "DARK") != 0  && strcmp(data.object_color, "LIGHT") != 0 )
    //            {
    //                //This means we successfully picked up the object out of the camera's way
    //                firstRun = FALSE_VAL;
    //                firstRead = TRUE_VAL;
    //                *robot_arm_status = DROP_OFF_MODE;
    //            }
    //
    //            else //If we failed to pick up the object and the camera still sees it
    //            {
    //                servo_config = TRUE_VAL;
    //                claw_pickup = FALSE_VAL;
    //                holding_mode = FALSE_VAL;
    //
    //                status0 = CONTINUING_CONFIG;
    //                status1 = CONTINUING_CONFIG;
    //                status2 = CONTINUING_CONFIG;
    //                status3 = CONTINUING_CONFIG;
    //
    //                firstRun = FALSE_VAL;
    //                firstRead = TRUE_VAL;
    //                *robot_arm_status = PICK_UP_MODE;
    //            }
                //printString("REAAAAAAAAAAAAAAAAADDDDDDDDDDDDDDDDDDDDYYYYYYYYYYYYYYYY FFFFFFOOOOOOOORRRRRRRRRR DDDDDDDRRRRRRRRRRROOOOOOOOPPPPPPPPPPPPPPOOOOOOOFFFFFFFFFFF!!!!");
                temp = false;
                firstRun = FALSE_VAL;
                firstRead = TRUE_VAL;
                *robot_arm_status = DROP_OFF_MODE; //We will let the robot arm thread know
                //It is ready for DROP_OFF_MODE Next time it stops.

                //We should publish to the server that it's okay for the rover to start moving

            }

            else
            {
                //We must continue to increment the PWMs
                firstRun = TRUE_VAL;
                firstRead = FALSE_VAL;
            }
        }

        if(firstRun == TRUE_VAL)
        {
            startSoftwareTimer();
        }
    }

    dbgOutputLoc(DBG_PICKUP_EXIT);
}

void dropOff(int * robot_arm_status, char* object_color)
{
    static int firstRead = TRUE_VAL;
    static int firstRun = TRUE_VAL;
    static int setup_dropoff = TRUE_VAL;
    static int claw_dropoff = FALSE_VAL;
    static int post_claw_dropoff = FALSE_VAL;
    static int idle_mode = FALSE_VAL;

    static int status0 = CONTINUING_CONFIG;
    static int status1 = CONTINUING_CONFIG;
    static int status2 = CONTINUING_CONFIG;
    static int status3 = CONTINUING_CONFIG;

    static int baseAngle;
    static int extendAngle;
    static int liftAngle;
    static int clawAngle;

    dbgOutputLoc(DBG_DROPOFF_ENTER);
    if(checkIfTimerActive() == pdFALSE)
    {
        //printString("TTTTTTTTTTIIIIIIIIIIMMMMMMMMMEEEEEEEERRRRRRRRRRRR EEEEEEEEEXXXXXPPPPPPPPPPPPIIIIIIRRRRRRREEEEEEEEEDDDDDDDDD DDDDRRRRRRRRRRRROOOPPPPPPPPPOOOOFFFFFFFFFFFFF!!!");
        if(firstRead == TRUE_VAL)
        {
            if(strcmp(object_color, "DARK") == 0)
            {
                baseAngle = 0; //Deposit dark object to the bin on the right
            }

            else if(strcmp(object_color, "LIGHT") == 0)
            {
                baseAngle = 170; // Deposit light object to the bin on the left
            }

            extendAngle = 50; //Might need to change to be read in from the camera
            liftAngle = 0;
            clawAngle = 0; //It should already tightly closed

            firstRead = FALSE_VAL;
            firstRun = TRUE_VAL;

            setup_dropoff = TRUE_VAL;
            claw_dropoff = FALSE_VAL;
            post_claw_dropoff = FALSE_VAL;
            idle_mode = FALSE_VAL;

            status0 = CONTINUING_CONFIG;
            status1 = CONTINUING_CONFIG;
            status2 = CONTINUING_CONFIG;
            status3 = CONTINUING_CONFIG;

        }

        //increment/ decrement the duty cycle/ degree of the pwm by 1
        //PWM formulas for each motor will return a completion status on it's progress to configuration.
        status0 = setPWM_Base(baseAngle);
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_EXIT_BASE);

        status2 = setPWM_Lift(liftAngle);
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_EXIT_LIFT);

        status1 = setPWM_Extend(extendAngle);
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_EXIT_EXTEND);

        status3 = setPWM_Claw(clawAngle);
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_EXIT_CLAW);


        if(setup_dropoff == TRUE_VAL && claw_dropoff == FALSE_VAL && post_claw_dropoff == FALSE_VAL && idle_mode == FALSE_VAL)
        {
            dbgOutputLoc(DBG_DROPOFF_SETUP_DROPOFF_ENTER);
            //Now that all the configurations are done, we should use the claw to
            //drop off the object
            if(status0 == CONFIG_COMPLETE && status1 == CONFIG_COMPLETE && status2 == CONFIG_COMPLETE && status3 == CONFIG_COMPLETE)
            {
                //printString("GGGGGGOOOOOOOOOOOIIIIIIIIIIIIIINNNNNNNNNNNNGGGGGGGGGGGG TTTTTTOOOOOOOO CLLLLLLAAAAAAWWWWWWW DDDDDDDDDRRROOOOOPPPPPPPPPPPPOOOOOOOOOOFFFFFFFFFFFF");

                dbgOutputLoc(DBG_DROPOFF_SETUP_DROPOFF_EXIT);

                setup_dropoff = FALSE_VAL;
                claw_dropoff = TRUE_VAL;

                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
            }

            //We must continue to increment the PWMs
            firstRun = TRUE_VAL;
            firstRead = FALSE_VAL;
        }

        else if(setup_dropoff == FALSE_VAL && claw_dropoff == TRUE_VAL && post_claw_dropoff == FALSE_VAL && idle_mode == FALSE_VAL)
        {
            dbgOutputLoc(DBG_DROPOFF_CLAW_DROPOFF_ENTER);
            static bool temp = false;
            if(temp == false)
            {
                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;

                temp = true;
            }

            extendAngle = 120;
            //clawAngle = 60;

            if(status0 == CONFIG_COMPLETE && status1 == CONFIG_COMPLETE && status2 == CONFIG_COMPLETE && status3 == CONFIG_COMPLETE)
            {
                //printString("GGGGGGGGOOOOOOOOOIIIIIIIIIIIINNNNNGGGGGGGG TTTOOOOOOOO IIIIIIIIIIDDDDDDDDDDDDLLLLLLLLLLLEEEEEEEEE");

                dbgOutputLoc(DBG_DROPOFF_CLAW_DROPOFF_EXIT);

                claw_dropoff = FALSE_VAL;
                post_claw_dropoff = TRUE_VAL;

                temp = false;
                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
            }

            //We must continue to increment the PWMs
            firstRun = TRUE_VAL;
            firstRead = FALSE_VAL;
        }

        else if(setup_dropoff == FALSE_VAL && claw_dropoff == FALSE_VAL && post_claw_dropoff == TRUE_VAL && idle_mode == FALSE_VAL)
        {
            static bool temp = false;
            if(temp == false)
            {
                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
                temp = true;
            }

            clawAngle = 60;
            //extendAngle = 70;

            if(status0 == CONFIG_COMPLETE && status1 == CONFIG_COMPLETE && status2 == CONFIG_COMPLETE && status3 == CONFIG_COMPLETE)
            {
                //printString("GGGGGGGGOOOOOOOOOIIIIIIIIIIIINNNNNGGGGGGGG TTTOOOOOOOO IIIIIIIIIIDDDDDDDDDDDDLLLLLLLLLLLEEEEEEEEE");


                post_claw_dropoff = FALSE_VAL;
                idle_mode = TRUE_VAL;

                temp = false;

                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
            }

            //We must continue to increment the PWMs
            firstRun = TRUE_VAL;
            firstRead = FALSE_VAL;
        }

        else if(setup_dropoff == FALSE_VAL && claw_dropoff == FALSE_VAL && post_claw_dropoff == FALSE_VAL && idle_mode == TRUE_VAL)
        {
            dbgOutputLoc(DBG_DROPOFF_IDLE_MODE_ENTER);
            static bool temp = false;
            if(temp == false)
            {
                status0 = CONTINUING_CONFIG;
                status1 = CONTINUING_CONFIG;
                status2 = CONTINUING_CONFIG;
                status3 = CONTINUING_CONFIG;
                temp = true;
            }
            //baseAngle = 90;
            clawAngle = 60;
            extendAngle = 50;
            //liftAngle = 40;

            //We're done configuring the PWMs if this if statement is true
            if(status0 == CONFIG_COMPLETE && status1 == CONFIG_COMPLETE && status2 == CONFIG_COMPLETE && status3 == CONFIG_COMPLETE)
            {
                //printString("RRRRRRRRRRRREEEEEEEEEEEEAAAAAAAAAAAAAADDDDDDDDDDDDDYYYYYYYYYYYYYY FFFFFFFFFFFFOOOOOORRRRRRR PPPIIIIIICCCCKKKKKKKUUUUPPPPPPPPPPP!!");

                dbgOutputLoc(DBG_DROPOFF_IDLE_MODE_EXIT);

                temp = false;
                firstRun = FALSE_VAL;
                firstRead = TRUE_VAL;
                *robot_arm_status = PICK_UP_MODE; //We will let the robot arm thread know
                //It is ready for PICK_UP_MODE Next time it stops.

                //We should publish to the server that it's okay for the rover to start moving

            }

            else
            {
                //We must continue to increment the PWMs
                firstRun = TRUE_VAL;
                firstRead = FALSE_VAL;
            }
        }


        if(firstRun == TRUE_VAL)
        {
            startSoftwareTimer();
        }

    }

    dbgOutputLoc(DBG_DROPOFF_EXIT);
}

//*****************************************************************************
//
//! This function connect the MQTT device to an AP with the SSID which was
//! configured in SSID_NAME definition which can be found in Network_if.h file,
//! if the device can't connect to to this AP a request from the user for other
//! SSID will appear.
//!
//! \param  none
//!
//! \return NoneM
//!
//*****************************************************************************
int32_t Mqtt_IF_Connect()
{
    int32_t lRetVal;
    char SSID_Remote_Name[32];
    int8_t Str_Length;

    memset(SSID_Remote_Name, '\0', sizeof(SSID_Remote_Name));
    Str_Length = strlen(SSID_NAME);

    if(Str_Length)
    {
        /*Copy the Default SSID to the local variable                        */
        strncpy(SSID_Remote_Name, SSID_NAME, Str_Length);
    }

   /*Display Application Banner                                             */
    DisplayBanner(APPLICATION_NAME);

    /*Reset The state of the machine                                         */
    Network_IF_ResetMCUStateMachine();

    /*Start the driver                                                       */
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to start SimpleLink Device\n\r", lRetVal);
        return(-1);
    }

    /*Initialize AP security params                                          */
    SecurityParams.Key = (signed char *) SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    /*Connect to the Access Point                                            */
    lRetVal = Network_IF_ConnectAP(SSID_Remote_Name, SecurityParams);
    if(lRetVal < 0)
    {
        UART_PRINT("Connection to an AP failed\n\r");
        return(-1);
    }

    return(0);
}

//*****************************************************************************
//!
//! MQTT Start - Initialize and create all the items required to run the MQTT
//! protocol
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void Mqtt_start()
{
    int32_t threadArg = 100;
    int32_t threadArg2 = 100;
    int32_t threadArg3 = 100;
    int32_t threadArg4 = 100;

    pthread_t receiveThread;
    pthread_t robotThread;
    pthread_t readThread;

    pthread_attr_t pAttrs;
    pthread_attr_t pAttrs2;
    pthread_attr_t pAttrs3;
    pthread_attr_t pAttrs4;

    struct sched_param priParam;
    struct sched_param priParam2;
    struct sched_param priParam3;
    struct sched_param priParam4;

    int32_t retc = 0;
    int32_t retc1 = 0;
    int32_t retc_2 = 0;
    int32_t retc_3 = 0;

    mq_attr attr;
    mq_attr attr2;


    unsigned mode = 0;
    static bool needToConfigTimer = true;

    if(needToConfigTimer)
    {
        PWM_init();
        createQueue();
        configurePWM();
        configureSTimer();
    }

    /*sync object for inter thread communication                             */
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = sizeof(struct msgQueue);

    attr2.mq_maxmsg = 10;
    attr2.mq_msgsize = sizeof(struct msgQueue);

    g_PBQueue = mq_open("g_PBQueue", O_CREAT, mode, &attr);
    g_JBQueue = mq_open("g_JBQueue", O_CREAT, mode, &attr2);

    if(((int)g_PBQueue) <= 0)
    {
        UART_PRINT("MQTT Message Queue create fail\n\r");
        gInitState &= ~MQTT_INIT_STATE;
        return;
    }

    if(((int)g_JBQueue) <= 0)
    {
        UART_PRINT("MQTT Message Queue create fail\n\r");
        gInitState &= ~MQTT_INIT_STATE;
        return;
    }

    /*Set priority and stack size attributes                                 */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 2;
    retc = pthread_attr_setschedparam(&pAttrs, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs, MQTTTHREADSIZE);
    retc |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);

    if(retc != 0)
    {
        gInitState &= ~MQTT_INIT_STATE;
        UART_PRINT("MQTT thread create fail\n\r");
        return;
    }

    retc = pthread_create(&mqttThread, &pAttrs, MqttClient, (void *) &threadArg);
    if(retc != 0)
    {
        gInitState &= ~MQTT_INIT_STATE;
        UART_PRINT("MQTT thread create fail a\n\r");
        return;
    }

    pthread_attr_init(&pAttrs2);
    priParam2.sched_priority = 2;
    retc1 = pthread_attr_setschedparam(&pAttrs2, &priParam2);
    retc1 |= pthread_attr_setstacksize(&pAttrs2, RTHREADSIZE);
    retc1 |= pthread_attr_setdetachstate(&pAttrs2, PTHREAD_CREATE_DETACHED);


    retc1 = pthread_create(&receiveThread, &pAttrs2, ReceiveClient, (void *) &threadArg2);
    if(retc1 != 0)
    {
        UART_PRINT("MQTT thread create fail 2\n\r");
        return;
    }

    pthread_attr_init(&pAttrs3);
    priParam3.sched_priority = 2;
    retc_2 = pthread_attr_setschedparam(&pAttrs3, &priParam3);
    retc_2 |= pthread_attr_setstacksize(&pAttrs3, THREADSTACKSIZE2);
    retc_2 |= pthread_attr_setdetachstate(&pAttrs3, PTHREAD_CREATE_DETACHED);


    retc_2 = pthread_create(&robotThread, &pAttrs3, robotArmThread, (void *) &threadArg3);
    if(retc_2 != 0)
    {
        UART_PRINT("Robot arm thread create failed \n\r");
       return;
    }

    pthread_attr_init(&pAttrs4);
    priParam4.sched_priority = 2;
    retc_3 = pthread_attr_setschedparam(&pAttrs4, &priParam4);
    retc_3 |= pthread_attr_setstacksize(&pAttrs4, THREADSTACKSIZE3);
    retc_3 |= pthread_attr_setdetachstate(&pAttrs4, PTHREAD_CREATE_DETACHED);


    retc_3 = pthread_create(&readThread, &pAttrs4, readDataThread, (void *) &threadArg4);
    if(retc_3 != 0)
    {
       UART_PRINT("Read data thread create failed \n\r");
       return;
    }

    if(needToConfigTimer)
    {
        configureDataTimer();
        needToConfigTimer = false;
    }

    gInitState &= ~MQTT_INIT_STATE;
}

//*****************************************************************************
//!
//! MQTT Stop - Close the client instance and free all the items required to
//! run the MQTT protocol
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************

void Mqtt_Stop()
{
    struct msgQueue queueElement;
    struct msgQueue queueElemRecv;

    if(gApConnectionState >= 0)
    {
        Mqtt_ClientStop(1);
    }

    queueElement.event = THREAD_TERMINATE_REQ;
    queueElement.msgPtr = NULL;

    /*write message indicating publish message                               */
    if(MQTT_SendMsgToQueue(&queueElement))
    {
        UART_PRINT(
            "\n\n\rQueue is full, throw first msg and send the new one\n\n\r");
        mq_receive(g_PBQueue, (char*) &queueElemRecv, sizeof(struct msgQueue),
                   NULL);
        MQTT_SendMsgToQueue(&queueElement);
    }

//    sleep(2);

    mq_close(g_PBQueue);
    mq_close(g_JBQueue);
    g_PBQueue = NULL;
    g_JBQueue = NULL;

    sl_Stop(SL_STOP_TIMEOUT);
    UART_PRINT("\n\r Client Stop completed\r\n");
}

int32_t MqttClient_start()
{
    int32_t lRetVal = -1;
    int32_t iCount = 0;

    int32_t threadArg = 100;
    pthread_attr_t pAttrs;
    struct sched_param priParam;

    MqttClientExmple_params.clientId = ClientId;
    MqttClientExmple_params.connParams = &Mqtt_ClientCtx;
    MqttClientExmple_params.mqttMode31 = MQTT_3_1;
    MqttClientExmple_params.blockingSend = true;

    gInitState |= CLIENT_INIT_STATE;

    /*Initialize MQTT client lib                                             */
    gMqttClient = MQTTClient_create(MqttClientCallback,
                                    &MqttClientExmple_params);
    if(gMqttClient == NULL)
    {
        /*lib initialization failed                                          */
        gInitState &= ~CLIENT_INIT_STATE;
        return(-1);
    }

    /*Open Client Receive Thread start the receive task. Set priority and    */
    /*stack size attributes                                                  */
    pthread_attr_init(&pAttrs);
    priParam.sched_priority = 2;
    lRetVal = pthread_attr_setschedparam(&pAttrs, &priParam);
    lRetVal |= pthread_attr_setstacksize(&pAttrs, RXTASKSIZE);
    lRetVal |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);
    lRetVal |=
        pthread_create(&g_rx_task_hndl, &pAttrs, MqttClientThread,
                       (void *) &threadArg);
    if(lRetVal != 0)
    {
        UART_PRINT("Client Thread Create Failed failed\n\r");
        gInitState &= ~CLIENT_INIT_STATE;
        return(-1);
    }
#ifdef SECURE_CLIENT
    setTime();
#endif

    /*setting will parameters                                                */
    MQTTClient_set(gMqttClient, MQTTClient_WILL_PARAM, &will_param,
                   sizeof(will_param));

#ifdef CLNT_USR_PWD
    /*Set user name for client connection                                    */
    MQTTClient_set(gMqttClient, MQTTClient_USER_NAME, (void *)ClientUsername,
                   strlen(
                       (char*)ClientUsername));

    /*Set password                                                           */
    MQTTClient_set(gMqttClient, MQTTClient_PASSWORD, (void *)ClientPassword,
                   strlen(
                       (char*)ClientPassword));
#endif
    /*Initiate MQTT Connect                                                  */
    if(gApConnectionState >= 0)
    {
#if CLEAN_SESSION == false
        bool clean = CLEAN_SESSION;
        MQTTClient_set(gMqttClient, MQTTClient_CLEAN_CONNECT, (void *)&clean,
                       sizeof(bool));
#endif
        /*The return code of MQTTClient_connect is the ConnACK value that
           returns from the server */
        lRetVal = MQTTClient_connect(gMqttClient);

        /*negative lRetVal means error,
           0 means connection successful without session stored by the server,
           greater than 0 means successful connection with session stored by
           the server */
        if(0 > lRetVal)
        {
            /*lib initialization failed                                      */
            UART_PRINT("Connection to broker failed, Error code: %d\n\r",
                       lRetVal);

            gUiConnFlag = 0;
        }
        else
        {
            gUiConnFlag = 1;
        }
        /*Subscribe to topics when session is not stored by the server       */
        if((gUiConnFlag == 1) && (0 == lRetVal))
        {
            uint8_t subIndex;
            MQTTClient_SubscribeParams subscriptionInfo[
                SUBSCRIPTION_TOPIC_COUNT];

            for(subIndex = 0; subIndex < SUBSCRIPTION_TOPIC_COUNT; subIndex++)
            {
                subscriptionInfo[subIndex].topic = topic[subIndex];
                subscriptionInfo[subIndex].qos = qos[subIndex];
            }

            if(MQTTClient_subscribe(gMqttClient, subscriptionInfo,
                                    SUBSCRIPTION_TOPIC_COUNT) < 0)
            {
                UART_PRINT("\n\r Subscription Error \n\r");
                MQTTClient_disconnect(gMqttClient);
                gUiConnFlag = 0;
            }
            else
            {
                for(iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
                {
                    UART_PRINT("Client subscribed on %s\n\r,", topic[iCount]);
                }
            }
        }
    }

    gInitState &= ~CLIENT_INIT_STATE;

    return(0);
}

//*****************************************************************************
//!
//! MQTT Client stop - Unsubscribe from the subscription topics and exit the
//! MQTT client lib.
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************

void Mqtt_ClientStop(uint8_t disconnect)
{
    uint32_t iCount;

    MQTTClient_UnsubscribeParams subscriptionInfo[SUBSCRIPTION_TOPIC_COUNT];

    for(iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
    {
        subscriptionInfo[iCount].topic = topic[iCount];
    }

    MQTTClient_unsubscribe(gMqttClient, subscriptionInfo,
                           SUBSCRIPTION_TOPIC_COUNT);
    for(iCount = 0; iCount < SUBSCRIPTION_TOPIC_COUNT; iCount++)
    {
        UART_PRINT("Unsubscribed from the topic %s\r\n", topic[iCount]);
    }
    gUiConnFlag = 0;

    /*exiting the Client library                                             */
    MQTTClient_delete(gMqttClient);
}

//*****************************************************************************
//!
//! Utility function which prints the borders
//!
//! \param[in] ch  -  hold the charater for the border.
//! \param[in] n   -  hold the size of the border.
//!
//! \return none.
//!
//*****************************************************************************

void printBorder(char ch,
                 int n)
{
    int i = 0;

    for(i = 0; i < n; i++)
    {
        putch(ch);
    }
}

//*****************************************************************************
//!
//! Set the ClientId with its own mac address
//! This routine converts the mac address which is given
//! by an integer type variable in hexadecimal base to ASCII
//! representation, and copies it into the ClientId parameter.
//!
//! \param  macAddress  -   Points to string Hex.
//!
//! \return void.
//!
//*****************************************************************************
int32_t SetClientIdNamefromMacAddress()
{
    int32_t ret = 0;
    uint8_t Client_Mac_Name[2];
    uint8_t Index;
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint8_t macAddress[SL_MAC_ADDR_LEN];

    /*Get the device Mac address */
    ret = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                       &macAddress[0]);

    /*When ClientID isn't set, use the mac address as ClientID               */
    if(ClientId[0] == '\0')
    {
        /*6 bytes is the length of the mac address                           */
        for(Index = 0; Index < SL_MAC_ADDR_LEN; Index++)
        {
            /*Each mac address byte contains two hexadecimal characters      */
            /*Copy the 4 MSB - the most significant character                */
            Client_Mac_Name[0] = (macAddress[Index] >> 4) & 0xf;
            /*Copy the 4 LSB - the least significant character               */
            Client_Mac_Name[1] = macAddress[Index] & 0xf;

            if(Client_Mac_Name[0] > 9)
            {
                /*Converts and copies from number that is greater than 9 in  */
                /*hexadecimal representation (a to f) into ascii character   */
                ClientId[2 * Index] = Client_Mac_Name[0] + 'a' - 10;
            }
            else
            {
                /*Converts and copies from number 0 - 9 in hexadecimal       */
                /*representation into ascii character                        */
                ClientId[2 * Index] = Client_Mac_Name[0] + '0';
            }
            if(Client_Mac_Name[1] > 9)
            {
                /*Converts and copies from number that is greater than 9 in  */
                /*hexadecimal representation (a to f) into ascii character   */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + 'a' - 10;
            }
            else
            {
                /*Converts and copies from number 0 - 9 in hexadecimal       */
                /*representation into ascii character                        */
                ClientId[2 * Index + 1] = Client_Mac_Name[1] + '0';
            }
        }
    }
    return(ret);
}

//*****************************************************************************
//!
//! Utility function which Display the app banner
//!
//! \param[in] appName     -  holds the application name.
//! \param[in] appVersion  -  holds the application version.
//!
//! \return none.
//!
//*****************************************************************************

int32_t DisplayAppBanner(char* appName,
                         char* appVersion)
{
    int32_t ret = 0;
    uint8_t macAddress[SL_MAC_ADDR_LEN];
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint16_t ConfigSize = 0;
    uint8_t ConfigOpt = SL_DEVICE_GENERAL_VERSION;
    SlDeviceVersion_t ver = {0};

    ConfigSize = sizeof(SlDeviceVersion_t);

    /*Print device version info. */
    ret =
        sl_DeviceGet(SL_DEVICE_GENERAL, &ConfigOpt, &ConfigSize,
                     (uint8_t*)(&ver));

    /*Print device Mac address */
    ret |= (int32_t)sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                       &macAddress[0]);

    UART_PRINT(lineBreak);
    UART_PRINT("\t");
    printBorder('=', 44);
    UART_PRINT(lineBreak);
    UART_PRINT("\t   %s Example Ver: %s",appName, appVersion);
    UART_PRINT(lineBreak);
    UART_PRINT("\t");
    printBorder('=', 44);
    UART_PRINT(lineBreak);
    UART_PRINT(lineBreak);
    UART_PRINT("\t CHIP: 0x%x",ver.ChipId);
    UART_PRINT(lineBreak);
    UART_PRINT("\t MAC:  %d.%d.%d.%d",ver.FwVersion[0],ver.FwVersion[1],
               ver.FwVersion[2],
               ver.FwVersion[3]);
    UART_PRINT(lineBreak);
    UART_PRINT("\t PHY:  %d.%d.%d.%d",ver.PhyVersion[0],ver.PhyVersion[1],
               ver.PhyVersion[2],
               ver.PhyVersion[3]);
    UART_PRINT(lineBreak);
    UART_PRINT("\t NWP:  %d.%d.%d.%d",ver.NwpVersion[0],ver.NwpVersion[1],
               ver.NwpVersion[2],
               ver.NwpVersion[3]);
    UART_PRINT(lineBreak);
    UART_PRINT("\t ROM:  %d",ver.RomVersion);
    UART_PRINT(lineBreak);
    UART_PRINT("\t HOST: %s", SL_DRIVER_VERSION);
    UART_PRINT(lineBreak);
    UART_PRINT("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x", macAddress[0],
               macAddress[1], macAddress[2], macAddress[3], macAddress[4],
               macAddress[5]);
    UART_PRINT(lineBreak);
    UART_PRINT(lineBreak);
    UART_PRINT("\t");
    printBorder('=', 44);
    UART_PRINT(lineBreak);
    UART_PRINT(lineBreak);

    return(ret);
}

void mainThread(void * args)
{
    pthread_t spawn_thread = (pthread_t) NULL;
    pthread_attr_t pAttrs_spawn;
    struct sched_param priParam;
    int32_t retc = 0;
    UART_Handle tUartHndl;

    /*Initialize SlNetSock layer with CC31xx/CC32xx interface */
    retc = ti_net_SlNet_initConfig();
    if(0 != retc)
    {
        UART_PRINT("Failed to initialize SlNetSock\n\r");
    }


    tUartHndl = InitTerm();
    /*remove uart receive from LPDS dependency                               */
    UART_control(tUartHndl, UART_CMD_RXDISABLE, NULL);

    /*Create the sl_Task                                                     */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = SPAWN_TASK_PRIORITY;
    retc = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs_spawn, TASKSTACKSIZE);
    retc |= pthread_attr_setdetachstate
                                    (&pAttrs_spawn, PTHREAD_CREATE_DETACHED);

    retc = pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);

    if(retc != 0)
    {
        UART_PRINT("could not create simplelink task\n\r");
        while(1)
        {
            ;
        }
    }

    retc = sl_Start(0, 0, 0);
    if(retc < 0)
    {
        /*Handle Error */
        UART_PRINT("\n sl_Start failed\n");
        while(1)
        {
            ;
        }
    }

    /*Output device information to the UART terminal */
    retc = DisplayAppBanner(APPLICATION_NAME, APPLICATION_VERSION);
    /*Set the ClientId with its own mac address */
    retc |= SetClientIdNamefromMacAddress();


    retc = sl_Stop(SL_STOP_TIMEOUT);
    if(retc < 0)
    {
        /*Handle Error */
        UART_PRINT("\n sl_Stop failed\n");
        while(1)
        {
            ;
        }
    }

    if(retc < 0)
    {
        /*Handle Error */
        UART_PRINT("mqtt_client - Unable to retrieve device information \n");
        while(1)
        {
            ;
        }
    }

    while(1)
    {
        gResetApplication = false;
        topic[0] = SUBSCRIPTION_TOPIC0;
        topic[1] = SUBSCRIPTION_TOPIC1;
//        topic[2] = SUBSCRIPTION_TOPIC2;
//        topic[3] = SUBSCRIPTION_TOPIC3;
       // topic[4] = SUBSCRIPTION_TOPIC4;
        gInitState = 0;

        /*Connect to AP                                                      */
        gApConnectionState = Mqtt_IF_Connect();

        gInitState |= MQTT_INIT_STATE;
        /*Run MQTT Main Thread (it will open the Client and Server)          */
        Mqtt_start();
        //Mqtt_start2();

       while(gResetApplication == false)
       {
           vTaskDelay(pdMS_TO_TICKS(1000)) ;
       }

//        /*Stop the MQTT Process                                              */
         Mqtt_Stop();
    }
}
