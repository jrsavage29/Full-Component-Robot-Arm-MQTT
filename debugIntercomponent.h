  /*
 * debug.h
 *
 *  Created on: Jun 14, 2020
 *      Author: suras
 */
#ifndef DEBUGINTERCOMPONENT_H_
#define DEBUGINTERCOMPONENT_H_

#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART.h>
#include <stddef.h>
#include "ti_drivers_config.h"
#include <FreeRTOS.h>
#include <task.h>

#define UART_BAUD_RATE 115200
#define OFF            0
#define ON             1

//Pin definitions
#define GPIO_PIN_0 CONFIG_GPIO_P15_BIT_1
#define GPIO_PIN_1 CONFIG_GPIO_P63_BIT_2
#define GPIO_PIN_2 CONFIG_GPIO_P50_BIT_3
#define GPIO_PIN_3 CONFIG_GPIO_P53_BIT_4
#define GPIO_PIN_4 CONFIG_GPIO_P18_BIT_5
#define GPIO_PIN_5 CONFIG_GPIO_P03_BIT_6
#define GPIO_PIN_6 CONFIG_GPIO_P61_BIT_7
#define GPIO_PIN_7 CONFIG_GPIO_P62_BIT_8

//Software Timer Events
#define DBG_STIMER_START                            0x10
#define DBG_STIMER_ENTER_ISR                        0x11
#define DBG_STIMER_EXIT_ISR                         0x1F

//Robot arm Thread Events
#define DBG_MAINTHREAD_ENTER                        0x20
#define DBG_MAINTHREAD_EXIT                         0x21
#define DBG_MAINTHREAD_DONE_CONFIG                  0x22
#define DBG_MAINTHREAD_KEEP_CONFIG                  0x23
#define DBG_MAINTHREAD_READ_DATA                    0x24
#define DBG_READTHREAD_ENTER                        0x25
#define DBG_READTHREAD_EXIT                         0x26
#define DBG_READTHREAD_SEND_DATA                    0x27
#define DBG_READTHREAD_READ_FROM_MQTT               0x28

//Pickup Events
#define DBG_PICKUP_ENTER                            0x30
#define DBG_PICKUP_EXIT                             0x31
#define DBG_PICKUP_SERVO_CONFIG_ENTER               0x32
#define DBG_PICKUP_SERVO_CONFIG_EXIT                0x33
#define DBG_PICKUP_CLAW_PICKUP_ENTER                0x34
#define DBG_PICKUP_CLAW_PICKUP_EXIT                 0x35
#define DBG_PICKUP_HOLDING_MODE_ENTER               0x36
#define DBG_PICKUP_HOLDING_MODE_EXIT                0x37

//Dropoff Events
#define DBG_DROPOFF_ENTER                           0x60
#define DBG_DROPOFF_EXIT                            0x61
#define DBG_DROPOFF_SETUP_DROPOFF_ENTER             0x62
#define DBG_DROPOFF_SETUP_DROPOFF_EXIT              0x63
#define DBG_DROPOFF_CLAW_DROPOFF_ENTER              0x64
#define DBG_DROPOFF_CLAW_DROPOFF_EXIT               0x65
#define DBG_DROPOFF_IDLE_MODE_ENTER                 0x66
#define DBG_DROPOFF_IDLE_MODE_EXIT                  0x67


#define INSIDE_RECV_CLIENT                          0x42
#define INSIDE_CLIENT_CALLBACK                      0x43
#define INSIDE_MSG_RECV                             0x44
#define INSIDE_RECEIVE_CLIENT                       0x45
#define AFTER_TIMER_CONFIG                          0x46
#define AFTER_MQ_RECEIVE                            0x47
#define INSIDE_MSG_RECV2                            0x48
#define INSIDE_MQTT_CLIENTTHREAD                    0x49

#define INSIDE_MQTT_CLIENT                          0x50
//#define INSIDE_MQTT_CLIENT_CASE 0x51
#define INSIDE_MQTT_CLIENT_PUSH                     0x52
#define INSIDE_MQTT_CLIENT_RECV                     0x53
#define INSIDE_MQTT_CLIENT_DISCONNECTION            0x54
#define INSIDE_MQTT_CLIENT_DISC_BUTTON              0x55
#define AFTER_MQTTCLIENT_DISCONNECT                 0x56
#define EXIT_MQTT_CLIENT_RECV                       0x57
#define NEXT_MQTT_CLIENT_RECV                       0x58
#define AFTER_SEND_MSG_Q                            0x59

#define INSIDE_RECV_CLIENT_FOR                      0x70
#define INSIDE_MQTT_CLIENT_CASE                     0x71
#define ERROR_QUEUE_FULL                            0x72
#define ENTER_TIMER_ISR                             0x73
#define EXIT_TIMER_ISR                              0x74


#define DBG_STOP_ALL                                0x7F

static UART_Handle uart0;

void dbgUARTVal(unsigned char outVal);
void dbgOutputLoc(unsigned int outLoc);
void dbgStop();
void dbgInitUART();


#endif /* DEBUGINTERCOMPONENT_H_ */
