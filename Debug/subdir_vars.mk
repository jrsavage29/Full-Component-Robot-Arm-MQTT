################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../CC3220SF_LAUNCHXL_FREERTOS.cmd 

SYSCFG_SRCS += \
../common.syscfg 

C_SRCS += \
../client_cbs.c \
./syscfg/ti_net_config.c \
./syscfg/ti_drivers_net_wifi_config.c \
./syscfg/ti_drivers_config.c \
../dataTimer.c \
../debugIntercomponent.c \
../main_freertos.c \
../message_queue.c \
../mqtt_client_app.c \
../network_if.c \
../servo_PWM.c \
../softwareTimer.c \
../uart_term.c 

GEN_FILES += \
./syscfg/ti_net_config.c \
./syscfg/ti_drivers_net_wifi_config.c \
./syscfg/ti_drivers_config.c 

GEN_MISC_DIRS += \
./syscfg/ 

C_DEPS += \
./client_cbs.d \
./syscfg/ti_net_config.d \
./syscfg/ti_drivers_net_wifi_config.d \
./syscfg/ti_drivers_config.d \
./dataTimer.d \
./debugIntercomponent.d \
./main_freertos.d \
./message_queue.d \
./mqtt_client_app.d \
./network_if.d \
./servo_PWM.d \
./softwareTimer.d \
./uart_term.d 

OBJS += \
./client_cbs.obj \
./syscfg/ti_net_config.obj \
./syscfg/ti_drivers_net_wifi_config.obj \
./syscfg/ti_drivers_config.obj \
./dataTimer.obj \
./debugIntercomponent.obj \
./main_freertos.obj \
./message_queue.obj \
./mqtt_client_app.obj \
./network_if.obj \
./servo_PWM.obj \
./softwareTimer.obj \
./uart_term.obj 

GEN_MISC_FILES += \
./syscfg/ti_drivers_config.h \
./syscfg/syscfg_c.rov.xs 

GEN_MISC_DIRS__QUOTED += \
"syscfg\" 

OBJS__QUOTED += \
"client_cbs.obj" \
"syscfg\ti_net_config.obj" \
"syscfg\ti_drivers_net_wifi_config.obj" \
"syscfg\ti_drivers_config.obj" \
"dataTimer.obj" \
"debugIntercomponent.obj" \
"main_freertos.obj" \
"message_queue.obj" \
"mqtt_client_app.obj" \
"network_if.obj" \
"servo_PWM.obj" \
"softwareTimer.obj" \
"uart_term.obj" 

GEN_MISC_FILES__QUOTED += \
"syscfg\ti_drivers_config.h" \
"syscfg\syscfg_c.rov.xs" 

C_DEPS__QUOTED += \
"client_cbs.d" \
"syscfg\ti_net_config.d" \
"syscfg\ti_drivers_net_wifi_config.d" \
"syscfg\ti_drivers_config.d" \
"dataTimer.d" \
"debugIntercomponent.d" \
"main_freertos.d" \
"message_queue.d" \
"mqtt_client_app.d" \
"network_if.d" \
"servo_PWM.d" \
"softwareTimer.d" \
"uart_term.d" 

GEN_FILES__QUOTED += \
"syscfg\ti_net_config.c" \
"syscfg\ti_drivers_net_wifi_config.c" \
"syscfg\ti_drivers_config.c" 

C_SRCS__QUOTED += \
"../client_cbs.c" \
"./syscfg/ti_net_config.c" \
"./syscfg/ti_drivers_net_wifi_config.c" \
"./syscfg/ti_drivers_config.c" \
"../dataTimer.c" \
"../debugIntercomponent.c" \
"../main_freertos.c" \
"../message_queue.c" \
"../mqtt_client_app.c" \
"../network_if.c" \
"../servo_PWM.c" \
"../softwareTimer.c" \
"../uart_term.c" 

SYSCFG_SRCS__QUOTED += \
"../common.syscfg" 


