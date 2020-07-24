/*
 * timertwo.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Kevin Kleinegger
 */

#include <softwareTimer.h>

void configureSTimer()
{
    xAutoReloadTimer = xTimerCreate(
                      "AutoReload",
                      AUTO_RELOAD_TIMER_PERIOD,
                      pdFALSE,
                      0,
                      softwareTimerCallback );
}

void startSoftwareTimer()
{
    //dbgOutputLoc(DBG_STIMER_START);
    xTimerStart(xAutoReloadTimer, portMAX_DELAY);

}

void restartSoftwareTimer()
{
    xTimerReset(xAutoReloadTimer, portMAX_DELAY);
}

void stopSoftwareTimer()
{
    xTimerStop(xAutoReloadTimer, portMAX_DELAY);
}

BaseType_t checkIfTimerActive()
{

    return xTimerIsTimerActive(xAutoReloadTimer);
}

void softwareTimerCallback(TimerHandle_t xAutoReloadTimer)
{

    //dbgOutputLoc(DBG_STIMER_ENTER_ISR);
    Motor_Angles timer_status_only;
    timer_status_only.timer_expired = TRUE_VAL;
    sendTimerStatusfromISR(timer_status_only);
    //dbgOutputLoc(DBG_STIMER_EXIT_ISR); //0x1F

}

