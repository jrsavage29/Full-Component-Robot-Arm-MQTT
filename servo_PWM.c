
/*
 * timerone.c
 *
 *  Created on: Jun 11, 2020
 *      Author: Kevin Kleinegger
*/

#include <servo_PWM.h>

void configurePWM()
{
    dutyCycle_Base = INITIAL_BASE_DUTY_CYCLE; //increment by 10 us to represent 1 degree angle changes
    dutyCycle_Extend = INITIAL_EXTEND_DUTY_CYCLE;
    dutyCycle_Lift = INITIAL_LIFT_DUTY_CYCLE;
    dutyCycle_Claw = INITIAL_CLAW_DUTY_CYCLE;

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = INITIAL_BASE_DUTY_CYCLE;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = PWM_PERIOD;

    pwm_Base = PWM_open(CONFIG_PWM_0, &params);
    if (pwm_Base == NULL)
    {
        /* CONFIG_PWM_0 did not open */
        while (1);
    }

    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = INITIAL_EXTEND_DUTY_CYCLE;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = PWM_PERIOD;

    pwm_Extend = PWM_open(CONFIG_PWM_1, &params);
    if (pwm_Extend == NULL)
    {
        /* CONFIG_PWM_1 did not open */
        while (1);
    }

    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = INITIAL_LIFT_DUTY_CYCLE;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = PWM_PERIOD;

    pwm_Lift = PWM_open(CONFIG_PWM_2, &params);
    if (pwm_Lift == NULL)
    {
        /* CONFIG_PWM_2 did not open */
        while (1);
    }

    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = INITIAL_CLAW_DUTY_CYCLE;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = PWM_PERIOD;

    pwm_Claw = PWM_open(CONFIG_PWM_3, &params);
    if (pwm_Claw == NULL)
    {
        /* CONFIG_PWM_3 did not open */
        while (1);
    }

    PWM_start(pwm_Base);
    PWM_start(pwm_Extend);
    PWM_start(pwm_Lift);
    PWM_start(pwm_Claw);

}

int setPWM_Base(int angle_Base)
{
    //dbgOutputLoc(DBG_SUBROUTINE_PWM_ENTER_BASE);

    static int target_angle;
    target_angle = 11*angle_Base + DEFAULT_BASE_DUTY_CYCLE;

    //A pulse width of 1500 us moves the servo to angle 0
    if(target_angle != dutyCycle_Base)
    {
        if(angle_Base > 180 || angle_Base < 0)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_BASE);
            return CONFIG_COMPLETE; //This means we don't need to increment or decrement to the desired angle anymore.
        }

        if(target_angle > dutyCycle_Base)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_INCREMENTING_BASE);
            dutyCycle_Base += 11;
            PWM_setDuty(pwm_Base, dutyCycle_Base);
        }

        else if(target_angle < dutyCycle_Base)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_DECREMENTING_BASE);
            dutyCycle_Base -= 11;
            PWM_setDuty(pwm_Base, dutyCycle_Base);
        }

        //dbgOutputLoc(DBG_SUBROUTINE_PWM_CONTINUE_CONFIG_BASE);
        return CONTINUING_CONFIG; //This means we should continue incrementing/ decrementing the servo angle.
    }

    else if(target_angle == dutyCycle_Base)
    {
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_BASE);
        return CONFIG_COMPLETE; //This means we don't need to increment or decrement to the desired angle anymore.
    }

    //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_BASE_ERROR);
    return CONTINUING_CONFIG;

}

int setPWM_Extend(int angle_Extend)
{
    //dbgOutputLoc(DBG_SUBROUTINE_PWM_ENTER_EXTEND);

    static int target_angle;
    target_angle = 11*angle_Extend + DEFAULT_EXTEND_DUTY_CYCLE;

    if(target_angle != dutyCycle_Extend)
    {
        if(angle_Extend > 180 || angle_Extend < 0)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_EXTEND);
            return CONFIG_COMPLETE; //This means we don't need to increment or decrement to the desired angle anymore.
        }

        if(target_angle > dutyCycle_Extend)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_INCREMENTING_EXTEND);
            dutyCycle_Extend = dutyCycle_Extend + 11;
            PWM_setDuty(pwm_Extend, dutyCycle_Extend);
        }

        else if(target_angle < dutyCycle_Extend)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_DECREMENTING_EXTEND);
            dutyCycle_Extend = dutyCycle_Extend - 11;
            PWM_setDuty(pwm_Extend, dutyCycle_Extend);
        }

        //dbgOutputLoc(DBG_SUBROUTINE_PWM_CONTINUE_CONFIG_EXTEND);
        return CONTINUING_CONFIG; //This means we should continue incrementing/ decrementing the servo angle.
    }

    else if(target_angle == dutyCycle_Extend)
    {
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_EXTEND);
        return CONFIG_COMPLETE; //This means we don't need to increment or decrement to the desired angle anymore.
    }

    //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_EXTEND_ERROR);
    return CONTINUING_CONFIG;

}

int setPWM_Lift(int angle_Lift)
{
    //dbgOutputLoc(DBG_SUBROUTINE_PWM_ENTER_LIFT);

    static int target_angle;
    target_angle =  DEFAULT_LIFT_DUTY_CYCLE - 11*angle_Lift;
    //A pulse width of 1500 us moves the servo to angle 0
    if(target_angle != dutyCycle_Lift)
    {
        if(angle_Lift > 180 || angle_Lift < 0)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_LIFT);
            return CONFIG_COMPLETE; //This means we don't need to increment or decrement to the desired angle anymore.
        }

        if(target_angle > dutyCycle_Lift)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_INCREMENTING_LIFT);
            dutyCycle_Lift = dutyCycle_Lift + 11;
            PWM_setDuty(pwm_Lift, dutyCycle_Lift);
        }

        else if(target_angle < dutyCycle_Lift)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_DECREMENTING_LIFT);
            dutyCycle_Lift = dutyCycle_Lift - 11;
            PWM_setDuty(pwm_Lift, dutyCycle_Lift);
        }

        //dbgOutputLoc(DBG_SUBROUTINE_PWM_CONTINUE_CONFIG_LIFT);
        return CONTINUING_CONFIG; //This means we should continue incrementing/ decrementing the servo angle.
    }

    else if(target_angle == dutyCycle_Lift)
    {
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_LIFT);
        return CONFIG_COMPLETE; //This means we don't need to increment or decrement to the desired angle anymore.
    }

    //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_LIFT_ERROR);
    return CONTINUING_CONFIG;

}

int setPWM_Claw(int angle_Claw)
{
    //dbgOutputLoc(DBG_SUBROUTINE_PWM_ENTER_CLAW);

    static int target_angle;
    target_angle = 10*angle_Claw + DEFAULT_CLAW_DUTY_CYCLE;
    //A pulse width of 1500 us moves the servo to angle 0
    if(target_angle != dutyCycle_Claw)
    {
        if(angle_Claw > 180 || angle_Claw < 0)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_CLAW);
            return CONFIG_COMPLETE; //This means we don't need to increment or decrement to the desired angle anymore.
        }

        if(target_angle > dutyCycle_Claw)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_INCREMENTING_CLAW);
            dutyCycle_Claw = dutyCycle_Claw + 10;
            PWM_setDuty(pwm_Claw, dutyCycle_Claw);
        }

        else if(target_angle < dutyCycle_Claw)
        {
            //dbgOutputLoc(DBG_SUBROUTINE_PWM_DECREMENTING_CLAW);
            dutyCycle_Claw = dutyCycle_Claw - 10;
            PWM_setDuty(pwm_Claw, dutyCycle_Claw);
        }

        //dbgOutputLoc(DBG_SUBROUTINE_PWM_CONTINUE_CONFIG_CLAW);
        return CONTINUING_CONFIG; //This means we should continue incrementing/ decrementing the servo angle.
    }

    else if(target_angle == dutyCycle_Claw)
    {
        //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_CLAW);
        return CONFIG_COMPLETE; //This means we don't need to increment or decrement to the desired angle anymore.
    }

    //dbgOutputLoc(DBG_SUBROUTINE_PWM_END_CONFIG_CLAW_ERROR);
    return CONTINUING_CONFIG;
}

void stopPWMs()
{
    PWM_stop(pwm_Base);
    PWM_stop(pwm_Extend);
    PWM_stop(pwm_Lift);
    PWM_stop(pwm_Claw);
}

