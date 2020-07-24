
#include <debugIntercomponent.h>

//Written by Surasith Boonaneksap
//Sources
//-Example timerled.c and its associated files (ti_drivers_config.h, GPIO.h, Timer.h).
//-https://en.cppreference.com/w/c/language
//-https://www.geeksforgeeks.org/strings-in-c-2/
//-file:///C:/ti/simplelink_cc32xx_sdk_4_10_00_07/docs/tidrivers/doxygen/html/_u_a_r_t_8h.html#a1036c2d5258ff70e9afe6cbfc326544f
//-file:///C:/ti/simplelink_cc32xx_sdk_4_10_00_07/docs/tidrivers/doxygen/html/_g_p_i_o_8h.html#ti_drivers_GPIO_Examples
//-https://www.freertos.org/a00134.html

void dbgUARTVal(unsigned char outVal)
{
    UART_write(uart0, &outVal, 1);
}

void dbgOutputLoc(unsigned int outLoc)
{
    if (outLoc <= 127)
    {
        GPIO_write(GPIO_PIN_7, ON);
        GPIO_write(GPIO_PIN_0, outLoc & 1);
        GPIO_write(GPIO_PIN_1, outLoc >> 1 & 1);
        GPIO_write(GPIO_PIN_2, outLoc >> 2 & 1);
        GPIO_write(GPIO_PIN_3, outLoc >> 3 & 1);
        GPIO_write(GPIO_PIN_4, outLoc >> 4 & 1);
        GPIO_write(GPIO_PIN_5, outLoc >> 5 & 1);
        GPIO_write(GPIO_PIN_6, outLoc >> 6 & 1);
        GPIO_write(GPIO_PIN_7, OFF);
    }
    else
    {
        dbgStop();
    }
}

void dbgStop()
{
    //display error code
    dbgOutputLoc(DBG_STOP_ALL);
    //stop all tasks
    vTaskSuspendAll();
    //disable interrupts
    taskDISABLE_INTERRUPTS();
    while(1)
    {
    }
}

void dbgInitUART()
{
    UART_init();

    UART_Params params;
    UART_Params_init (&params);
    params.baudRate = UART_BAUD_RATE; //Defined in debug.h
    params.writeDataMode = UART_DATA_BINARY;

    uart0 = UART_open(CONFIG_UART_0, &params);
}
