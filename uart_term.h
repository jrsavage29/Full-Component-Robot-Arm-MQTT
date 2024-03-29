#ifndef __UART_IF_H__
#define __UART_IF_H__

// TI-Driver includes
#include <ti/drivers/UART.h>
#include "ti_drivers_config.h"
#include <string.h>
#include <stdio.h>

//Defines

#define UART_PRINT Report
#define DBG_PRINT  Report
#define ERR_PRINT(x) Report("Error [%d] at line [%d] in function [%s]  \n\r",\
                                                                x, __LINE__, \
                                                                 __FUNCTION__)

/* API */

UART_Handle InitTerm(void);

int Report(const char *pcFormat,
           ...);

int TrimSpace(char * pcInput);

int GetCmd(char *pcBuffer,
           unsigned int uiBufLen);

void Message(const char *str);

void ClearTerm();

char getch(void);

void putch(char ch);

void printString(char * string);
void writeToUART(unsigned char outVal);
void printNum(int32_t num);

#endif // __UART_IF_H__
