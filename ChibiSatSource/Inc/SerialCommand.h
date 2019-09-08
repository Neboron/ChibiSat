
#ifndef __SERIALCOMMAND_H
#define __SERIALCOMMAND_H

#ifdef __cplusplus
extern "C" {
#endif


//INCLUDES:
#include "stm32f1xx_hal.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>


//DEFINES:
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

//VARIABLES:
typedef struct ConsoleCmd_t
{
    const char *cmd;
    int (*doit)(int argc, char **argv);
}   ConsoleCmd_t;

typedef struct Console_t 
{
    //UART_Cfg UartCfg;                 /** Queue processing */
    //UART_HandleTypeDef *UartHal;      /** HAL UART pointer */
    const ConsoleCmd_t *Ops;          /** Supported commands */
    uint16_t OpsLen;                  /** Amount of commands */
}   Console_t;


//FUNCTIONS DECLARATION:
int  ConsoleInit(Console_t *Console, const ConsoleCmd_t *Ops, uint16_t OpsLen);
void SerialSendHandlar(void);
void SerialBufferSend(uint8_t* Buf, uint32_t *Len);
void PrintString(char Buf[]);
void PrintInt(int val);
void PrintFloat(float val);
void SerialParse(uint8_t* Buf, uint32_t *Len);


#ifdef __cplusplus
}
#endif

#endif /* __SERIALCOMMAND */

