#ifndef UART_MESSAGING_H
#define UART_MESSAGING_H

#ifdef __cplusplus
extern "C"{
#endif


/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/

#include "stdint.h"

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
#define UART_Channel 0x00000000
#define UART_BaudRate 921600

/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/

/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/


/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/


/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/


/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

#define ReadUartDataFromAddress(xMonitoredValue_t_Address) (xMonitoredValue_t_Address)->valueUart
#define WriteUartDataAtAddress(data, xMonitoredValue_t_Address) (xMonitoredValue_t_Address)->valueUart = ((data) <= (xMonitoredValue_t_Address)->maxValue) ? (data) : ((xMonitoredValue_t_Address)->maxValue)

void UartMessaging_Init(void);
void UartMessaging_Test(void);
void UartMessaging_Update(void);
void UartMessaging_SetCellVoltage(uint16_t Value, uint16_t index);
void UartMessaging_SetCellVoltageErrors(boolean Value, uint16_t index);
void UartMessaging_SetCellTemperature(uint16_t Value, uint16_t index);
void UartMessaging_SetCellTemperatureErrors(boolean Value, uint16_t index);
uint16_t UartMessaging_ReadCellVoltage(uint16_t index);
boolean UartMessaging_ReadCellVoltageErrors(uint16_t index);
uint16_t UartMessaging_ReadCellTemperature(uint16_t index);
boolean UartMessaging_ReadCellTemperatureErrors(uint16_t index);

#ifdef __cplusplus
}
#endif

#endif
