#ifndef CAN_MESSAGING_H
#define CAN_MESSAGING_H

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
#include"Mcu.h"
#include"Can_43_FLEXCAN.h"
#include "CDD_Uart.h"
#include "Messaging.h"
#include "UartMessaging.h"

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
typedef enum{
	CAN_IDLE,
	CAN_TRANSMITTING
}CAN_STATE;
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
extern volatile bool transmission_data_updated;

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/
void Can_Receive_Interrupt_INVERTOR_STANGA(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_INVERTOR_DREAPTA(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_INVERTOARE(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_BORD(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_ACCELERATIE(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_FRANA(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_BATERIE(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_BATERIE_TENSIUNI_CELULE(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_BATERIE_TEMPERATURI_CELULE(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_BATERIE_2(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_BATERIE_CHARGER(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_COMUNICATII(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_LEFT_INVERTER_MSG1(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_RIGHT_INVERTER_MSG1(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_LEFT_INVERTER_MSG2(PduIdType RxPduId, const PduInfoType * PduInfoPtr);
void Can_Receive_Interrupt_RIGHT_INVERTER_MSG2(PduIdType RxPduId, const PduInfoType * PduInfoPtr);

void Can_Transmit_Interrupt_INVERTOR_STANGA(void);
void Can_Transmit_Interrupt_INVERTOR_DREAPTA(void);
void Can_Transmit_Interrupt_INVERTOARE(void);
void Can_Transmit_Interrupt_BORD(void);
void Can_Transmit_Interrupt_ACCELERATIE(void);
void Can_Transmit_Interrupt_FRANA(void);
void Can_Transmit_Interrupt_BATERIE(void);
void Can_Transmit_Interrupt_BATERIE_TENSIUNI_CELULE(void);
void Can_Transmit_Interrupt_BATERIE_TEMPERATURI_CELULE(void);
void Can_Transmit_Interrupt_BATERIE_2(void);
void Can_Transmit_Interrupt_BATERIE_CHARGER(void);
void Can_Transmit_Interrupt_COMUNICATII(void);

void Can_Timer_Timeout(void);

/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/
#define ReadCanDataFromAddress(xMonitoredValue_t_Address) (xMonitoredValue_t_Address)->valueCan
#define WriteCanDataAtAddress(data, xMonitoredValue_t_Address) ((xMonitoredValue_t_Address)->valueCan = ((data) <= (xMonitoredValue_t_Address)->maxValue) ? (data) : ((xMonitoredValue_t_Address)->maxValue), transmission_data_updated = 1)

void CanMessaging_Init(void);
void CanMessaging_Test(void);
void CanMessaging_Update(void);
void CanMessaging_AppTest(void);
void CanMessaging_SetCellVoltage(uint16_t Value, uint16_t index);
void CanMessaging_SetCellVoltageErrors(boolean Value, uint16_t index);
void CanMessaging_SetCellTemperature(uint16_t Value, uint16_t index);
void CanMessaging_SetCellTemperatureErrors(boolean Value, uint16_t index);
uint16_t CanMessaging_ReadCellVoltage(uint16_t index);
boolean CanMessaging_ReadCellVoltageErrors(uint16_t index);
uint16_t CanMessaging_ReadCellTemperature(uint16_t index);
boolean CanMessaging_ReadCellTemperatureErrors(uint16_t index);
bool CanMessaging_GetInvertersReceiveTimeout(void);
bool CanMessaging_GetPedalsReceiveTimeout(void);
bool CanMessaging_GetBatteryReceiveTimeout(void);
bool CanMessaging_GetDashboardReceiveTimeout(void);
bool CanMessaging_GetCommunicationsReceiveTimeout(void);

#ifdef __cplusplus
}
#endif

#endif
