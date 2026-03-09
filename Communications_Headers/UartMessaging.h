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

typedef enum{
	idUartInvertorStanga = 0x10,
	idUartInvertorDreapta = 0x11,
	idUartInvertoare = 0x12,
	idUartBord = 0x13,
	idUartAcceleratie = 0x30,
	idUartFrana = 0x31,
	idUartBaterie = 0x14
}idUart_t;

typedef enum{
    /* TSAC */
    Uart_TSAC_MedianCellTemperature,
    Uart_TSAC_HighestCellTemperature,
    Uart_TSAC_LowestCellTemperature,
    Uart_TSAC_MedianCellVoltage,
    Uart_TSAC_HighestCellVoltage,
    Uart_TSAC_LowestCellVoltage,
    Uart_TSAC_OverallVoltage,
    Uart_TSAC_OverallCurrent,
    Uart_TSAC_IsAmsSafe,
    Uart_TSAC_IsImdSafe,
    Uart_TSAC_IsTransceiverWorking,
    Uart_TSAC_IsShuntWorking,
    Uart_TSAC_IsBms0Working,
    Uart_TSAC_IsBms1Working,
    /* PEDALS */
    Uart_PEDALS_AcceleratorSensor1Voltage,
    Uart_PEDALS_AcceleratorSensor2Voltage,
    Uart_PEDALS_AcceleratorSensor1TravelPercentage,
    Uart_PEDALS_AcceleratorSensor2TravelPercentage,
    Uart_PEDALS_BrakeSensor1Voltage,
    Uart_PEDALS_BrakeSensor2Voltage,
    Uart_PEDALS_BrakeSensor1TravelPercentage,
    Uart_PEDALS_BrakeSensor2TravelPercentage,
    Uart_PEDALS_PressureSensorVoltage,
    Uart_PEDALS_PressureSensorBars,
    Uart_PEDALS_Accel_Sensor1_ShortToGnd,
    Uart_PEDALS_Accel_Sensor1_ShortToVcc,
    Uart_PEDALS_Accel_Sensor1_OutOfRangeOutput,
    Uart_PEDALS_Accel_Sensor2_ShortToGnd,
    Uart_PEDALS_Accel_Sensor2_ShortToVcc,
    Uart_PEDALS_Accel_Sensor2_OutOfRangeOutput,
    Uart_PEDALS_Accel_Implausibility,
    Uart_PEDALS_Brake_Sensor1_ShortToGnd,
    Uart_PEDALS_Brake_Sensor1_ShortToVcc,
    Uart_PEDALS_Brake_Sensor1_OutOfRangeOutput,
    Uart_PEDALS_Brake_Sensor2_ShortToGnd,
    Uart_PEDALS_Brake_Sensor2_ShortToVcc,
    Uart_PEDALS_Brake_Sensor2_OutOfRangeOutput,
    Uart_PEDALS_Brake_Implausibility,
    /* INVERTERS */
    Uart_INVERTERS_LeftInverterTemperature,
    Uart_INVERTERS_LeftMotorTemperature,
    Uart_INVERTERS_LeftInverterInputVoltage,
    Uart_INVERTERS_LeftInverterCurrent,
    Uart_INVERTERS_LeftMotorRpm,
    Uart_INVERTERS_LeftMotorSpeedKmh,
    Uart_INVERTERS_LeftInverterThrottle,
    Uart_INVERTERS_LeftInverterThrottleFeedback,
    Uart_INVERTERS_RightInverterTemperature,
    Uart_INVERTERS_RightMotorTemperature,
    Uart_INVERTERS_RightInverterInputVoltage,
    Uart_INVERTERS_RightInverterCurrent,
    Uart_INVERTERS_RightMotorRpm,
    Uart_INVERTERS_RightMotorSpeedKmh,
    Uart_INVERTERS_RightInverterSentThrottle,
    Uart_INVERTERS_RightInverterThrottleFeedback,
    Uart_INVERTERS_IsCarInReverse,
    Uart_INVERTERS_IsCarRunning,
    /* DASHBOARD */
    Uart_DASHBOARD_ActivationButtonPressed,
    Uart_DASHBOARD_CarReverseCommandPressed,
    Uart_DASHBOARD_IsDisplayWorking,
    Uart_DASHBOARD_IsSegmentsDriverWorking
}UartMonitoredValue_t;

#define UART_Channel 0x00000000

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

void UartMessaging_Init(void);
void UartMessaging_Test(void);
void UartMessaging_Update(void);
void UartMessaging_SetValue(UartMonitoredValue_t DesiredValueType, uint32_t Value);
uint32_t UartMessaging_ReadValue(UartMonitoredValue_t DesiredValueType);
void UartMessaging_CreateBuffer(idUart_t type);
uint8_t CRC_calculate(uint8_t length);

#ifdef __cplusplus
}
#endif

#endif
