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
#include "Mcu.h"
#include "Port.h"
#include "Platform.h"
#include "Can_GeneralTypes.h"
#include "Can_43_FLEXCAN.h"
#include "CanIf.h"
#include "SchM_Can_43_FLEXCAN.h"
#include "CDD_Uart.h"
#include "check_example.h"
#include "Dio.h"
#include "Mcl.h"
#include "UartMessaging.h"
#include "Messaging_Types.h"

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/


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

InvertersMonitoredValues_t invertoareUart;
PedalsMonitoredValues_t pedaleUart;
TsacMonitoredValues_t baterieUart;
DashboardMonitoredValues_t bordUart;

uint8_t bufferUart[10];

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/


/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

void UartMessaging_Init(void){
	Uart_Init(NULL_PTR);
}

void UartMessaging_Test(void){
	int cnt = 0;
	volatile int i;
	while(1){
		UartMessaging_SetValue(Uart_TSAC_MedianCellTemperature, Uart_TSAC_MedianCellTemperature + cnt);
		UartMessaging_SetValue(Uart_TSAC_HighestCellTemperature, Uart_TSAC_HighestCellTemperature + cnt);
		UartMessaging_SetValue(Uart_TSAC_LowestCellTemperature, Uart_TSAC_LowestCellTemperature + cnt);
		UartMessaging_SetValue(Uart_TSAC_MedianCellVoltage, Uart_TSAC_MedianCellVoltage + cnt);
		UartMessaging_SetValue(Uart_TSAC_HighestCellVoltage, Uart_TSAC_HighestCellVoltage + cnt);
		UartMessaging_SetValue(Uart_TSAC_LowestCellVoltage, Uart_TSAC_LowestCellVoltage + cnt);
		UartMessaging_SetValue(Uart_TSAC_OverallVoltage, Uart_TSAC_OverallVoltage + cnt);
		UartMessaging_SetValue(Uart_TSAC_OverallCurrent, Uart_TSAC_OverallCurrent + cnt);
		//baterieUart.CellVoltage[CELLS_NUM];
		//baterieUart.ThermistorTemperature[THERMISTOR_NUM];
		UartMessaging_SetValue(Uart_TSAC_IsAmsSafe, 1);
		UartMessaging_SetValue(Uart_TSAC_IsImdSafe, 1);
		UartMessaging_SetValue(Uart_TSAC_IsTransceiverWorking, 1);
		UartMessaging_SetValue(Uart_TSAC_IsShuntWorking, 1);
		UartMessaging_SetValue(Uart_TSAC_IsBms0Working, 1);
		UartMessaging_SetValue(Uart_TSAC_IsBms1Working, 1);

		UartMessaging_SetValue(Uart_PEDALS_AcceleratorSensor1Voltage, Uart_PEDALS_AcceleratorSensor1Voltage + cnt);
		UartMessaging_SetValue(Uart_PEDALS_AcceleratorSensor2Voltage, Uart_PEDALS_AcceleratorSensor2Voltage + cnt);
		UartMessaging_SetValue(Uart_PEDALS_AcceleratorSensor1TravelPercentage, Uart_PEDALS_AcceleratorSensor1TravelPercentage + cnt);
		UartMessaging_SetValue(Uart_PEDALS_AcceleratorSensor2TravelPercentage, Uart_PEDALS_AcceleratorSensor2TravelPercentage + cnt);
		UartMessaging_SetValue(Uart_PEDALS_BrakeSensor1Voltage, Uart_PEDALS_BrakeSensor1Voltage + cnt);
		UartMessaging_SetValue(Uart_PEDALS_BrakeSensor2Voltage, Uart_PEDALS_BrakeSensor2Voltage + cnt);
		UartMessaging_SetValue(Uart_PEDALS_BrakeSensor1TravelPercentage, Uart_PEDALS_BrakeSensor1TravelPercentage + cnt);
		UartMessaging_SetValue(Uart_PEDALS_BrakeSensor2TravelPercentage, Uart_PEDALS_BrakeSensor2TravelPercentage + cnt);
		UartMessaging_SetValue(Uart_PEDALS_PressureSensorVoltage, Uart_PEDALS_PressureSensorVoltage + cnt);
		UartMessaging_SetValue(Uart_PEDALS_PressureSensorBars, Uart_PEDALS_PressureSensorBars + cnt);
		UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor1_ShortToGnd, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor1_ShortToVcc, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor1_OutOfRangeOutput, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor2_ShortToGnd, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor2_ShortToVcc, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor2_OutOfRangeOutput, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Accel_Implausibility, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor1_ShortToGnd, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor1_ShortToVcc, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor1_OutOfRangeOutput, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor2_ShortToGnd, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor2_ShortToVcc, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor2_OutOfRangeOutput, cnt & 1);
		UartMessaging_SetValue(Uart_PEDALS_Brake_Implausibility, cnt & 1);

		UartMessaging_SetValue(Uart_INVERTERS_LeftInverterTemperature, Uart_INVERTERS_LeftInverterTemperature + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_LeftMotorTemperature, Uart_INVERTERS_LeftMotorTemperature + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_LeftInverterInputVoltage, Uart_INVERTERS_LeftInverterInputVoltage + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_LeftInverterCurrent, Uart_INVERTERS_LeftInverterCurrent + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_LeftMotorRpm, Uart_INVERTERS_LeftMotorRpm + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_LeftMotorSpeedKmh, Uart_INVERTERS_LeftMotorSpeedKmh + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_LeftInverterThrottle, Uart_INVERTERS_LeftInverterThrottle + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_LeftInverterThrottleFeedback, Uart_INVERTERS_LeftInverterThrottleFeedback + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_RightInverterTemperature, Uart_INVERTERS_RightInverterTemperature + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_RightMotorTemperature, Uart_INVERTERS_RightMotorTemperature + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_RightInverterInputVoltage, Uart_INVERTERS_RightInverterInputVoltage + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_RightInverterCurrent, Uart_INVERTERS_RightInverterCurrent + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_RightMotorRpm, Uart_INVERTERS_RightMotorRpm + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_RightMotorSpeedKmh, Uart_INVERTERS_RightMotorSpeedKmh + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_RightInverterSentThrottle, Uart_INVERTERS_RightInverterSentThrottle + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_RightInverterThrottleFeedback, Uart_INVERTERS_RightInverterThrottleFeedback + cnt);
		UartMessaging_SetValue(Uart_INVERTERS_IsCarInReverse, cnt & 1);
		UartMessaging_SetValue(Uart_INVERTERS_IsCarRunning, cnt & 1);

		UartMessaging_SetValue(Uart_DASHBOARD_ActivationButtonPressed, cnt & 1);
		UartMessaging_SetValue(Uart_DASHBOARD_CarReverseCommandPressed, cnt & 1);
		UartMessaging_SetValue(Uart_DASHBOARD_IsDisplayWorking, cnt & 1);
		UartMessaging_SetValue(Uart_DASHBOARD_IsSegmentsDriverWorking, cnt & 1);

		cnt++;
		UartMessaging_Update();
		i=100000;
		while(i--);
	}
}

void UartMessaging_Update(void){
	volatile int i;
	UartMessaging_CreateBuffer(idUartInvertorStanga);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	i=100000;
	while(i--);
	UartMessaging_CreateBuffer(idUartInvertorDreapta);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	i=100000;
	while(i--);
	UartMessaging_CreateBuffer(idUartInvertoare);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	i=100000;
	while(i--);
	UartMessaging_CreateBuffer(idUartBord);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	i=100000;
	while(i--);
	UartMessaging_CreateBuffer(idUartAcceleratie);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	i=100000;
	while(i--);
	UartMessaging_CreateBuffer(idUartFrana);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	i=100000;
	while(i--);
	UartMessaging_CreateBuffer(idUartBaterie);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	i=100000;
	while(i--);
}

void UartMessaging_SetValue(UartMonitoredValue_t DesiredValueType, uint32_t Value){
	switch(DesiredValueType){
		//TSAC
		case Uart_TSAC_MedianCellTemperature:
			baterieUart.MedianCellTemperature = Value;
			break;
		case Uart_TSAC_HighestCellTemperature:
			baterieUart.HighestCellTemperature = Value;
			break;
		case Uart_TSAC_LowestCellTemperature:
			baterieUart.LowestCellTemperature = Value;
			break;
		case Uart_TSAC_MedianCellVoltage:
			baterieUart.MedianCellVoltage = Value;
			break;
		case Uart_TSAC_HighestCellVoltage:
			baterieUart.HighestCellVoltage = Value;
			break;
		case Uart_TSAC_LowestCellVoltage:
			baterieUart.LowestCellVoltage = Value;
			break;
		case Uart_TSAC_OverallVoltage:
			baterieUart.OverallVoltage = Value;
			break;
		case Uart_TSAC_OverallCurrent:
			if(Value>8095)
				baterieUart.OverallCurrent = 0;
			else
				baterieUart.OverallCurrent = Value;
			break;
		case Uart_TSAC_IsAmsSafe:
			baterieUart.AmsError = Value;
			break;
		case Uart_TSAC_IsImdSafe:
			baterieUart.ImdError = Value;
			break;
		case Uart_TSAC_IsTransceiverWorking:
			baterieUart.TransceiverError = Value;
			break;
		case Uart_TSAC_IsShuntWorking:
			baterieUart.ShuntError = Value;
			break;
		case Uart_TSAC_IsBms0Working:
			baterieUart.Bms0Error = Value;
			break;
		case Uart_TSAC_IsBms1Working:
			baterieUart.Bms1Error = Value;
			break;
		//PEDALS
		case Uart_PEDALS_AcceleratorSensor1Voltage:
			pedaleUart.AcceleratorSensor1Voltage = Value;
			break;
		case Uart_PEDALS_AcceleratorSensor2Voltage:
			pedaleUart.AcceleratorSensor2Voltage = Value;
			break;
		case Uart_PEDALS_AcceleratorSensor1TravelPercentage:
			if(Value>100)
				pedaleUart.AcceleratorSensor1TravelPercentage = 0;
			else
				pedaleUart.AcceleratorSensor1TravelPercentage = Value;
			break;
		case Uart_PEDALS_AcceleratorSensor2TravelPercentage:
			if(Value>100)
				pedaleUart.AcceleratorSensor2TravelPercentage = 0;
			else
				pedaleUart.AcceleratorSensor2TravelPercentage = Value;
			break;
		case Uart_PEDALS_BrakeSensor1Voltage:
			pedaleUart.BrakeSensor1Voltage = Value;
			break;
		case Uart_PEDALS_BrakeSensor2Voltage:
			pedaleUart.BrakeSensor2Voltage = Value;
			break;
		case Uart_PEDALS_BrakeSensor1TravelPercentage:
			if(Value>100)
				pedaleUart.BrakeSensor1TravelPercentage = 0;
			else
				pedaleUart.BrakeSensor1TravelPercentage = Value;
			break;
		case Uart_PEDALS_BrakeSensor2TravelPercentage:
			if(Value>100)
				pedaleUart.BrakeSensor2TravelPercentage = 0;
			else
				pedaleUart.BrakeSensor2TravelPercentage = Value;
			break;
		case Uart_PEDALS_PressureSensorVoltage:
			if(Value>500)
				pedaleUart.PressureSensorVoltage = 0;
			else
				pedaleUart.PressureSensorVoltage = Value;
			break;
		case Uart_PEDALS_PressureSensorBars:
			pedaleUart.PressureSensorBars = Value;
			break;
		case Uart_PEDALS_Accel_Sensor1_ShortToGnd:
			pedaleUart.Accel_Sensor1_ShortToGnd = Value;
			break;
		case Uart_PEDALS_Accel_Sensor1_ShortToVcc:
			pedaleUart.Accel_Sensor1_ShortToVcc = Value;
			break;
		case Uart_PEDALS_Accel_Sensor1_OutOfRangeOutput:
			pedaleUart.Accel_Sensor1_OutOfRangeOutput = Value;
			break;
		case Uart_PEDALS_Accel_Sensor2_ShortToGnd:
			pedaleUart.Accel_Sensor2_ShortToGnd = Value;
			break;
		case Uart_PEDALS_Accel_Sensor2_ShortToVcc:
			pedaleUart.Accel_Sensor2_ShortToVcc = Value;
			break;
		case Uart_PEDALS_Accel_Sensor2_OutOfRangeOutput:
			pedaleUart.Accel_Sensor2_OutOfRangeOutput = Value;
			break;
		case Uart_PEDALS_Accel_Implausibility:
			pedaleUart.Accel_Implausibility = Value;
			break;
		case Uart_PEDALS_Brake_Sensor1_ShortToGnd:
			pedaleUart.Brake_Sensor1_ShortToGnd = Value;
			break;
		case Uart_PEDALS_Brake_Sensor1_ShortToVcc:
			pedaleUart.Brake_Sensor1_ShortToVcc = Value;
			break;
		case Uart_PEDALS_Brake_Sensor1_OutOfRangeOutput:
			pedaleUart.Brake_Sensor1_OutOfRangeOutput = Value;
			break;
		case Uart_PEDALS_Brake_Sensor2_ShortToGnd:
			pedaleUart.Brake_Sensor2_ShortToGnd = Value;
			break;
		case Uart_PEDALS_Brake_Sensor2_ShortToVcc:
			pedaleUart.Brake_Sensor2_ShortToVcc = Value;
			break;
		case Uart_PEDALS_Brake_Sensor2_OutOfRangeOutput:
			pedaleUart.Brake_Sensor2_OutOfRangeOutput = Value;
			break;
		case Uart_PEDALS_Brake_Implausibility:
			pedaleUart.Brake_Implausibility = Value;
			break;
		//INVERTERS
		case Uart_INVERTERS_LeftInverterTemperature:
			invertoareUart.LeftInverterTemperature = Value;
			break;
		case Uart_INVERTERS_LeftMotorTemperature:
			invertoareUart.LeftMotorTemperature = Value;
			break;
		case Uart_INVERTERS_LeftInverterInputVoltage:
			if(Value>1800)
				invertoareUart.LeftInverterInputVoltage = 0;
			else
				invertoareUart.LeftInverterInputVoltage = Value;
			break;
		case Uart_INVERTERS_LeftInverterCurrent:
			if(Value>4000)
				invertoareUart.LeftInverterCurrent = 0;
			else
				invertoareUart.LeftInverterCurrent = Value;
			break;
		case Uart_INVERTERS_LeftMotorRpm:
			if(Value>6000)
				invertoareUart.LeftMotorRpm = 0;
			else
				invertoareUart.LeftMotorRpm = Value;
			break;
		case Uart_INVERTERS_LeftMotorSpeedKmh:
			invertoareUart.LeftMotorSpeedKmh = Value;
			break;
		case Uart_INVERTERS_LeftInverterThrottle:
			if(Value>250)
				invertoareUart.LeftInverterThrottle = 0;
			else
				invertoareUart.LeftInverterThrottle = Value;
			break;
		case Uart_INVERTERS_LeftInverterThrottleFeedback:
			if(Value>250)
				invertoareUart.LeftInverterThrottleFeedback = 0;
			else
				invertoareUart.LeftInverterThrottleFeedback = Value;
			break;
		case Uart_INVERTERS_RightInverterTemperature:
			invertoareUart.RightInverterTemperature = Value;
			break;
		case Uart_INVERTERS_RightMotorTemperature:
			invertoareUart.RightMotorTemperature = Value;
			break;
		case Uart_INVERTERS_RightInverterInputVoltage:
			if(Value>1800)
				invertoareUart.RightInverterInputVoltage = 0;
			else
				invertoareUart.RightInverterInputVoltage = Value;
			break;
		case Uart_INVERTERS_RightInverterCurrent:
			if(Value>4000)
				invertoareUart.RightInverterCurrent = 0;
			else
				invertoareUart.RightInverterCurrent = Value;
			break;
		case Uart_INVERTERS_RightMotorRpm:
			if(Value>6000)
				invertoareUart.RightMotorRpm = 0;
			else
				invertoareUart.RightMotorRpm = Value;
			break;
		case Uart_INVERTERS_RightMotorSpeedKmh:
			invertoareUart.RightMotorSpeedKmh = Value;
			break;
		case Uart_INVERTERS_RightInverterSentThrottle:
			if(Value>250)
				invertoareUart.RightInverterSentThrottle = 0;
			else
				invertoareUart.RightInverterSentThrottle = Value;
			break;
		case Uart_INVERTERS_RightInverterThrottleFeedback:
			if(Value>250)
				invertoareUart.RightInverterThrottleFeedback = 0;
			else
				invertoareUart.RightInverterThrottleFeedback = Value;
			break;
		case Uart_INVERTERS_IsCarInReverse:
			invertoareUart.IsCarInReverse = Value;
			break;
		case Uart_INVERTERS_IsCarRunning:
			invertoareUart.IsCarRunning = Value;
			break;
		//DASHBOARD
		case Uart_DASHBOARD_ActivationButtonPressed:
			bordUart.ActivationButtonPressed = Value;
			break;
		case Uart_DASHBOARD_CarReverseCommandPressed:
			bordUart.CarReverseCommandPressed = Value;
			break;
		case Uart_DASHBOARD_IsDisplayWorking:
			bordUart.IsDisplayWorking = Value;
			break;
		case Uart_DASHBOARD_IsSegmentsDriverWorking:
			bordUart.IsSegmentsDriverWorking = Value;
			break;
	}
}

uint32_t UartMessaging_ReadValue(UartMonitoredValue_t DesiredValueType){
	switch(DesiredValueType){
		case Uart_TSAC_MedianCellTemperature:
			return baterieUart.MedianCellTemperature;
		case Uart_TSAC_HighestCellTemperature:
			return baterieUart.HighestCellTemperature;
		case Uart_TSAC_LowestCellTemperature:
			return baterieUart.LowestCellTemperature;
		case Uart_TSAC_MedianCellVoltage:
			return baterieUart.MedianCellVoltage;
		case Uart_TSAC_HighestCellVoltage:
			return baterieUart.HighestCellVoltage;
		case Uart_TSAC_LowestCellVoltage:
			return baterieUart.LowestCellVoltage;
		case Uart_TSAC_OverallVoltage:
			return baterieUart.OverallVoltage;
		case Uart_TSAC_OverallCurrent:
			return baterieUart.OverallCurrent;
		case Uart_TSAC_IsAmsSafe:
			return baterieUart.AmsError;
		case Uart_TSAC_IsImdSafe:
			return baterieUart.ImdError;
		case Uart_TSAC_IsTransceiverWorking:
			return baterieUart.TransceiverError;
		case Uart_TSAC_IsShuntWorking:
			return baterieUart.ShuntError;
		case Uart_TSAC_IsBms0Working:
			return baterieUart.Bms0Error;
		case Uart_TSAC_IsBms1Working:
			return baterieUart.Bms1Error;
		case Uart_PEDALS_AcceleratorSensor1Voltage:
			return pedaleUart.AcceleratorSensor1Voltage;
		case Uart_PEDALS_AcceleratorSensor2Voltage:
			return pedaleUart.AcceleratorSensor2Voltage;
		case Uart_PEDALS_AcceleratorSensor1TravelPercentage:
			return pedaleUart.AcceleratorSensor1TravelPercentage;
		case Uart_PEDALS_AcceleratorSensor2TravelPercentage:
			return pedaleUart.AcceleratorSensor2TravelPercentage;
		case Uart_PEDALS_BrakeSensor1Voltage:
			return pedaleUart.BrakeSensor1Voltage;
		case Uart_PEDALS_BrakeSensor2Voltage:
			return pedaleUart.BrakeSensor2Voltage;
		case Uart_PEDALS_BrakeSensor1TravelPercentage:
			return pedaleUart.BrakeSensor1TravelPercentage;
		case Uart_PEDALS_BrakeSensor2TravelPercentage:
			return pedaleUart.BrakeSensor2TravelPercentage;
		case Uart_PEDALS_PressureSensorVoltage:
			return pedaleUart.PressureSensorVoltage;
		case Uart_PEDALS_PressureSensorBars:
			return pedaleUart.PressureSensorBars;
		case Uart_PEDALS_Accel_Sensor1_ShortToGnd:
			return pedaleUart.Accel_Sensor1_ShortToGnd;
		case Uart_PEDALS_Accel_Sensor1_ShortToVcc:
			return pedaleUart.Accel_Sensor1_ShortToVcc;
		case Uart_PEDALS_Accel_Sensor1_OutOfRangeOutput:
			return pedaleUart.Accel_Sensor1_OutOfRangeOutput;
		case Uart_PEDALS_Accel_Sensor2_ShortToGnd:
			return pedaleUart.Accel_Sensor2_ShortToGnd;
		case Uart_PEDALS_Accel_Sensor2_ShortToVcc:
			return pedaleUart.Accel_Sensor2_ShortToVcc;
		case Uart_PEDALS_Accel_Sensor2_OutOfRangeOutput:
			return pedaleUart.Accel_Sensor2_OutOfRangeOutput;
		case Uart_PEDALS_Accel_Implausibility:
			return pedaleUart.Accel_Implausibility;
		case Uart_PEDALS_Brake_Sensor1_ShortToGnd:
			return pedaleUart.Brake_Sensor1_ShortToGnd;
		case Uart_PEDALS_Brake_Sensor1_ShortToVcc:
			return pedaleUart.Brake_Sensor1_ShortToVcc;
		case Uart_PEDALS_Brake_Sensor1_OutOfRangeOutput:
			return pedaleUart.Brake_Sensor1_OutOfRangeOutput;
		case Uart_PEDALS_Brake_Sensor2_ShortToGnd:
			return pedaleUart.Brake_Sensor2_ShortToGnd;
		case Uart_PEDALS_Brake_Sensor2_ShortToVcc:
			return pedaleUart.Brake_Sensor2_ShortToVcc;
		case Uart_PEDALS_Brake_Sensor2_OutOfRangeOutput:
			return pedaleUart.Brake_Sensor2_OutOfRangeOutput;
		case Uart_PEDALS_Brake_Implausibility:
			return pedaleUart.Brake_Implausibility;
		case Uart_INVERTERS_LeftInverterTemperature:
			return invertoareUart.LeftInverterTemperature;
		case Uart_INVERTERS_LeftMotorTemperature:
			return invertoareUart.LeftMotorTemperature;
		case Uart_INVERTERS_LeftInverterInputVoltage:
			return invertoareUart.LeftInverterInputVoltage;
		case Uart_INVERTERS_LeftInverterCurrent:
			return invertoareUart.LeftInverterCurrent;
		case Uart_INVERTERS_LeftMotorRpm:
			return invertoareUart.LeftMotorRpm;
		case Uart_INVERTERS_LeftMotorSpeedKmh:
			return invertoareUart.LeftMotorSpeedKmh;
		case Uart_INVERTERS_LeftInverterThrottle:
			return invertoareUart.LeftInverterThrottle;
		case Uart_INVERTERS_LeftInverterThrottleFeedback:
			return invertoareUart.LeftInverterThrottleFeedback;
		case Uart_INVERTERS_RightInverterTemperature:
			return invertoareUart.RightInverterTemperature;
		case Uart_INVERTERS_RightMotorTemperature:
			return invertoareUart.RightMotorTemperature;
		case Uart_INVERTERS_RightInverterInputVoltage:
			return invertoareUart.RightInverterInputVoltage;
		case Uart_INVERTERS_RightInverterCurrent:
			return invertoareUart.RightInverterCurrent;
		case Uart_INVERTERS_RightMotorRpm:
			return invertoareUart.RightMotorRpm;
		case Uart_INVERTERS_RightMotorSpeedKmh:
			return invertoareUart.RightMotorSpeedKmh;
		case Uart_INVERTERS_RightInverterSentThrottle:
			return invertoareUart.RightInverterSentThrottle;
		case Uart_INVERTERS_RightInverterThrottleFeedback:
			return invertoareUart.RightInverterThrottleFeedback;
		case Uart_INVERTERS_IsCarInReverse:
			return invertoareUart.IsCarInReverse;
		case Uart_INVERTERS_IsCarRunning:
			return invertoareUart.IsCarRunning;
		case Uart_DASHBOARD_ActivationButtonPressed:
			return bordUart.ActivationButtonPressed;
		case Uart_DASHBOARD_CarReverseCommandPressed:
			return bordUart.CarReverseCommandPressed;
		case Uart_DASHBOARD_IsDisplayWorking:
			return bordUart.IsDisplayWorking;
		case Uart_DASHBOARD_IsSegmentsDriverWorking:
			return bordUart.IsSegmentsDriverWorking;
	}
	return 0;
}

void UartMessaging_CreateBuffer(idUart_t type){
	switch(type){
		case idUartInvertorStanga:
			bufferUart[0] = idUartInvertorStanga;
			bufferUart[1] = UartMessaging_ReadValue(Uart_INVERTERS_LeftMotorRpm) >> 5;
			bufferUart[2] = (((UartMessaging_ReadValue(Uart_INVERTERS_LeftMotorRpm) & (0x001F) )) << 3) | ((UartMessaging_ReadValue(Uart_INVERTERS_LeftInverterInputVoltage) & (0x0700)) >> 8);
			bufferUart[3] = ((uint8_t) UartMessaging_ReadValue(Uart_INVERTERS_LeftInverterInputVoltage) & (0x00FF));
			bufferUart[4] = UartMessaging_ReadValue(Uart_INVERTERS_LeftInverterThrottleFeedback);
			bufferUart[5] = UartMessaging_ReadValue(Uart_INVERTERS_LeftMotorSpeedKmh);
			bufferUart[6] = UartMessaging_ReadValue(Uart_INVERTERS_LeftInverterThrottle);
			bufferUart[7] = UartMessaging_ReadValue(Uart_INVERTERS_LeftInverterTemperature);
			bufferUart[8] = UartMessaging_ReadValue(Uart_INVERTERS_LeftMotorTemperature);
			bufferUart[9] = CRC_calculate(10);
			break;
		case idUartInvertorDreapta:
			bufferUart[0] = idUartInvertorDreapta;
			bufferUart[1] = UartMessaging_ReadValue(Uart_INVERTERS_RightMotorRpm) >> 5;
			bufferUart[2] = (((UartMessaging_ReadValue(Uart_INVERTERS_RightMotorRpm) & (0x001F) )) << 3) | ((UartMessaging_ReadValue(Uart_INVERTERS_RightInverterInputVoltage) & (0x0700)) >> 8);
			bufferUart[3] = ((uint8_t) UartMessaging_ReadValue(Uart_INVERTERS_RightInverterInputVoltage) & (0x7FF));
			bufferUart[4] = UartMessaging_ReadValue(Uart_INVERTERS_RightInverterThrottleFeedback);
			bufferUart[5] = UartMessaging_ReadValue(Uart_INVERTERS_RightMotorSpeedKmh);
			bufferUart[6] = UartMessaging_ReadValue(Uart_INVERTERS_RightInverterSentThrottle);
			bufferUart[7] = UartMessaging_ReadValue(Uart_INVERTERS_RightInverterTemperature);
			bufferUart[8] = UartMessaging_ReadValue(Uart_INVERTERS_RightMotorTemperature);
			bufferUart[9] = CRC_calculate(10);
			break;
		case idUartInvertoare:
			bufferUart[0] = idUartInvertoare;
			bufferUart[1] = (UartMessaging_ReadValue(Uart_INVERTERS_IsCarRunning) << 7) | (UartMessaging_ReadValue(Uart_INVERTERS_IsCarInReverse) << 6);
			bufferUart[2] = 0;
			bufferUart[3] = 0;
			bufferUart[4] = 0;
			bufferUart[5] = 0;
			bufferUart[6] = (UartMessaging_ReadValue(Uart_INVERTERS_RightInverterCurrent) & (0x0FF0)) >> 4;
			bufferUart[7] = ((UartMessaging_ReadValue(Uart_INVERTERS_RightInverterCurrent) & (0x000F)) << 4) | ((UartMessaging_ReadValue(Uart_INVERTERS_LeftInverterCurrent) & (0x0F00)) >> 8);
			bufferUart[8] = UartMessaging_ReadValue(Uart_INVERTERS_LeftInverterCurrent) & (0x00FF);
			bufferUart[9] = CRC_calculate(10);
			break;
		case idUartBord:
			bufferUart[0] = idUartBord;
			bufferUart[1] = (UartMessaging_ReadValue(Uart_DASHBOARD_ActivationButtonPressed) << 7) | (UartMessaging_ReadValue(Uart_DASHBOARD_CarReverseCommandPressed) << 6) | (UartMessaging_ReadValue(Uart_DASHBOARD_IsDisplayWorking) << 5) | (UartMessaging_ReadValue(Uart_DASHBOARD_IsSegmentsDriverWorking) << 4);
			bufferUart[2] = 0;
			bufferUart[3] = 0;
			bufferUart[4] = 0;
			bufferUart[5] = 0;
			bufferUart[6] = 0;
			bufferUart[7] = 0;
			bufferUart[8] = 0;
			bufferUart[9] = CRC_calculate(10);
			break;
		case idUartAcceleratie:
			bufferUart[0] = idUartAcceleratie;
			bufferUart[1] = (UartMessaging_ReadValue(Uart_PEDALS_Accel_Sensor1_ShortToGnd) << 7) | (UartMessaging_ReadValue(Uart_PEDALS_Accel_Sensor1_ShortToVcc) << 6) | (UartMessaging_ReadValue(Uart_PEDALS_Accel_Sensor1_OutOfRangeOutput) << 5) | (UartMessaging_ReadValue(Uart_PEDALS_Accel_Sensor2_ShortToGnd) << 4) | (UartMessaging_ReadValue(Uart_PEDALS_Accel_Sensor2_ShortToVcc) << 3) | (UartMessaging_ReadValue(Uart_PEDALS_Accel_Sensor2_OutOfRangeOutput) << 2) | (UartMessaging_ReadValue(Uart_PEDALS_Accel_Implausibility) << 1);
			bufferUart[2] = (UartMessaging_ReadValue(Uart_PEDALS_PressureSensorVoltage) & (0x01C0)) >> 6;
			bufferUart[3] = ((UartMessaging_ReadValue(Uart_PEDALS_PressureSensorVoltage) & (0x003F)) << 2) | ((UartMessaging_ReadValue(Uart_PEDALS_AcceleratorSensor2TravelPercentage) & (0x60)) >> 5);
			bufferUart[4] = ((UartMessaging_ReadValue(Uart_PEDALS_AcceleratorSensor2TravelPercentage) & (0x1F)) << 3) | ((UartMessaging_ReadValue(Uart_PEDALS_AcceleratorSensor1TravelPercentage) & (0x70)) >> 4);
			bufferUart[5] = ((UartMessaging_ReadValue(Uart_PEDALS_AcceleratorSensor1TravelPercentage) & (0x0F)) << 4) | ((UartMessaging_ReadValue(Uart_PEDALS_AcceleratorSensor2Voltage) & (0x3C00)) >> 10);
			bufferUart[6] = (UartMessaging_ReadValue(Uart_PEDALS_AcceleratorSensor2Voltage) & (0x03FC)) >> 2;
			bufferUart[7] = ((UartMessaging_ReadValue(Uart_PEDALS_AcceleratorSensor1Voltage) & (0x3F00)) >>8) | ((UartMessaging_ReadValue(Uart_PEDALS_AcceleratorSensor2Voltage) & (0x0003)) << 6);
			bufferUart[8] = UartMessaging_ReadValue(Uart_PEDALS_AcceleratorSensor1Voltage) & (0x00FF);
			bufferUart[9] = CRC_calculate(10);
			break;
		case idUartFrana:
			bufferUart[0] = idUartFrana;
			bufferUart[1] = (UartMessaging_ReadValue(Uart_PEDALS_Brake_Sensor1_ShortToGnd) << 7) | (UartMessaging_ReadValue(Uart_PEDALS_Brake_Sensor1_ShortToVcc) << 6) | (UartMessaging_ReadValue(Uart_PEDALS_Brake_Sensor1_OutOfRangeOutput) << 5) | (UartMessaging_ReadValue(Uart_PEDALS_Brake_Sensor2_ShortToGnd) << 4) | (UartMessaging_ReadValue(Uart_PEDALS_Brake_Sensor2_ShortToVcc) << 3) | (UartMessaging_ReadValue(Uart_PEDALS_Brake_Sensor2_OutOfRangeOutput) << 2) | (UartMessaging_ReadValue(Uart_PEDALS_Brake_Implausibility) << 1);
			bufferUart[2] = (UartMessaging_ReadValue(Uart_PEDALS_PressureSensorBars) & (0xC0)) >> 6;
			bufferUart[3] = ((UartMessaging_ReadValue(Uart_PEDALS_PressureSensorBars) & (0x3F)) << 2) | ((UartMessaging_ReadValue(Uart_PEDALS_BrakeSensor2TravelPercentage) & (0x60)) >> 5);
			bufferUart[4] = ((UartMessaging_ReadValue(Uart_PEDALS_BrakeSensor2TravelPercentage) & (0x1F)) << 3) | ((UartMessaging_ReadValue(Uart_PEDALS_BrakeSensor1TravelPercentage) & (0x70)) >> 4);
			bufferUart[5] = ((UartMessaging_ReadValue(Uart_PEDALS_BrakeSensor1TravelPercentage) & (0x0F)) << 4) | ((UartMessaging_ReadValue(Uart_PEDALS_BrakeSensor2Voltage) & (0x3C00)) >> 10);
			bufferUart[6] = (UartMessaging_ReadValue(Uart_PEDALS_BrakeSensor2Voltage) & (0x03FC)) >> 2;
			bufferUart[7] = ((UartMessaging_ReadValue(Uart_PEDALS_BrakeSensor1Voltage) & (0x3F00)) >>8) | ((UartMessaging_ReadValue(Uart_PEDALS_BrakeSensor2Voltage) & (0x0003)) << 6);
			bufferUart[8] = UartMessaging_ReadValue(Uart_PEDALS_BrakeSensor1Voltage) & (0x00FF);
			bufferUart[9] = CRC_calculate(10);
			break;
		case idUartBaterie:
			bufferUart[0] = idUartBaterie;
			bufferUart[1] = 0;
			bufferUart[2] = 0;
			bufferUart[3] = (UartMessaging_ReadValue(Uart_TSAC_HighestCellVoltage) & (0x03C0)) >> 6;
			bufferUart[4] = ((UartMessaging_ReadValue(Uart_TSAC_HighestCellTemperature) & (0x0300)) >> 8) | ((UartMessaging_ReadValue(Uart_TSAC_HighestCellVoltage) & (0x003F)) << 2);
			bufferUart[5] = UartMessaging_ReadValue(Uart_TSAC_HighestCellTemperature) & (0x00FF);
			bufferUart[6] = (UartMessaging_ReadValue(Uart_TSAC_OverallVoltage) & (0x07F8)) >> 3;
			bufferUart[7] = ((UartMessaging_ReadValue(Uart_TSAC_OverallVoltage) & (0x0007)) << 5) | ((UartMessaging_ReadValue(Uart_TSAC_OverallCurrent) & (0x1F00)) >> 8);
			bufferUart[8] = UartMessaging_ReadValue(Uart_TSAC_OverallCurrent) & (0x00FF);
			bufferUart[9] = CRC_calculate(10);
			break;
	}
}

uint8_t CRC_calculate(uint8_t length){
	uint8_t crc=0, message[length];
	uint16_t divisor = 0x8D, dividend;
	int i, j;

	for(i=0; i<length-1; i++)
	{
		message[i] = bufferUart[i];
	}

	message[length-1]=0;

	dividend = (message[0] << 8) | message[1];
	for(j=15; j>=8; j--)
		if(dividend & (1 << j))
			dividend ^= divisor << (j-8);

	for(i=2; i<length; i++)
		{
			dividend = (dividend << 8) | message[i];

			for(j=15; j>=8; j--)
				if(dividend & (1 << j))
					dividend ^= divisor << (j-8);
		}

	crc = (dividend % 256);

	return crc;
}



#ifdef __cplusplus
}
#endif
