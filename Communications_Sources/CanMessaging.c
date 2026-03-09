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
#include "CanMessaging.h"
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


InvertersMonitoredValues_t invertoareCan;
PedalsMonitoredValues_t pedaleCan;
TsacMonitoredValues_t baterieCan;
DashboardMonitoredValues_t bordCan;

uint8_t bufferCan[8];


/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/


/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

void CanMessaging_Init(void){
	Can_43_FLEXCAN_Init(NULL_PTR);
	CanIf_Init(NULL_PTR);

	Dio_WriteChannel(32, STD_HIGH); //CAN3_EN
	volatile uint64 i = 1000000;
	while(i--);
	Dio_WriteChannel(33, STD_HIGH); //CAN3_STB_N
	i = 1000000;
	while(i--);
	Dio_WriteChannel(100, STD_HIGH); //CAN2_EN
	i = 1000000;
	while(i--);
	Dio_WriteChannel(118, STD_HIGH); //CAN2_STB_N
	i = 1000000;
	while(i--);

	Can_43_FLEXCAN_SetControllerMode(Can_43_FLEXCANConf_CanController_CanController_0, CAN_CS_STARTED);
	Can_43_FLEXCAN_SetControllerMode(Can_43_FLEXCANConf_CanController_CanController_1, CAN_CS_STARTED);
	Can_43_FLEXCAN_EnableControllerInterrupts(0);
}

void CanMessaging_Test(void){
	int cnt = 0;
	volatile int i;
	while(1){
		CanMessaging_SetValue(Can_TSAC_MedianCellTemperature, Can_TSAC_MedianCellTemperature + cnt);
		CanMessaging_SetValue(Can_TSAC_HighestCellTemperature, Can_TSAC_HighestCellTemperature + cnt);
		CanMessaging_SetValue(Can_TSAC_LowestCellTemperature, Can_TSAC_LowestCellTemperature + cnt);
		CanMessaging_SetValue(Can_TSAC_MedianCellVoltage, Can_TSAC_MedianCellVoltage + cnt);
		CanMessaging_SetValue(Can_TSAC_HighestCellVoltage, Can_TSAC_HighestCellVoltage + cnt);
		CanMessaging_SetValue(Can_TSAC_LowestCellVoltage, Can_TSAC_LowestCellVoltage + cnt);
		CanMessaging_SetValue(Can_TSAC_OverallVoltage, Can_TSAC_OverallVoltage + cnt);
		CanMessaging_SetValue(Can_TSAC_OverallCurrent, Can_TSAC_OverallCurrent + cnt);
		//baterieCan.CellVoltage[CELLS_NUM];
		//baterieCan.ThermistorTemperature[THERMISTOR_NUM];
		CanMessaging_SetValue(Can_TSAC_IsAmsSafe, 1);
		CanMessaging_SetValue(Can_TSAC_IsImdSafe, 1);
		CanMessaging_SetValue(Can_TSAC_IsTransceiverWorking, 1);
		CanMessaging_SetValue(Can_TSAC_IsShuntWorking, 1);
		CanMessaging_SetValue(Can_TSAC_IsBms0Working, 1);
		CanMessaging_SetValue(Can_TSAC_IsBms1Working, 1);

		CanMessaging_SetValue(Can_PEDALS_AcceleratorSensor1Voltage, Can_PEDALS_AcceleratorSensor1Voltage + cnt);
		CanMessaging_SetValue(Can_PEDALS_AcceleratorSensor2Voltage, Can_PEDALS_AcceleratorSensor2Voltage + cnt);
		CanMessaging_SetValue(Can_PEDALS_AcceleratorSensor1TravelPercentage, Can_PEDALS_AcceleratorSensor1TravelPercentage + cnt);
		CanMessaging_SetValue(Can_PEDALS_AcceleratorSensor2TravelPercentage, Can_PEDALS_AcceleratorSensor2TravelPercentage + cnt);
		CanMessaging_SetValue(Can_PEDALS_BrakeSensor1Voltage, Can_PEDALS_BrakeSensor1Voltage + cnt);
		CanMessaging_SetValue(Can_PEDALS_BrakeSensor2Voltage, Can_PEDALS_BrakeSensor2Voltage + cnt);
		CanMessaging_SetValue(Can_PEDALS_BrakeSensor1TravelPercentage, Can_PEDALS_BrakeSensor1TravelPercentage + cnt);
		CanMessaging_SetValue(Can_PEDALS_BrakeSensor2TravelPercentage, Can_PEDALS_BrakeSensor2TravelPercentage + cnt);
		CanMessaging_SetValue(Can_PEDALS_PressureSensorVoltage, Can_PEDALS_PressureSensorVoltage + cnt);
		CanMessaging_SetValue(Can_PEDALS_PressureSensorBars, Can_PEDALS_PressureSensorBars + cnt);
		CanMessaging_SetValue(Can_PEDALS_Accel_Sensor1_ShortToGnd, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Accel_Sensor1_ShortToVcc, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Accel_Sensor1_OutOfRangeOutput, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Accel_Sensor2_ShortToGnd, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Accel_Sensor2_ShortToVcc, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Accel_Sensor2_OutOfRangeOutput, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Accel_Implausibility, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Brake_Sensor1_ShortToGnd, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Brake_Sensor1_ShortToVcc, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Brake_Sensor1_OutOfRangeOutput, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Brake_Sensor2_ShortToGnd, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Brake_Sensor2_ShortToVcc, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Brake_Sensor2_OutOfRangeOutput, cnt & 1);
		CanMessaging_SetValue(Can_PEDALS_Brake_Implausibility, cnt & 1);

		CanMessaging_SetValue(Can_INVERTERS_LeftInverterTemperature, Can_INVERTERS_LeftInverterTemperature + cnt);
		CanMessaging_SetValue(Can_INVERTERS_LeftMotorTemperature, Can_INVERTERS_LeftMotorTemperature + cnt);
		CanMessaging_SetValue(Can_INVERTERS_LeftInverterInputVoltage, Can_INVERTERS_LeftInverterInputVoltage + cnt);
		CanMessaging_SetValue(Can_INVERTERS_LeftInverterCurrent, Can_INVERTERS_LeftInverterCurrent + cnt);
		CanMessaging_SetValue(Can_INVERTERS_LeftMotorRpm, Can_INVERTERS_LeftMotorRpm + cnt);
		CanMessaging_SetValue(Can_INVERTERS_LeftMotorSpeedKmh, Can_INVERTERS_LeftMotorSpeedKmh + cnt);
		CanMessaging_SetValue(Can_INVERTERS_LeftInverterThrottle, Can_INVERTERS_LeftInverterThrottle + cnt);
		CanMessaging_SetValue(Can_INVERTERS_LeftInverterThrottleFeedback, Can_INVERTERS_LeftInverterThrottleFeedback + cnt);
		CanMessaging_SetValue(Can_INVERTERS_RightInverterTemperature, Can_INVERTERS_RightInverterTemperature + cnt);
		CanMessaging_SetValue(Can_INVERTERS_RightMotorTemperature, Can_INVERTERS_RightMotorTemperature + cnt);
		CanMessaging_SetValue(Can_INVERTERS_RightInverterInputVoltage, Can_INVERTERS_RightInverterInputVoltage + cnt);
		CanMessaging_SetValue(Can_INVERTERS_RightInverterCurrent, Can_INVERTERS_RightInverterCurrent + cnt);
		CanMessaging_SetValue(Can_INVERTERS_RightMotorRpm, Can_INVERTERS_RightMotorRpm + cnt);
		CanMessaging_SetValue(Can_INVERTERS_RightMotorSpeedKmh, Can_INVERTERS_RightMotorSpeedKmh + cnt);
		CanMessaging_SetValue(Can_INVERTERS_RightInverterSentThrottle, Can_INVERTERS_RightInverterSentThrottle + cnt);
		CanMessaging_SetValue(Can_INVERTERS_RightInverterThrottleFeedback, Can_INVERTERS_RightInverterThrottleFeedback + cnt);
		CanMessaging_SetValue(Can_INVERTERS_IsCarInReverse, cnt & 1);
		CanMessaging_SetValue(Can_INVERTERS_IsCarRunning, cnt & 1);

		CanMessaging_SetValue(Can_DASHBOARD_ActivationButtonPressed, cnt & 1);
		CanMessaging_SetValue(Can_DASHBOARD_CarReverseCommandPressed, cnt & 1);
		CanMessaging_SetValue(Can_DASHBOARD_IsDisplayWorking, cnt & 1);
		CanMessaging_SetValue(Can_DASHBOARD_IsSegmentsDriverWorking, cnt & 1);

		cnt++;
		CanMessaging_Update();
		i=100000;
		while(i--);
	}
}

void CanMessaging_Update(void){

	Can_PduType pduInfo;
	pduInfo.swPduHandle=0;
	pduInfo.length=8;

	CanMessaging_CreateBuffer(idCanInvertorStanga);
	pduInfo.sdu=bufferCan;
	pduInfo.id=idCanInvertorStanga | 0x80000000U;
	Can_43_FLEXCAN_Write(CAN_HTH_HANDLE, &pduInfo);

	CanMessaging_CreateBuffer(idCanInvertorDreapta);
	pduInfo.sdu=bufferCan;
	pduInfo.id=idCanInvertorDreapta | 0x80000000U;
	Can_43_FLEXCAN_Write(CAN_HTH_HANDLE, &pduInfo);

	CanMessaging_CreateBuffer(idCanInvertoare);
	pduInfo.sdu=bufferCan;
	pduInfo.id=idCanInvertoare | 0x80000000U;
	Can_43_FLEXCAN_Write(CAN_HTH_HANDLE, &pduInfo);

	CanMessaging_CreateBuffer(idCanBord);
	pduInfo.sdu=bufferCan;
	pduInfo.id=idCanBord | 0x80000000U;
	Can_43_FLEXCAN_Write(CAN_HTH_HANDLE, &pduInfo);

	CanMessaging_CreateBuffer(idCanAcceleratie);
	pduInfo.sdu=bufferCan;
	pduInfo.id=idCanAcceleratie | 0x80000000U;
	Can_43_FLEXCAN_Write(CAN_HTH_HANDLE, &pduInfo);

	CanMessaging_CreateBuffer(idCanFrana);
	pduInfo.sdu=bufferCan;
	pduInfo.id=idCanFrana | 0x80000000U;
	Can_43_FLEXCAN_Write(CAN_HTH_HANDLE, &pduInfo);

	CanMessaging_CreateBuffer(idCanBaterie);
	pduInfo.sdu=bufferCan;
	pduInfo.id=idCanBaterie | 0x80000000U;
	Can_43_FLEXCAN_Write(CAN_HTH_HANDLE, &pduInfo);
}

void CanMessaging_SetValue(CanMonitoredValue_t DesiredValueType, uint32_t Value){
	switch(DesiredValueType){
		//TSAC
		case Can_TSAC_MedianCellTemperature:
			baterieCan.MedianCellTemperature = Value;
			break;
		case Can_TSAC_HighestCellTemperature:
			baterieCan.HighestCellTemperature = Value;
			break;
		case Can_TSAC_LowestCellTemperature:
			baterieCan.LowestCellTemperature = Value;
			break;
		case Can_TSAC_MedianCellVoltage:
			baterieCan.MedianCellVoltage = Value;
			break;
		case Can_TSAC_HighestCellVoltage:
			baterieCan.HighestCellVoltage = Value;
			break;
		case Can_TSAC_LowestCellVoltage:
			baterieCan.LowestCellVoltage = Value;
			break;
		case Can_TSAC_OverallVoltage:
			baterieCan.OverallVoltage = Value;
			break;
		case Can_TSAC_OverallCurrent:
			if(Value>8095)
				baterieCan.OverallCurrent = 0;
			else
				baterieCan.OverallCurrent = Value;
			break;
		case Can_TSAC_IsAmsSafe:
			baterieCan.AmsError = Value;
			break;
		case Can_TSAC_IsImdSafe:
			baterieCan.ImdError = Value;
			break;
		case Can_TSAC_IsTransceiverWorking:
			baterieCan.TransceiverError = Value;
			break;
		case Can_TSAC_IsShuntWorking:
			baterieCan.ShuntError = Value;
			break;
		case Can_TSAC_IsBms0Working:
			baterieCan.Bms0Error = Value;
			break;
		case Can_TSAC_IsBms1Working:
			baterieCan.Bms1Error = Value;
			break;
		//PEDALS
		case Can_PEDALS_AcceleratorSensor1Voltage:
			pedaleCan.AcceleratorSensor1Voltage = Value;
			break;
		case Can_PEDALS_AcceleratorSensor2Voltage:
			pedaleCan.AcceleratorSensor2Voltage = Value;
			break;
		case Can_PEDALS_AcceleratorSensor1TravelPercentage:
			if(Value>100)
				pedaleCan.AcceleratorSensor1TravelPercentage = 0;
			else
				pedaleCan.AcceleratorSensor1TravelPercentage = Value;
			break;
		case Can_PEDALS_AcceleratorSensor2TravelPercentage:
			if(Value>100)
				pedaleCan.AcceleratorSensor2TravelPercentage = 0;
			else
				pedaleCan.AcceleratorSensor2TravelPercentage = Value;
			break;
		case Can_PEDALS_BrakeSensor1Voltage:
			pedaleCan.BrakeSensor1Voltage = Value;
			break;
		case Can_PEDALS_BrakeSensor2Voltage:
			pedaleCan.BrakeSensor2Voltage = Value;
			break;
		case Can_PEDALS_BrakeSensor1TravelPercentage:
			if(Value>100)
				pedaleCan.BrakeSensor1TravelPercentage = 0;
			else
				pedaleCan.BrakeSensor1TravelPercentage = Value;
			break;
		case Can_PEDALS_BrakeSensor2TravelPercentage:
			if(Value>100)
				pedaleCan.BrakeSensor2TravelPercentage = 0;
			else
				pedaleCan.BrakeSensor2TravelPercentage = Value;
			break;
		case Can_PEDALS_PressureSensorVoltage:
			if(Value>500)
				pedaleCan.PressureSensorVoltage = 0;
			else
				pedaleCan.PressureSensorVoltage = Value;
			break;
		case Can_PEDALS_PressureSensorBars:
			pedaleCan.PressureSensorBars = Value;
			break;
		case Can_PEDALS_Accel_Sensor1_ShortToGnd:
			pedaleCan.Accel_Sensor1_ShortToGnd = Value;
			break;
		case Can_PEDALS_Accel_Sensor1_ShortToVcc:
			pedaleCan.Accel_Sensor1_ShortToVcc = Value;
			break;
		case Can_PEDALS_Accel_Sensor1_OutOfRangeOutput:
			pedaleCan.Accel_Sensor1_OutOfRangeOutput = Value;
			break;
		case Can_PEDALS_Accel_Sensor2_ShortToGnd:
			pedaleCan.Accel_Sensor2_ShortToGnd = Value;
			break;
		case Can_PEDALS_Accel_Sensor2_ShortToVcc:
			pedaleCan.Accel_Sensor2_ShortToVcc = Value;
			break;
		case Can_PEDALS_Accel_Sensor2_OutOfRangeOutput:
			pedaleCan.Accel_Sensor2_OutOfRangeOutput = Value;
			break;
		case Can_PEDALS_Accel_Implausibility:
			pedaleCan.Accel_Implausibility = Value;
			break;
		case Can_PEDALS_Brake_Sensor1_ShortToGnd:
			pedaleCan.Brake_Sensor1_ShortToGnd = Value;
			break;
		case Can_PEDALS_Brake_Sensor1_ShortToVcc:
			pedaleCan.Brake_Sensor1_ShortToVcc = Value;
			break;
		case Can_PEDALS_Brake_Sensor1_OutOfRangeOutput:
			pedaleCan.Brake_Sensor1_OutOfRangeOutput = Value;
			break;
		case Can_PEDALS_Brake_Sensor2_ShortToGnd:
			pedaleCan.Brake_Sensor2_ShortToGnd = Value;
			break;
		case Can_PEDALS_Brake_Sensor2_ShortToVcc:
			pedaleCan.Brake_Sensor2_ShortToVcc = Value;
			break;
		case Can_PEDALS_Brake_Sensor2_OutOfRangeOutput:
			pedaleCan.Brake_Sensor2_OutOfRangeOutput = Value;
			break;
		case Can_PEDALS_Brake_Implausibility:
			pedaleCan.Brake_Implausibility = Value;
			break;
		//INVERTERS
		case Can_INVERTERS_LeftInverterTemperature:
			invertoareCan.LeftInverterTemperature = Value;
			break;
		case Can_INVERTERS_LeftMotorTemperature:
			invertoareCan.LeftMotorTemperature = Value;
			break;
		case Can_INVERTERS_LeftInverterInputVoltage:
			if(Value>1800)
				invertoareCan.LeftInverterInputVoltage = 0;
			else
				invertoareCan.LeftInverterInputVoltage = Value;
			break;
		case Can_INVERTERS_LeftInverterCurrent:
			if(Value>4000)
				invertoareCan.LeftInverterCurrent = 0;
			else
				invertoareCan.LeftInverterCurrent = Value;
			break;
		case Can_INVERTERS_LeftMotorRpm:
			if(Value>6000)
				invertoareCan.LeftMotorRpm = 0;
			else
				invertoareCan.LeftMotorRpm = Value;
			break;
		case Can_INVERTERS_LeftMotorSpeedKmh:
			invertoareCan.LeftMotorSpeedKmh = Value;
			break;
		case Can_INVERTERS_LeftInverterThrottle:
			if(Value>250)
				invertoareCan.LeftInverterThrottle = 0;
			else
				invertoareCan.LeftInverterThrottle = Value;
			break;
		case Can_INVERTERS_LeftInverterThrottleFeedback:
			if(Value>250)
				invertoareCan.LeftInverterThrottleFeedback = 0;
			else
				invertoareCan.LeftInverterThrottleFeedback = Value;
			break;
		case Can_INVERTERS_RightInverterTemperature:
			invertoareCan.RightInverterTemperature = Value;
			break;
		case Can_INVERTERS_RightMotorTemperature:
			invertoareCan.RightMotorTemperature = Value;
			break;
		case Can_INVERTERS_RightInverterInputVoltage:
			if(Value>1800)
				invertoareCan.RightInverterInputVoltage = 0;
			else
				invertoareCan.RightInverterInputVoltage = Value;
			break;
		case Can_INVERTERS_RightInverterCurrent:
			if(Value>4000)
				invertoareCan.RightInverterCurrent = 0;
			else
				invertoareCan.RightInverterCurrent = Value;
			break;
		case Can_INVERTERS_RightMotorRpm:
			if(Value>6000)
				invertoareCan.RightMotorRpm = 0;
			else
				invertoareCan.RightMotorRpm = Value;
			break;
		case Can_INVERTERS_RightMotorSpeedKmh:
			invertoareCan.RightMotorSpeedKmh = Value;
			break;
		case Can_INVERTERS_RightInverterSentThrottle:
			if(Value>250)
				invertoareCan.RightInverterSentThrottle = 0;
			else
				invertoareCan.RightInverterSentThrottle = Value;
			break;
		case Can_INVERTERS_RightInverterThrottleFeedback:
			if(Value>250)
				invertoareCan.RightInverterThrottleFeedback = 0;
			else
				invertoareCan.RightInverterThrottleFeedback = Value;
			break;
		case Can_INVERTERS_IsCarInReverse:
			invertoareCan.IsCarInReverse = Value;
			break;
		case Can_INVERTERS_IsCarRunning:
			invertoareCan.IsCarRunning = Value;
			break;
		//DASHBOARD
		case Can_DASHBOARD_ActivationButtonPressed:
			bordCan.ActivationButtonPressed = Value;
			break;
		case Can_DASHBOARD_CarReverseCommandPressed:
			bordCan.CarReverseCommandPressed = Value;
			break;
		case Can_DASHBOARD_IsDisplayWorking:
			bordCan.IsDisplayWorking = Value;
			break;
		case Can_DASHBOARD_IsSegmentsDriverWorking:
			bordCan.IsSegmentsDriverWorking = Value;
			break;
	}
}

uint32_t CanMessaging_ReadValue(CanMonitoredValue_t DesiredValueType){
	switch(DesiredValueType){
		case Can_TSAC_MedianCellTemperature:
			return baterieCan.MedianCellTemperature;
	    case Can_TSAC_HighestCellTemperature:
	    	return baterieCan.HighestCellTemperature;
	    case Can_TSAC_LowestCellTemperature:
	    	return baterieCan.LowestCellTemperature;
	    case Can_TSAC_MedianCellVoltage:
	    	return baterieCan.MedianCellVoltage;
	    case Can_TSAC_HighestCellVoltage:
	    	return baterieCan.HighestCellVoltage;
	    case Can_TSAC_LowestCellVoltage:
	    	return baterieCan.LowestCellVoltage;
	    case Can_TSAC_OverallVoltage:
	    	return baterieCan.OverallVoltage;
	    case Can_TSAC_OverallCurrent:
	    	return baterieCan.OverallCurrent;
	    case Can_TSAC_IsAmsSafe:
	    	return baterieCan.AmsError;
	    case Can_TSAC_IsImdSafe:
	    	return baterieCan.ImdError;
	    case Can_TSAC_IsTransceiverWorking:
	    	return baterieCan.TransceiverError;
	    case Can_TSAC_IsShuntWorking:
	    	return baterieCan.ShuntError;
	    case Can_TSAC_IsBms0Working:
	    	return baterieCan.Bms0Error;
	    case Can_TSAC_IsBms1Working:
	    	return baterieCan.Bms1Error;
	    case Can_PEDALS_AcceleratorSensor1Voltage:
	    	return pedaleCan.AcceleratorSensor1Voltage;
	    case Can_PEDALS_AcceleratorSensor2Voltage:
	    	return pedaleCan.AcceleratorSensor2Voltage;
		case Can_PEDALS_AcceleratorSensor1TravelPercentage:
			return pedaleCan.AcceleratorSensor1TravelPercentage;
		case Can_PEDALS_AcceleratorSensor2TravelPercentage:
			return pedaleCan.AcceleratorSensor2TravelPercentage;
		case Can_PEDALS_BrakeSensor1Voltage:
			return pedaleCan.BrakeSensor1Voltage;
		case Can_PEDALS_BrakeSensor2Voltage:
			return pedaleCan.BrakeSensor2Voltage;
		case Can_PEDALS_BrakeSensor1TravelPercentage:
			return pedaleCan.BrakeSensor1TravelPercentage;
		case Can_PEDALS_BrakeSensor2TravelPercentage:
			return pedaleCan.BrakeSensor2TravelPercentage;
		case Can_PEDALS_PressureSensorVoltage:
			return pedaleCan.PressureSensorVoltage;
		case Can_PEDALS_PressureSensorBars:
			return pedaleCan.PressureSensorBars;
		case Can_PEDALS_Accel_Sensor1_ShortToGnd:
			return pedaleCan.Accel_Sensor1_ShortToGnd;
		case Can_PEDALS_Accel_Sensor1_ShortToVcc:
			return pedaleCan.Accel_Sensor1_ShortToVcc;
		case Can_PEDALS_Accel_Sensor1_OutOfRangeOutput:
			return pedaleCan.Accel_Sensor1_OutOfRangeOutput;
		case Can_PEDALS_Accel_Sensor2_ShortToGnd:
			return pedaleCan.Accel_Sensor2_ShortToGnd;
		case Can_PEDALS_Accel_Sensor2_ShortToVcc:
			return pedaleCan.Accel_Sensor2_ShortToVcc;
		case Can_PEDALS_Accel_Sensor2_OutOfRangeOutput:
			return pedaleCan.Accel_Sensor2_OutOfRangeOutput;
		case Can_PEDALS_Accel_Implausibility:
			return pedaleCan.Accel_Implausibility;
		case Can_PEDALS_Brake_Sensor1_ShortToGnd:
			return pedaleCan.Brake_Sensor1_ShortToGnd;
		case Can_PEDALS_Brake_Sensor1_ShortToVcc:
			return pedaleCan.Brake_Sensor1_ShortToVcc;
		case Can_PEDALS_Brake_Sensor1_OutOfRangeOutput:
			return pedaleCan.Brake_Sensor1_OutOfRangeOutput;
		case Can_PEDALS_Brake_Sensor2_ShortToGnd:
			return pedaleCan.Brake_Sensor2_ShortToGnd;
		case Can_PEDALS_Brake_Sensor2_ShortToVcc:
			return pedaleCan.Brake_Sensor2_ShortToVcc;
		case Can_PEDALS_Brake_Sensor2_OutOfRangeOutput:
			return pedaleCan.Brake_Sensor2_OutOfRangeOutput;
		case Can_PEDALS_Brake_Implausibility:
			return pedaleCan.Brake_Implausibility;
		case Can_INVERTERS_LeftInverterTemperature:
			return invertoareCan.LeftInverterTemperature;
		case Can_INVERTERS_LeftMotorTemperature:
			return invertoareCan.LeftMotorTemperature;
		case Can_INVERTERS_LeftInverterInputVoltage:
			return invertoareCan.LeftInverterInputVoltage;
		case Can_INVERTERS_LeftInverterCurrent:
			return invertoareCan.LeftInverterCurrent;
		case Can_INVERTERS_LeftMotorRpm:
			return invertoareCan.LeftMotorRpm;
		case Can_INVERTERS_LeftMotorSpeedKmh:
			return invertoareCan.LeftMotorSpeedKmh;
		case Can_INVERTERS_LeftInverterThrottle:
			return invertoareCan.LeftInverterThrottle;
		case Can_INVERTERS_LeftInverterThrottleFeedback:
			return invertoareCan.LeftInverterThrottleFeedback;
		case Can_INVERTERS_RightInverterTemperature:
			return invertoareCan.RightInverterTemperature;
		case Can_INVERTERS_RightMotorTemperature:
			return invertoareCan.RightMotorTemperature;
		case Can_INVERTERS_RightInverterInputVoltage:
			return invertoareCan.RightInverterInputVoltage;
		case Can_INVERTERS_RightInverterCurrent:
			return invertoareCan.RightInverterCurrent;
		case Can_INVERTERS_RightMotorRpm:
			return invertoareCan.RightMotorRpm;
		case Can_INVERTERS_RightMotorSpeedKmh:
			return invertoareCan.RightMotorSpeedKmh;
		case Can_INVERTERS_RightInverterSentThrottle:
			return invertoareCan.RightInverterSentThrottle;
		case Can_INVERTERS_RightInverterThrottleFeedback:
			return invertoareCan.RightInverterThrottleFeedback;
		case Can_INVERTERS_IsCarInReverse:
			return invertoareCan.IsCarInReverse;
		case Can_INVERTERS_IsCarRunning:
			return invertoareCan.IsCarRunning;
		case Can_DASHBOARD_ActivationButtonPressed:
			return bordCan.ActivationButtonPressed;
		case Can_DASHBOARD_CarReverseCommandPressed:
			return bordCan.CarReverseCommandPressed;
		case Can_DASHBOARD_IsDisplayWorking:
			return bordCan.IsDisplayWorking;
		case Can_DASHBOARD_IsSegmentsDriverWorking:
			return bordCan.IsSegmentsDriverWorking;
	}
	return 0;
}

boolean CanMessaging_ReceiveData(Can_HwHandleType handle, Can_IdType id, PduLengthType length, uint8_t* data){
	switch((id&MASK)){
		case idCanFrana:
			//extragere date
			UartMessaging_SetValue(Uart_PEDALS_BrakeSensor1Voltage, ((((uint16_t)data[6])<<8) | data[7]) & (0x3FFF));
			UartMessaging_SetValue(Uart_PEDALS_BrakeSensor2Voltage, ((((((uint16_t)data[4])<<8) | data[5]) & (0x0FFF)) << 2) | (data[6]>>6));
			UartMessaging_SetValue(Uart_PEDALS_BrakeSensor1TravelPercentage, (((uint8_t)(data[3]<<4)) | (data[4]>>4)) & (0x7F));
			UartMessaging_SetValue(Uart_PEDALS_BrakeSensor2TravelPercentage, (((uint8_t)(data[2]<<5)) | (data[3]>>3)) & (0x7F));
			UartMessaging_SetValue(Uart_PEDALS_PressureSensorBars, ((uint8_t)(data[1]<<6)) | (data[2]>>2));
			UartMessaging_SetValue(Uart_PEDALS_Brake_Implausibility, (data[0] & (1<<1)) >> 1);
			UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor1_OutOfRangeOutput, (data[0] & (1<<5)) >> 5);
			UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor1_ShortToVcc, (data[0] & (1<<6)) >> 6);
			UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor1_ShortToGnd, (data[0] & (1<<7)) >> 7);
			UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor2_OutOfRangeOutput, (data[0] & (1<<2)) >> 2);
			UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor2_ShortToVcc, (data[0] & (1<<3)) >> 3);
			UartMessaging_SetValue(Uart_PEDALS_Brake_Sensor2_ShortToGnd, (data[0] & (1<<4)) >> 4);

			CanMessaging_SetValue(Can_PEDALS_BrakeSensor1Voltage, ((((uint16_t)data[6])<<8) | data[7]) & (0x3FFF));
			CanMessaging_SetValue(Can_PEDALS_BrakeSensor2Voltage, ((((((uint16_t)data[4])<<8) | data[5]) & (0x0FFF)) << 2) | (data[6]>>6));
			CanMessaging_SetValue(Can_PEDALS_BrakeSensor1TravelPercentage, (((uint8_t)(data[3]<<4)) | (data[4]>>4)) & (0x7F));
			CanMessaging_SetValue(Can_PEDALS_BrakeSensor2TravelPercentage, (((uint8_t)(data[2]<<5)) | (data[3]>>3)) & (0x7F));
			CanMessaging_SetValue(Can_PEDALS_PressureSensorBars, ((uint8_t)(data[1]<<6)) | (data[2]>>2));
			CanMessaging_SetValue(Can_PEDALS_Brake_Implausibility, (data[0] & (1<<1)) >> 1);
			CanMessaging_SetValue(Can_PEDALS_Brake_Sensor1_OutOfRangeOutput, (data[0] & (1<<5)) >> 5);
			CanMessaging_SetValue(Can_PEDALS_Brake_Sensor1_ShortToVcc, (data[0] & (1<<6)) >> 6);
			CanMessaging_SetValue(Can_PEDALS_Brake_Sensor1_ShortToGnd, (data[0] & (1<<7)) >> 7);
			CanMessaging_SetValue(Can_PEDALS_Brake_Sensor2_OutOfRangeOutput, (data[0] & (1<<2)) >> 2);
			CanMessaging_SetValue(Can_PEDALS_Brake_Sensor2_ShortToVcc, (data[0] & (1<<3)) >> 3);
			CanMessaging_SetValue(Can_PEDALS_Brake_Sensor2_ShortToGnd, (data[0] & (1<<4)) >> 4);
			break;

		case idCanAcceleratie:
			//extragere date
			UartMessaging_SetValue(Uart_PEDALS_AcceleratorSensor1Voltage, ((((uint16_t)data[6])<<8) | data[7]) & (0x3FFF));
			UartMessaging_SetValue(Uart_PEDALS_AcceleratorSensor2Voltage, ((((((uint16_t)data[4])<<8) | data[5]) & (0x0FFF)) << 2) | (data[6]>>6));
			UartMessaging_SetValue(Uart_PEDALS_AcceleratorSensor1TravelPercentage, (((uint8_t)(data[3]<<4)) | (data[4]>>4)) & (0x7F));
			UartMessaging_SetValue(Uart_PEDALS_AcceleratorSensor2TravelPercentage, (((uint8_t)(data[2]<<5)) | (data[3]>>3)) & (0x7F));
			UartMessaging_SetValue(Uart_PEDALS_PressureSensorVoltage, ((((uint16_t)data[1]<<6)) | (data[2]>>2)) & (0x01FF));
			UartMessaging_SetValue(Uart_PEDALS_Accel_Implausibility, (data[0] & (1<<1)) >> 1);
			UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor1_OutOfRangeOutput, (data[0] & (1<<5)) >> 5);
			UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor1_ShortToVcc, (data[0] & (1<<6)) >> 6);
			UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor1_ShortToGnd, (data[0] & (1<<7)) >> 7);
			UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor2_OutOfRangeOutput, (data[0] & (1<<2)) >> 2);
			UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor2_ShortToVcc, (data[0] & (1<<3)) >> 3);
			UartMessaging_SetValue(Uart_PEDALS_Accel_Sensor2_ShortToGnd, (data[0] & (1<<4)) >>4);

			CanMessaging_SetValue(Can_PEDALS_AcceleratorSensor1Voltage, ((((uint16_t)data[6])<<8) | data[7]) & (0x3FFF));
			CanMessaging_SetValue(Can_PEDALS_AcceleratorSensor2Voltage, ((((((uint16_t)data[4])<<8) | data[5]) & (0x0FFF)) << 2) | (data[6]>>6));
			CanMessaging_SetValue(Can_PEDALS_AcceleratorSensor1TravelPercentage, (((uint8_t)(data[3]<<4)) | (data[4]>>4)) & (0x7F));
			CanMessaging_SetValue(Can_PEDALS_AcceleratorSensor2TravelPercentage, (((uint8_t)(data[2]<<5)) | (data[3]>>3)) & (0x7F));
			CanMessaging_SetValue(Can_PEDALS_PressureSensorVoltage, ((((uint16_t)data[1]<<6)) | (data[2]>>2)) & (0x01FF));
			CanMessaging_SetValue(Can_PEDALS_Accel_Implausibility, (data[0] & (1<<1)) >> 1);
			CanMessaging_SetValue(Can_PEDALS_Accel_Sensor1_OutOfRangeOutput, (data[0] & (1<<5)) >> 5);
			CanMessaging_SetValue(Can_PEDALS_Accel_Sensor1_ShortToVcc, (data[0] & (1<<6)) >> 6);
			CanMessaging_SetValue(Can_PEDALS_Accel_Sensor1_ShortToGnd, (data[0] & (1<<7)) >> 7);
			CanMessaging_SetValue(Can_PEDALS_Accel_Sensor2_OutOfRangeOutput, (data[0] & (1<<2)) >> 2);
			CanMessaging_SetValue(Can_PEDALS_Accel_Sensor2_ShortToVcc, (data[0] & (1<<3)) >> 3);
			CanMessaging_SetValue(Can_PEDALS_Accel_Sensor2_ShortToGnd, (data[0] & (1<<4)) >>4);
			break;

		case idCanInvertorStanga:
			//extragere date
			UartMessaging_SetValue(Uart_INVERTERS_LeftMotorTemperature, data[7]);
			UartMessaging_SetValue(Uart_INVERTERS_LeftInverterTemperature, data[6]);
			UartMessaging_SetValue(Uart_INVERTERS_LeftInverterThrottle, data[5]);
			UartMessaging_SetValue(Uart_INVERTERS_LeftMotorSpeedKmh, data[4]);
			UartMessaging_SetValue(Uart_INVERTERS_LeftInverterThrottleFeedback, data[3]);
			UartMessaging_SetValue(Uart_INVERTERS_LeftInverterInputVoltage, ((((uint16_t)data[1])<<8) | data[2]) & (0x07FF));
			UartMessaging_SetValue(Uart_INVERTERS_LeftMotorRpm, ((((uint16_t)data[0])<<8) | data[1]) >> 3);

			CanMessaging_SetValue(Can_INVERTERS_LeftMotorTemperature, data[7]);
			CanMessaging_SetValue(Can_INVERTERS_LeftInverterTemperature, data[6]);
			CanMessaging_SetValue(Can_INVERTERS_LeftInverterThrottle, data[5]);
			CanMessaging_SetValue(Can_INVERTERS_LeftMotorSpeedKmh, data[4]);
			CanMessaging_SetValue(Can_INVERTERS_LeftInverterThrottleFeedback, data[3]);
			CanMessaging_SetValue(Can_INVERTERS_LeftInverterInputVoltage, ((((uint16_t)data[1])<<8) | data[2]) & (0x07FF));
			CanMessaging_SetValue(Can_INVERTERS_LeftMotorRpm, ((((uint16_t)data[0])<<8) | data[1]) >> 3);
			break;

		case idCanInvertorDreapta:
			//extragere date
			UartMessaging_SetValue(Uart_INVERTERS_RightMotorTemperature, data[7]);
			UartMessaging_SetValue(Uart_INVERTERS_RightInverterTemperature, data[6]);
			UartMessaging_SetValue(Uart_INVERTERS_RightInverterSentThrottle, data[5]);
			UartMessaging_SetValue(Uart_INVERTERS_RightMotorSpeedKmh, data[4]);
			UartMessaging_SetValue(Uart_INVERTERS_RightInverterThrottleFeedback, data[3]);
			UartMessaging_SetValue(Uart_INVERTERS_RightInverterInputVoltage, ((((uint16_t)data[1])<<8) | data[2]) & (0x7FF));
			UartMessaging_SetValue(Uart_INVERTERS_RightMotorRpm, ((((uint16_t)data[0])<<8) | data[1]) >> 3);

			CanMessaging_SetValue(Can_INVERTERS_RightMotorTemperature, data[7]);
			CanMessaging_SetValue(Can_INVERTERS_RightInverterTemperature, data[6]);
			CanMessaging_SetValue(Can_INVERTERS_RightInverterSentThrottle, data[5]);
			CanMessaging_SetValue(Can_INVERTERS_RightMotorSpeedKmh, data[4]);
			CanMessaging_SetValue(Can_INVERTERS_RightInverterThrottleFeedback, data[3]);
			CanMessaging_SetValue(Can_INVERTERS_RightInverterInputVoltage, ((((uint16_t)data[1])<<8) | data[2]) & (0x7FF));
			CanMessaging_SetValue(Can_INVERTERS_RightMotorRpm, ((((uint16_t)data[0])<<8) | data[1]) >> 3);
			break;

		case idCanInvertoare:
			UartMessaging_SetValue(Uart_INVERTERS_IsCarRunning, (data[0] & (1<<7)) >> 7);
			UartMessaging_SetValue(Uart_INVERTERS_IsCarInReverse, (data[0] & (1<<6)) >> 6);
			UartMessaging_SetValue(Uart_INVERTERS_LeftInverterCurrent, ((((uint16_t)data[6])<<8) | data[7]) & (0x0FFF));
			UartMessaging_SetValue(Uart_INVERTERS_RightInverterCurrent, ((((uint16_t)data[5])<<8) | data[6]) >> 4);

			CanMessaging_SetValue(Can_INVERTERS_IsCarRunning, (data[0] & (1<<7)) >> 7);
			CanMessaging_SetValue(Can_INVERTERS_IsCarInReverse, (data[0] & (1<<6)) >> 6);
			CanMessaging_SetValue(Can_INVERTERS_LeftInverterCurrent, ((((uint16_t)data[6])<<8) | data[7]) & (0x0FFF));
			CanMessaging_SetValue(Can_INVERTERS_RightInverterCurrent, ((((uint16_t)data[5])<<8) | data[6]) >> 4);
			break;

		case idCanBaterie:
			//extragere date
			UartMessaging_SetValue(Uart_TSAC_OverallCurrent, ((((uint16_t)data[6])<<8) | data[7]) & (0x1FFF));
			UartMessaging_SetValue(Uart_TSAC_OverallVoltage, ((((uint16_t)data[5])<<8) | data[6]) >> 5);
			UartMessaging_SetValue(Uart_TSAC_HighestCellTemperature, ((((uint16_t)data[3])<<8) | data[4]) & (0x03FF));
			UartMessaging_SetValue(Uart_TSAC_HighestCellVoltage, (((((uint16_t)data[2])<<8) | data[3]) >> 2) & (0x03FF));

			CanMessaging_SetValue(Can_TSAC_OverallCurrent, ((((uint16_t)data[6])<<8) | data[7]) & (0x1FFF));
			CanMessaging_SetValue(Can_TSAC_OverallVoltage, ((((uint16_t)data[5])<<8) | data[6]) >> 5);
			CanMessaging_SetValue(Can_TSAC_HighestCellTemperature, ((((uint16_t)data[3])<<8) | data[4]) & (0x03FF));
			CanMessaging_SetValue(Can_TSAC_HighestCellVoltage, (((((uint16_t)data[2])<<8) | data[3]) >> 2) & (0x03FF));

			//More To Come:)
			break;

		case idCanBord:
			//extragere date
			UartMessaging_SetValue(Uart_DASHBOARD_ActivationButtonPressed, (data[0] & (1<<7)) >> 7);
			UartMessaging_SetValue(Uart_DASHBOARD_CarReverseCommandPressed, (data[0] & (1<<6)) >> 6);
			UartMessaging_SetValue(Uart_DASHBOARD_IsDisplayWorking, (data[0] & (1<<5)) >> 5);
			UartMessaging_SetValue(Uart_DASHBOARD_IsSegmentsDriverWorking, (data[0] & (1<<4)) >> 4);

			CanMessaging_SetValue(Can_DASHBOARD_ActivationButtonPressed, (data[0] & (1<<7)) >> 7);
			CanMessaging_SetValue(Can_DASHBOARD_CarReverseCommandPressed, (data[0] & (1<<6)) >> 6);
			CanMessaging_SetValue(Can_DASHBOARD_IsDisplayWorking, (data[0] & (1<<5)) >> 5);
			CanMessaging_SetValue(Can_DASHBOARD_IsSegmentsDriverWorking, (data[0] & (1<<4)) >> 4);
			break;
		default:
			return FALSE;
	}
	return TRUE;
}

void CanMessaging_CreateBuffer(idCan_t type){
	switch(type){
		case idCanInvertorStanga:
			bufferCan[0] = CanMessaging_ReadValue(Can_INVERTERS_LeftMotorRpm) >> 5;
			bufferCan[1] = ((CanMessaging_ReadValue(Can_INVERTERS_LeftMotorRpm) & (0x00F8))) | ((CanMessaging_ReadValue(Can_INVERTERS_LeftInverterInputVoltage) & (0x0700)) >> 8);
			bufferCan[2] = ((uint8_t) CanMessaging_ReadValue(Can_INVERTERS_LeftInverterInputVoltage) & (0x00FF));
			bufferCan[3] = CanMessaging_ReadValue(Can_INVERTERS_LeftInverterThrottleFeedback);
			bufferCan[4] = CanMessaging_ReadValue(Can_INVERTERS_LeftMotorSpeedKmh);
			bufferCan[5] = CanMessaging_ReadValue(Can_INVERTERS_LeftInverterThrottle);
			bufferCan[6] = CanMessaging_ReadValue(Can_INVERTERS_LeftInverterTemperature);
			bufferCan[7] = CanMessaging_ReadValue(Can_INVERTERS_LeftMotorTemperature);
			break;
		case idCanInvertorDreapta:
			bufferCan[0] = CanMessaging_ReadValue(Can_INVERTERS_RightMotorRpm) >> 5;
			bufferCan[1] = ((CanMessaging_ReadValue(Can_INVERTERS_RightMotorRpm) & (0x00F8))) | ((CanMessaging_ReadValue(Can_INVERTERS_RightInverterInputVoltage) & (0x700)) >> 8);
			bufferCan[2] = ((uint8_t) CanMessaging_ReadValue(Can_INVERTERS_RightInverterInputVoltage) & (0x7FF));
			bufferCan[3] = CanMessaging_ReadValue(Can_INVERTERS_RightInverterThrottleFeedback);
			bufferCan[4] = CanMessaging_ReadValue(Can_INVERTERS_RightMotorSpeedKmh);
			bufferCan[5] = CanMessaging_ReadValue(Can_INVERTERS_RightInverterSentThrottle);
			bufferCan[6] = CanMessaging_ReadValue(Can_INVERTERS_RightInverterTemperature);
			bufferCan[7] = CanMessaging_ReadValue(Can_INVERTERS_RightMotorTemperature);
			break;
		case idCanInvertoare:
			bufferCan[0] = (CanMessaging_ReadValue(Can_INVERTERS_IsCarRunning) << 7) | (CanMessaging_ReadValue(Can_INVERTERS_IsCarInReverse) << 6);
			bufferCan[1] = 0;
			bufferCan[2] = 0;
			bufferCan[3] = 0;
			bufferCan[4] = 0;
			bufferCan[5] = (CanMessaging_ReadValue(Can_INVERTERS_RightInverterCurrent) & (0x0FF0)) >> 4;
			bufferCan[6] = ((CanMessaging_ReadValue(Can_INVERTERS_RightInverterCurrent) & (0x000F)) << 4) | ((CanMessaging_ReadValue(Can_INVERTERS_LeftInverterCurrent) & (0x0F00)) >> 8);
			bufferCan[7] = CanMessaging_ReadValue(Can_INVERTERS_LeftInverterCurrent) & (0x00FF);
			break;
		case idCanBord:
			bufferCan[0] = (CanMessaging_ReadValue(Can_DASHBOARD_ActivationButtonPressed) << 7) | (CanMessaging_ReadValue(Can_DASHBOARD_CarReverseCommandPressed) << 6) | (CanMessaging_ReadValue(Can_DASHBOARD_IsDisplayWorking) << 5) | (CanMessaging_ReadValue(Can_DASHBOARD_IsSegmentsDriverWorking) << 4);
			bufferCan[1] = 0;
			bufferCan[2] = 0;
			bufferCan[3] = 0;
			bufferCan[4] = 0;
			bufferCan[5] = 0;
			bufferCan[6] = 0;
			bufferCan[7] = 0;
			break;
		case idCanAcceleratie:
			bufferCan[0] = (CanMessaging_ReadValue(Can_PEDALS_Accel_Sensor1_ShortToGnd) << 7) | (CanMessaging_ReadValue(Can_PEDALS_Accel_Sensor1_ShortToVcc) << 6) | (CanMessaging_ReadValue(Can_PEDALS_Accel_Sensor1_OutOfRangeOutput) << 5) | (CanMessaging_ReadValue(Can_PEDALS_Accel_Sensor2_ShortToGnd) << 4) | (CanMessaging_ReadValue(Can_PEDALS_Accel_Sensor2_ShortToVcc) << 3) | (CanMessaging_ReadValue(Can_PEDALS_Accel_Sensor2_OutOfRangeOutput) << 2) | (CanMessaging_ReadValue(Can_PEDALS_Accel_Implausibility) << 1);
			bufferCan[1] = (CanMessaging_ReadValue(Can_PEDALS_PressureSensorVoltage) & (0x01C0)) >> 6;
			bufferCan[2] = ((CanMessaging_ReadValue(Can_PEDALS_AcceleratorSensor2TravelPercentage) & (0x0060)) >> 5) | ((CanMessaging_ReadValue(Can_PEDALS_PressureSensorVoltage) & (0x003F)) << 2);
			bufferCan[3] = ((CanMessaging_ReadValue(Can_PEDALS_AcceleratorSensor2TravelPercentage) & (0x001F)) << 3) | ((CanMessaging_ReadValue(Can_PEDALS_AcceleratorSensor1TravelPercentage) & (0x007)) >>4);
			bufferCan[4] = ((CanMessaging_ReadValue(Can_PEDALS_AcceleratorSensor2Voltage) & (0x3C00)) >> 10) | ((CanMessaging_ReadValue(Can_PEDALS_AcceleratorSensor1TravelPercentage) & (0x000F)) << 4);
			bufferCan[5] = (CanMessaging_ReadValue(Can_PEDALS_AcceleratorSensor2Voltage) & (0x03FC)) >> 2;
			bufferCan[6] = (CanMessaging_ReadValue(Can_PEDALS_AcceleratorSensor2Voltage) & (0x0003) << 6) | ((CanMessaging_ReadValue(Can_PEDALS_AcceleratorSensor1Voltage) & (0x3F00)) >> 8);
			bufferCan[7] = CanMessaging_ReadValue(Can_PEDALS_AcceleratorSensor1Voltage) & (0x00FF);
			break;
		case idCanFrana:
			bufferCan[0] = (CanMessaging_ReadValue(Can_PEDALS_Brake_Sensor1_ShortToGnd) << 7) | (CanMessaging_ReadValue(Can_PEDALS_Brake_Sensor1_ShortToVcc) << 6) | (CanMessaging_ReadValue(Can_PEDALS_Brake_Sensor1_OutOfRangeOutput) << 5) | (CanMessaging_ReadValue(Can_PEDALS_Brake_Sensor2_ShortToGnd) << 4) | (CanMessaging_ReadValue(Can_PEDALS_Brake_Sensor2_ShortToVcc) << 3) | (CanMessaging_ReadValue(Can_PEDALS_Brake_Sensor2_OutOfRangeOutput) << 2) | (CanMessaging_ReadValue(Can_PEDALS_Brake_Implausibility) << 1);
			bufferCan[1] = (CanMessaging_ReadValue(Can_PEDALS_PressureSensorBars) & (0x00C0)) >> 6;
			bufferCan[2] = ((CanMessaging_ReadValue(Can_PEDALS_BrakeSensor2TravelPercentage) & (0x0060)) >> 5) | ((CanMessaging_ReadValue(Can_PEDALS_PressureSensorBars) & (0x003F)) << 2);
			bufferCan[3] = ((CanMessaging_ReadValue(Can_PEDALS_BrakeSensor2TravelPercentage) & (0x001F)) << 3) | ((CanMessaging_ReadValue(Can_PEDALS_BrakeSensor1TravelPercentage) & (0x007)) >>4);
			bufferCan[4] = ((CanMessaging_ReadValue(Can_PEDALS_BrakeSensor2Voltage) & (0x3C00)) >> 10) | ((CanMessaging_ReadValue(Can_PEDALS_BrakeSensor1TravelPercentage) & (0x000F)) << 4);
			bufferCan[5] = (CanMessaging_ReadValue(Can_PEDALS_BrakeSensor2Voltage) & (0x03FC)) >> 2;
			bufferCan[6] = (CanMessaging_ReadValue(Can_PEDALS_BrakeSensor2Voltage) & (0x0003) << 6) | ((CanMessaging_ReadValue(Can_PEDALS_BrakeSensor1Voltage) & (0x3F00)) >> 8);
			bufferCan[7] = CanMessaging_ReadValue(Can_PEDALS_BrakeSensor1Voltage) & (0x00FF);
			break;
		case idCanBaterie:
			bufferCan[0] = 0;
			bufferCan[1] = 0;
			bufferCan[2] = (CanMessaging_ReadValue(Can_TSAC_HighestCellVoltage) & (0x03C0)) >> 6;
			bufferCan[3] = ((CanMessaging_ReadValue(Can_TSAC_HighestCellTemperature) & (0x0300)) >> 8) | ((CanMessaging_ReadValue(Can_TSAC_HighestCellVoltage) & (0x003F)) << 2);
			bufferCan[4] = CanMessaging_ReadValue(Can_TSAC_HighestCellTemperature) & (0x00FF);
			bufferCan[5] = (CanMessaging_ReadValue(Can_TSAC_OverallVoltage) & (0x07F8)) >> 3;
			bufferCan[6] = ((CanMessaging_ReadValue(Can_TSAC_OverallVoltage) & (0x0007)) << 5) | ((CanMessaging_ReadValue(Can_TSAC_OverallCurrent) & (0x1F00)) >> 8);
			bufferCan[7] = CanMessaging_ReadValue(Can_TSAC_OverallCurrent) & (0x00FF);
			break;
	}
}




#ifdef __cplusplus
}
#endif
