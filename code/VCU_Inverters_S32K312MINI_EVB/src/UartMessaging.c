
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
#include "Dio.h"
#include "Mcl.h"
#include "UartMessaging.h"
#include "Messaging.h"

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/


/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/
/*Takes a uint64_t argument and any xMonitoredValue_t type of argument.*/
#define ReadDataFromAddressAndWriteInRawBufferUart(rawBufferU64, xMonitoredValue_t_Address) \
		(rawBufferU64) |= ((((uint64_t)((xMonitoredValue_t_Address)->valueUart) & (~(0xFFFFFFFFFFFFFFFFULL << (xMonitoredValue_t_Address)->nrOfBits))) << (xMonitoredValue_t_Address)->shift))

/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/
static uint8_t bufferUart[10];

/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
extern MonitoredValues_t MonitoredValues;

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/
static uint8_t CRC_calculate(uint8_t length){
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

static void UartMessaging_CreateBuffer(MessageId_t type, uint8_t *buffer){
	uint64_t buffer_merged = 0;
	switch(type){
		case ID_CAN_INVERTOR_STANGA:
		case ID_UART_INVERTOR_STANGA:
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftMotorTemperature);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterTemperature);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftMotorSpeedKmh);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottleFeedback);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftMotorRpm);
			break;
		case ID_CAN_INVERTOR_DREAPTA:
		case ID_UART_INVERTOR_DREAPTA:
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightMotorTemperature);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterTemperature);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterThrottle);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightMotorSpeedKmh);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterThrottleFeedback);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterInputVoltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightMotorRpm);
			break;
		case ID_CAN_INVERTOARE:
		case ID_UART_INVERTOARE:
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.IsCarRunning);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.IsCarInReverse);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterCurrent);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterCurrent);
			break;
		case ID_CAN_BORD:
		case ID_UART_BORD:
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.DashboardMonitoredValues.ActivationButtonPressed);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.DashboardMonitoredValues.CarReverseCommandPressed);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.DashboardMonitoredValues.IsDisplayWorking);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.DashboardMonitoredValues.IsSegmentsDriverWorking);
			break;
		case ID_CAN_ACCELERATIE:
		case ID_UART_ACCELERATIE:
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1Voltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2Voltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.PressureSensorVoltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Implausibility);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_OutOfRangeOutput);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToVcc);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToGnd);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_OutOfRangeOutput);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToVcc);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToGnd);
			break;
		case ID_CAN_FRANA:
		case ID_UART_FRANA:
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor1Voltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor2Voltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor2TravelPercentage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.PressureSensorBars);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Implausibility);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_OutOfRangeOutput);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToVcc);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToGnd);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_OutOfRangeOutput);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToVcc);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToGnd);
			break;
		case ID_CAN_BATERIE:
		case ID_UART_BATERIE:
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.OverallCurrent);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.OverallVoltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.HighestCellTemperature);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.HighestCellVoltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.LowestCellVoltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.LowestCellTemperature);
			break;
		case ID_CAN_BATERIE_2:
		case ID_UART_BATERIE_2:
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.MedianCellTemperature);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.MedianCellVoltage);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.ShuntError);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.TransceiverError);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.Bms0Error);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.Bms1Error);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.ThermistorsError);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.AmsError);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.DesiredChargingCurrent);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.DesiredChargingVoltage);
			break;
		case ID_CAN_BATERIE_CHARGER:
		case ID_UART_BATERIE_CHARGER:
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.ReportedChargingCurrent);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.ReportedChargingVolts);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.TsacMonitoredValues.ChargerCommand);
			break;
		case ID_CAN_COMUNICATII:
		case ID_UART_COMUNICATII:
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.CommunicationsMonitoredValues.IsDashboardVCUSimulated);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.CommunicationsMonitoredValues.IsInvertersVCUSimulated);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.CommunicationsMonitoredValues.IsPedalsVCUSimulated);
			ReadDataFromAddressAndWriteInRawBufferUart(buffer_merged, &MonitoredValues.CommunicationsMonitoredValues.IsTsacVCUSimulated);
			break;
		case ID_CAN_BATERIE_TENSIUNI_CELULE:
		case ID_UART_BATERIE_TENSIUNI_CELULE:
		case ID_CAN_BATERIE_TEMPERATURI_CELULE:
		case ID_UART_BATERIE_TEMPERATURI_CELULE:
		default:
			break;
	}
	buffer[0] = type;
	buffer[1] = (uint8_t)(buffer_merged >> 56U);
	buffer[2] = (uint8_t)(buffer_merged >> 48U);
	buffer[3] = (uint8_t)(buffer_merged >> 40U);
	buffer[4] = (uint8_t)(buffer_merged >> 32U);
	buffer[5] = (uint8_t)(buffer_merged >> 24U);
	buffer[6] = (uint8_t)(buffer_merged >> 16U);
	buffer[7] = (uint8_t)(buffer_merged >> 8U);
	buffer[8] = (uint8_t)buffer_merged;
	buffer[9] = CRC_calculate(10);
}

void UartMessaging_CreateCellVoltageBuffer(uint16_t index){
	bufferUart[0] = ID_UART_BATERIE_TENSIUNI_CELULE;
	bufferUart[1] = index | (UartMessaging_ReadCellVoltageErrors(index*5+0) << 7) | (UartMessaging_ReadCellVoltageErrors(index*5+1) << 6) | (UartMessaging_ReadCellVoltageErrors(index*5+2) << 5) | (UartMessaging_ReadCellVoltageErrors(index*5+3) << 4) | (UartMessaging_ReadCellVoltageErrors(index*5+4) << 3);
	bufferUart[2] = (UartMessaging_ReadCellVoltage(index*5+0) >> 8);
	bufferUart[3] = UartMessaging_ReadCellVoltage(index*5+0) & (0x00FF);
	bufferUart[4] = UartMessaging_ReadCellVoltage(index*5+1) >> 2;
	bufferUart[5] = ((UartMessaging_ReadCellVoltage(index*5+1) & (0x0003)) << 6) | (UartMessaging_ReadCellVoltage(index*5+2) >> 4);
	bufferUart[6] = ((UartMessaging_ReadCellVoltage(index*5+2) & (0x000F)) << 4) | (UartMessaging_ReadCellVoltage(index*5+3) >> 6);
	bufferUart[7] = ((UartMessaging_ReadCellVoltage(index*5+3) & (0x003F)) << 2) | (UartMessaging_ReadCellVoltage(index*5+4) >> 8);
	bufferUart[8] = UartMessaging_ReadCellVoltage(index*5+4) & (0x00FF);//PENTRU URMATORUL NEFERICIT, DACA APAR PROBLEME INSEAMNA CA AI MODIFICAT FUNCTIA DE READ VALUE SI AI SCORS SIGURANTA (god have mercy on your soul)
	bufferUart[9] = CRC_calculate(10);
}

void UartMessaging_CreateCellTemperatureBuffer(uint16_t index){
	bufferUart[0] = ID_UART_BATERIE_TEMPERATURI_CELULE;
	bufferUart[1] = (index >> 2) | (UartMessaging_ReadCellTemperatureErrors(index*5+0) << 7) | (UartMessaging_ReadCellTemperatureErrors(index*5+1) << 6) | (UartMessaging_ReadCellTemperatureErrors(index*5+2) << 5) | (UartMessaging_ReadCellTemperatureErrors(index*5+3) << 4) | (UartMessaging_ReadCellTemperatureErrors(index*5+4) << 3);
	bufferUart[2] = ((index & (0x0003)) << 6) | (UartMessaging_ReadCellTemperature(index*5+0) >> 8);
	bufferUart[3] = UartMessaging_ReadCellTemperature(index*5+0) & (0x00FF);
	bufferUart[4] = UartMessaging_ReadCellTemperature(index*5+1) >> 2;
	bufferUart[5] = ((UartMessaging_ReadCellTemperature(index*5+1) & (0x0003)) << 6) | (UartMessaging_ReadCellTemperature(index*5+2) >> 4);
	bufferUart[6] = ((UartMessaging_ReadCellTemperature(index*5+2) & (0x000F)) << 4) | (UartMessaging_ReadCellTemperature(index*5+3) >> 6);
	bufferUart[7] = ((UartMessaging_ReadCellTemperature(index*5+3) & (0x003F)) << 2) | (UartMessaging_ReadCellTemperature(index*5+4) >> 8);
	bufferUart[8] = UartMessaging_ReadCellTemperature(index*5+4) & (0x00FF); //PENTRU URMATORUL NEFERICIT, DACA APAR PROBLEME INSEAMNA CA AI MODIFICAT FUNCTIA DE READ VALUE SI AI SCORS SIGURANTA (god have mercy on your soul)
	bufferUart[9] = CRC_calculate(10);
}
/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/

void UartMessaging_Init(void){
	Uart_Init(NULL_PTR);
}

void UartMessaging_Test(void){
	uint64_t cnt = 0;
	volatile int i;
	while(1){
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.MedianCellTemperature.maxValue+1), &MonitoredValues.TsacMonitoredValues.MedianCellTemperature);
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.HighestCellTemperature.maxValue+1), &MonitoredValues.TsacMonitoredValues.HighestCellTemperature);
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.LowestCellTemperature.maxValue+1), &MonitoredValues.TsacMonitoredValues.LowestCellTemperature);
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.MedianCellVoltage.maxValue+1), &MonitoredValues.TsacMonitoredValues.MedianCellVoltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.HighestCellVoltage.maxValue+1), &MonitoredValues.TsacMonitoredValues.HighestCellVoltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.LowestCellVoltage.maxValue+1), &MonitoredValues.TsacMonitoredValues.LowestCellVoltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.OverallVoltage.maxValue+1), &MonitoredValues.TsacMonitoredValues.OverallVoltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.OverallCurrent.maxValue+1), &MonitoredValues.TsacMonitoredValues.OverallCurrent);

		WriteUartDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.AmsError);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.TransceiverError);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.ShuntError);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.Bms0Error);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.Bms1Error);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.ThermistorsError);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.ChargerStatus);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.ChargerCommand);

		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.ReportedChargingCurrent.maxValue+1), &MonitoredValues.TsacMonitoredValues.ReportedChargingCurrent);
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.ReportedChargingVolts.maxValue+1), &MonitoredValues.TsacMonitoredValues.ReportedChargingVolts);
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.DesiredChargingCurrent.maxValue+1), &MonitoredValues.TsacMonitoredValues.DesiredChargingCurrent);
		WriteUartDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.DesiredChargingVoltage.maxValue+1), &MonitoredValues.TsacMonitoredValues.DesiredChargingVoltage);

		for(uint16_t index = 0; index < CELLS_NUM; index++){
			UartMessaging_SetCellVoltageErrors(cnt & 1, index);
			UartMessaging_SetCellVoltage(cnt%1024, index);
		}

		for(uint16_t index = 0; index < THERMISTORS_NUM; index++){
			UartMessaging_SetCellTemperatureErrors(cnt & 1, index);
			UartMessaging_SetCellTemperature(cnt%1024, index);
		}

		WriteUartDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1Voltage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1Voltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2Voltage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2Voltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.BrakeSensor1Voltage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.BrakeSensor1Voltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.BrakeSensor2Voltage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.BrakeSensor2Voltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.BrakeSensor2TravelPercentage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.BrakeSensor2TravelPercentage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.PressureSensorVoltage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.PressureSensorVoltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.PressureSensorBars.maxValue+1), &MonitoredValues.PedalsMonitoredValues.PressureSensorBars);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToGnd);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToVcc);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_OutOfRangeOutput);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToGnd);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToVcc);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_OutOfRangeOutput);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Implausibility);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToGnd);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToVcc);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_OutOfRangeOutput);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToGnd);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToVcc);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_OutOfRangeOutput);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Implausibility);

		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftInverterTemperature.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftInverterTemperature);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftMotorTemperature.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftMotorTemperature);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftInverterCurrent.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftInverterCurrent);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftMotorRpm.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftMotorRpm);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftMotorSpeedKmh.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftMotorSpeedKmh);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftInverterThrottleFeedback.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottleFeedback);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightInverterTemperature.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightInverterTemperature);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightMotorTemperature.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightMotorTemperature);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightInverterInputVoltage.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightInverterInputVoltage);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightInverterCurrent.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightInverterCurrent);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightMotorRpm.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightMotorRpm);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightMotorSpeedKmh.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightMotorSpeedKmh);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightInverterThrottle.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightInverterThrottle);
		WriteUartDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightInverterThrottleFeedback.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightInverterThrottleFeedback);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.InvertersMonitoredValues.IsCarInReverse);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.InvertersMonitoredValues.IsCarRunning);

		WriteUartDataAtAddress(cnt&1, &MonitoredValues.DashboardMonitoredValues.ActivationButtonPressed);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.DashboardMonitoredValues.CarReverseCommandPressed);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.DashboardMonitoredValues.IsDisplayWorking);
		WriteUartDataAtAddress(cnt&1, &MonitoredValues.DashboardMonitoredValues.IsSegmentsDriverWorking);
		cnt++;
		UartMessaging_Update();
		i=100000;
		while(i--);
	}
}

void UartMessaging_Update(void){
	UartMessaging_CreateBuffer(ID_UART_INVERTOR_STANGA, bufferUart);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	UartMessaging_CreateBuffer(ID_UART_INVERTOR_DREAPTA, bufferUart);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	UartMessaging_CreateBuffer(ID_UART_INVERTOARE, bufferUart);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	UartMessaging_CreateBuffer(ID_UART_BORD, bufferUart);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	UartMessaging_CreateBuffer(ID_UART_ACCELERATIE, bufferUart);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	UartMessaging_CreateBuffer(ID_UART_FRANA, bufferUart);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	UartMessaging_CreateBuffer(ID_UART_BATERIE, bufferUart);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	for(uint16_t index = 0; index < CELLS_LINES; index++){
		UartMessaging_CreateCellVoltageBuffer(index);
		Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	}
	for(uint16_t index = 0; index < THERMISTORS_LINES; index++){
		UartMessaging_CreateCellTemperatureBuffer(index);
		Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	}
	UartMessaging_CreateBuffer(ID_UART_BATERIE_2, bufferUart);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
	UartMessaging_CreateBuffer(ID_UART_BATERIE_CHARGER, bufferUart);
	Uart_SyncSend(UART_Channel, bufferUart, 10, 10000000);
}

uint16_t UartMessaging_ReadCellVoltage(uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < CELLS_NUM)
		return MonitoredValues.TsacMonitoredValues.CellVoltage[index].valueUart;
	else
		return 0;
}
boolean UartMessaging_ReadCellVoltageErrors(uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < CELLS_NUM)
		return MonitoredValues.TsacMonitoredValues.CellVoltage[index].errorUart;
	else
		return 0;
}
uint16_t UartMessaging_ReadCellTemperature(uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < THERMISTORS_NUM)
		return MonitoredValues.TsacMonitoredValues.ThermistorTemperature[index].valueUart;
	else
		return 0;
}
boolean UartMessaging_ReadCellTemperatureErrors(uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < THERMISTORS_NUM)
		return MonitoredValues.TsacMonitoredValues.ThermistorTemperature[index].errorUart;
	else
		return 0;
}

void UartMessaging_SetCellVoltage(uint16_t Value, uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < CELLS_NUM){
		if(Value <= 1023)
			MonitoredValues.TsacMonitoredValues.CellVoltage[index].valueUart = Value;
		else
			MonitoredValues.TsacMonitoredValues.CellVoltage[index].valueUart = 1023;
	}
}
void UartMessaging_SetCellVoltageErrors(boolean Value, uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < CELLS_NUM)
		MonitoredValues.TsacMonitoredValues.CellVoltage[index].errorUart = Value;
}
void UartMessaging_SetCellTemperature(uint16_t Value, uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < THERMISTORS_NUM){
		if(Value <= 1023)
			MonitoredValues.TsacMonitoredValues.ThermistorTemperature[index].valueUart = Value;
		else
			MonitoredValues.TsacMonitoredValues.ThermistorTemperature[index].valueUart = 1023;
	}
}
void UartMessaging_SetCellTemperatureErrors(boolean Value, uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < THERMISTORS_NUM)
		MonitoredValues.TsacMonitoredValues.ThermistorTemperature[index].errorUart = Value;
}

#ifdef __cplusplus
}
#endif
