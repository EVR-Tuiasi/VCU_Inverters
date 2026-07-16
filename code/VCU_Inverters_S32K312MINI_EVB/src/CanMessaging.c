
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
#include "CanMessaging.h"
#include "Messaging.h"

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/
#define SEND_MASK 			0x80000000

#define CAN_HTH_INVERTOR_STANGA				12U
#define CAN_HTH_INVERTOR_DREAPTA			13U
#define CAN_HTH_INVERTOARE					14U
#define CAN_HTH_BORD						15U
#define CAN_HTH_ACCELERATIE					16U
#define CAN_HTH_FRANA						17U
#define CAN_HTH_BATERIE						18U
#define CAN_HTH_BATERIE_TENSIUNI_CELULE		19U
#define CAN_HTH_BATERIE_TEMPERATURI_CELULE	20U
#define CAN_HTH_BATERIE_2					21U
#define CAN_HTH_BATERIE_CHARGER				22U
#define CAN_HTH_COMUNICATII					23U

#define CAN_CONTROLLER_ID	0U

#define CAN_CHANNEL_EN 	85U
#define CAN_CHANNEL_STB_N 84U

/*Takes a uint64_t argument and any xMonitoredValue_t type of argument.*/
#define ReadDataFromAddressAndWriteInRawBufferCan(rawBufferU64, xMonitoredValue_t_Address) \
		(rawBufferU64) |= ((((uint64_t)((xMonitoredValue_t_Address)->valueCan) & (~(0xFFFFFFFFFFFFFFFFULL << (xMonitoredValue_t_Address)->nrOfBits))) << (xMonitoredValue_t_Address)->shift))

/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/
static uint8_t bufferCan_INVERTOR_STANGA[8];
static uint8_t bufferCan_INVERTOR_DREAPTA[8];
static uint8_t bufferCan_INVERTOARE[8];
static uint8_t bufferCan_BORD[8];
static uint8_t bufferCan_ACCELERATIE[8];
static uint8_t bufferCan_FRANA[8];
static uint8_t bufferCan_BATERIE[8];
static uint8_t bufferCan_BATERIE_TENSIUNI_CELULE[8];
static uint8_t bufferCan_BATERIE_TEMPERATURI_CELULE[8];
static uint8_t bufferCan_BATERIE_2[8];
static uint8_t bufferCan_BATERIE_CHARGER[8];
static uint8_t bufferCan_COMUNICATII[8];

static Can_PduType pduInfo_INVERTOR_STANGA = {
		ID_CAN_INVERTOR_STANGA | SEND_MASK,
		0,
		8,
		bufferCan_INVERTOR_STANGA
};
static Can_PduType pduInfo_INVERTOR_DREAPTA = {
		ID_CAN_INVERTOR_DREAPTA | SEND_MASK,
		1,
		8,
		bufferCan_INVERTOR_DREAPTA
};
static Can_PduType pduInfo_INVERTOARE = {
		ID_CAN_INVERTOARE | SEND_MASK,
		2,
		8,
		bufferCan_INVERTOARE
};
static Can_PduType pduInfo_BORD = {
		ID_CAN_BORD | SEND_MASK,
		3,
		8,
		bufferCan_BORD
};
static Can_PduType pduInfo_ACCELERATIE = {
		ID_CAN_ACCELERATIE | SEND_MASK,
		4,
		8,
		bufferCan_ACCELERATIE
};
static Can_PduType pduInfo_FRANA = {
		ID_CAN_FRANA | SEND_MASK,
		5,
		8,
		bufferCan_FRANA
};
static Can_PduType pduInfo_BATERIE = {
		ID_CAN_BATERIE | SEND_MASK,
		6,
		8,
		bufferCan_BATERIE
};
static Can_PduType pduInfo_BATERIE_TENSIUNI_CELULE = {
		ID_CAN_BATERIE_TENSIUNI_CELULE | SEND_MASK,
		7,
		8,
		bufferCan_BATERIE_TENSIUNI_CELULE
};
static Can_PduType pduInfo_BATERIE_TEMPERATURI_CELULE = {
		ID_CAN_BATERIE_TEMPERATURI_CELULE | SEND_MASK,
		8,
		8,
		bufferCan_BATERIE_TEMPERATURI_CELULE
};
static Can_PduType pduInfo_BATERIE_2 = {
		ID_CAN_BATERIE_2 | SEND_MASK,
		9,
		8,
		bufferCan_BATERIE_2
};
static Can_PduType pduInfo_BATERIE_CHARGER = {
		ID_CAN_BATERIE_CHARGER | SEND_MASK,
		10,
		8,
		bufferCan_BATERIE_CHARGER
};
static Can_PduType pduInfo_COMUNICATII = {
		ID_CAN_COMUNICATII | SEND_MASK,
		11,
		8,
		bufferCan_COMUNICATII
};

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
static void CanMessaging_CreateCellVoltageBuffer(uint16_t index, uint8_t *buffer){
	buffer[0] = index | (CanMessaging_ReadCellVoltageErrors(index*5+0) << 7) | (CanMessaging_ReadCellVoltageErrors(index*5+1) << 6) | (CanMessaging_ReadCellVoltageErrors(index*5+2) << 5) | (CanMessaging_ReadCellVoltageErrors(index*5+3) << 4) | (CanMessaging_ReadCellVoltageErrors(index*5+4) << 3);
	buffer[1] = (CanMessaging_ReadCellVoltage(index*5+0) >> 8);
	buffer[2] = CanMessaging_ReadCellVoltage(index*5+0) & (0x00FF);
	buffer[3] = CanMessaging_ReadCellVoltage(index*5+1) >> 2;
	buffer[4] = ((CanMessaging_ReadCellVoltage(index*5+1) & (0x0003)) << 6) | (CanMessaging_ReadCellVoltage(index*5+2) >> 4);
	buffer[5] = ((CanMessaging_ReadCellVoltage(index*5+2) & (0x000F)) << 4) | (CanMessaging_ReadCellVoltage(index*5+3) >> 6);
	buffer[6] = ((CanMessaging_ReadCellVoltage(index*5+3) & (0x003F)) << 2) | (CanMessaging_ReadCellVoltage(index*5+4) >> 8);
	buffer[7] = CanMessaging_ReadCellVoltage(index*5+4) & (0x00FF); //PENTRU URMATORUL NEFERICIT, DACA APAR PROBLEME INSEAMNA CA AI MODIFICAT FUNCTIA DE READ VALUE SI AI SCORS SIGURANTA (god have mercy on your soul)
}

static void CanMessaging_CreateCellTemperatureBuffer(uint16_t index, uint8_t *buffer){
	buffer[0] = (index >> 2) | (CanMessaging_ReadCellTemperatureErrors(index*5+0) << 7) | (CanMessaging_ReadCellTemperatureErrors(index*5+1) << 6) | (CanMessaging_ReadCellTemperatureErrors(index*5+2) << 5) | (CanMessaging_ReadCellTemperatureErrors(index*5+3) << 4) | (CanMessaging_ReadCellTemperatureErrors(index*5+4) << 3);
	buffer[1] = ((index & (0x0003)) << 6) | (CanMessaging_ReadCellTemperature(index*5+0) >> 8);
	buffer[2] = CanMessaging_ReadCellTemperature(index*5+0) & (0x00FF);
	buffer[3] = CanMessaging_ReadCellTemperature(index*5+1) >> 2;
	buffer[4] = ((CanMessaging_ReadCellTemperature(index*5+1) & (0x0003)) << 6) | (CanMessaging_ReadCellTemperature(index*5+2) >> 4);
	buffer[5] = ((CanMessaging_ReadCellTemperature(index*5+2) & (0x000F)) << 4) | (CanMessaging_ReadCellTemperature(index*5+3) >> 6);
	buffer[6] = ((CanMessaging_ReadCellTemperature(index*5+3) & (0x003F)) << 2) | (CanMessaging_ReadCellTemperature(index*5+4) >> 8);
	buffer[7] = CanMessaging_ReadCellTemperature(index*5+4) & (0x00FF); //PENTRU URMATORUL NEFERICIT, DACA APAR PROBLEME INSEAMNA CA AI MODIFICAT FUNCTIA DE READ VALUE SI AI SCORS SIGURANTA (god have mercy on your soul)
}


static void CanMessaging_CreateBuffer(MessageId_t type, uint8_t *buffer){
	uint64_t buffer_merged = 0;
	switch(type){
		case ID_CAN_INVERTOR_STANGA:
		case ID_UART_INVERTOR_STANGA:
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftMotorTemperature);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterTemperature);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftMotorSpeedKmh);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottleFeedback);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftMotorRpm);
			break;
		case ID_CAN_INVERTOR_DREAPTA:
		case ID_UART_INVERTOR_DREAPTA:
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightMotorTemperature);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterTemperature);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterThrottle);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightMotorSpeedKmh);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterThrottleFeedback);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterInputVoltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightMotorRpm);
			break;
		case ID_CAN_INVERTOARE:
		case ID_UART_INVERTOARE:
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.IsCarRunning);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.IsCarInReverse);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterCurrent);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterCurrent);
			break;
		case ID_CAN_BORD:
		case ID_UART_BORD:
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.DashboardMonitoredValues.ActivationButtonPressed);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.DashboardMonitoredValues.CarReverseCommandPressed);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.DashboardMonitoredValues.IsDisplayWorking);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.DashboardMonitoredValues.IsSegmentsDriverWorking);
			break;
		case ID_CAN_ACCELERATIE:
		case ID_UART_ACCELERATIE:
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1Voltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2Voltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.PressureSensorVoltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Implausibility);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_OutOfRangeOutput);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToVcc);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToGnd);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_OutOfRangeOutput);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToVcc);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToGnd);
			break;
		case ID_CAN_FRANA:
		case ID_UART_FRANA:
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor1Voltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor2Voltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor2TravelPercentage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.PressureSensorBars);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Implausibility);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_OutOfRangeOutput);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToVcc);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToGnd);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_OutOfRangeOutput);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToVcc);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToGnd);
			break;
		case ID_CAN_BATERIE:
		case ID_UART_BATERIE:
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.OverallCurrent);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.OverallVoltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.HighestCellTemperature);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.HighestCellVoltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.LowestCellVoltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.LowestCellTemperature);
			break;
		case ID_CAN_BATERIE_2:
		case ID_UART_BATERIE_2:
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.MedianCellTemperature);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.MedianCellVoltage);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.ShuntError);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.TransceiverError);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.Bms0Error);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.Bms1Error);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.ThermistorsError);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.AmsError);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.DesiredChargingCurrent);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.DesiredChargingVoltage);
			break;
		case ID_CAN_BATERIE_CHARGER:
		case ID_UART_BATERIE_CHARGER:
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.ReportedChargingCurrent);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.ReportedChargingVolts);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.TsacMonitoredValues.ChargerCommand);
			break;
		case ID_CAN_COMUNICATII:
		case ID_UART_COMUNICATII:
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.CommunicationsMonitoredValues.IsDashboardVCUSimulated);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.CommunicationsMonitoredValues.IsInvertersVCUSimulated);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.CommunicationsMonitoredValues.IsPedalsVCUSimulated);
			ReadDataFromAddressAndWriteInRawBufferCan(buffer_merged, &MonitoredValues.CommunicationsMonitoredValues.IsTsacVCUSimulated);
			break;
		case ID_CAN_BATERIE_TENSIUNI_CELULE:
		case ID_UART_BATERIE_TENSIUNI_CELULE:
		case ID_CAN_BATERIE_TEMPERATURI_CELULE:
		case ID_UART_BATERIE_TEMPERATURI_CELULE:
		default:
			break;
	}
	buffer[0] = (uint8_t)(buffer_merged >> 56U);
	buffer[1] = (uint8_t)(buffer_merged >> 48U);
	buffer[2] = (uint8_t)(buffer_merged >> 40U);
	buffer[3] = (uint8_t)(buffer_merged >> 32U);
	buffer[4] = (uint8_t)(buffer_merged >> 24U);
	buffer[5] = (uint8_t)(buffer_merged >> 16U);
	buffer[6] = (uint8_t)(buffer_merged >> 8U);
	buffer[7] = (uint8_t)buffer_merged;
}

void Can_Receive_Interrupt_INVERTOR_STANGA(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint64_t data_merged;
	data_merged = (((uint64_t)PduInfoPtr->SduDataPtr[0]) << 56) + (((uint64_t)PduInfoPtr->SduDataPtr[1]) << 48) + (((uint64_t)PduInfoPtr->SduDataPtr[2]) << 40) + (((uint64_t)PduInfoPtr->SduDataPtr[3]) << 32) + (((uint64_t)PduInfoPtr->SduDataPtr[4]) << 24) + (((uint64_t)PduInfoPtr->SduDataPtr[5]) << 16) + (((uint64_t)PduInfoPtr->SduDataPtr[6]) << 8) + (uint64_t)PduInfoPtr->SduDataPtr[7];
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.LeftMotorTemperature);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterTemperature);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.LeftMotorSpeedKmh);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottleFeedback);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.LeftMotorRpm);
	(void)RxPduId;
}

void Can_Receive_Interrupt_INVERTOR_DREAPTA(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint64_t data_merged;
	data_merged = (((uint64_t)PduInfoPtr->SduDataPtr[0]) << 56) + (((uint64_t)PduInfoPtr->SduDataPtr[1]) << 48) + (((uint64_t)PduInfoPtr->SduDataPtr[2]) << 40) + (((uint64_t)PduInfoPtr->SduDataPtr[3]) << 32) + (((uint64_t)PduInfoPtr->SduDataPtr[4]) << 24) + (((uint64_t)PduInfoPtr->SduDataPtr[5]) << 16) + (((uint64_t)PduInfoPtr->SduDataPtr[6]) << 8) + (uint64_t)PduInfoPtr->SduDataPtr[7];
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.RightMotorTemperature);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterTemperature);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterThrottle);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.RightMotorSpeedKmh);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterThrottleFeedback);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterInputVoltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.RightMotorRpm);
	(void)RxPduId;
}

void Can_Receive_Interrupt_INVERTOARE(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint64_t data_merged;
	data_merged = (((uint64_t)PduInfoPtr->SduDataPtr[0]) << 56) + (((uint64_t)PduInfoPtr->SduDataPtr[1]) << 48) + (((uint64_t)PduInfoPtr->SduDataPtr[2]) << 40) + (((uint64_t)PduInfoPtr->SduDataPtr[3]) << 32) + (((uint64_t)PduInfoPtr->SduDataPtr[4]) << 24) + (((uint64_t)PduInfoPtr->SduDataPtr[5]) << 16) + (((uint64_t)PduInfoPtr->SduDataPtr[6]) << 8) + (uint64_t)PduInfoPtr->SduDataPtr[7];
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.IsCarRunning);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.IsCarInReverse);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.LeftInverterCurrent);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.InvertersMonitoredValues.RightInverterCurrent);
	(void)RxPduId;
}

void Can_Receive_Interrupt_BORD(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint64_t data_merged;
	data_merged = (((uint64_t)PduInfoPtr->SduDataPtr[0]) << 56) + (((uint64_t)PduInfoPtr->SduDataPtr[1]) << 48) + (((uint64_t)PduInfoPtr->SduDataPtr[2]) << 40) + (((uint64_t)PduInfoPtr->SduDataPtr[3]) << 32) + (((uint64_t)PduInfoPtr->SduDataPtr[4]) << 24) + (((uint64_t)PduInfoPtr->SduDataPtr[5]) << 16) + (((uint64_t)PduInfoPtr->SduDataPtr[6]) << 8) + (uint64_t)PduInfoPtr->SduDataPtr[7];
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.DashboardMonitoredValues.ActivationButtonPressed);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.DashboardMonitoredValues.CarReverseCommandPressed);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.DashboardMonitoredValues.IsDisplayWorking);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.DashboardMonitoredValues.IsSegmentsDriverWorking);
	(void)RxPduId;
}

void Can_Receive_Interrupt_ACCELERATIE(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint64_t data_merged;
	data_merged = (((uint64_t)PduInfoPtr->SduDataPtr[0]) << 56) + (((uint64_t)PduInfoPtr->SduDataPtr[1]) << 48) + (((uint64_t)PduInfoPtr->SduDataPtr[2]) << 40) + (((uint64_t)PduInfoPtr->SduDataPtr[3]) << 32) + (((uint64_t)PduInfoPtr->SduDataPtr[4]) << 24) + (((uint64_t)PduInfoPtr->SduDataPtr[5]) << 16) + (((uint64_t)PduInfoPtr->SduDataPtr[6]) << 8) + (uint64_t)PduInfoPtr->SduDataPtr[7];
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1Voltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2Voltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.PressureSensorVoltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Implausibility);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_OutOfRangeOutput);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToVcc);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToGnd);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_OutOfRangeOutput);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToVcc);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToGnd);

	(void)RxPduId;
}

void Can_Receive_Interrupt_FRANA(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint64_t data_merged;
	data_merged = (((uint64_t)PduInfoPtr->SduDataPtr[0]) << 56) + (((uint64_t)PduInfoPtr->SduDataPtr[1]) << 48) + (((uint64_t)PduInfoPtr->SduDataPtr[2]) << 40) + (((uint64_t)PduInfoPtr->SduDataPtr[3]) << 32) + (((uint64_t)PduInfoPtr->SduDataPtr[4]) << 24) + (((uint64_t)PduInfoPtr->SduDataPtr[5]) << 16) + (((uint64_t)PduInfoPtr->SduDataPtr[6]) << 8) + (uint64_t)PduInfoPtr->SduDataPtr[7];
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor1Voltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor2Voltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.BrakeSensor2TravelPercentage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.PressureSensorBars);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Implausibility);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_OutOfRangeOutput);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToVcc);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToGnd);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_OutOfRangeOutput);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToVcc);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToGnd);
	(void)RxPduId;
}

void Can_Receive_Interrupt_BATERIE(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint64_t data_merged;
	data_merged = (((uint64_t)PduInfoPtr->SduDataPtr[0]) << 56) + (((uint64_t)PduInfoPtr->SduDataPtr[1]) << 48) + (((uint64_t)PduInfoPtr->SduDataPtr[2]) << 40) + (((uint64_t)PduInfoPtr->SduDataPtr[3]) << 32) + (((uint64_t)PduInfoPtr->SduDataPtr[4]) << 24) + (((uint64_t)PduInfoPtr->SduDataPtr[5]) << 16) + (((uint64_t)PduInfoPtr->SduDataPtr[6]) << 8) + (uint64_t)PduInfoPtr->SduDataPtr[7];
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.OverallCurrent);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.OverallVoltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.HighestCellTemperature);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.HighestCellVoltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.LowestCellVoltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.LowestCellTemperature);
	(void)RxPduId;
}

void Can_Receive_Interrupt_BATERIE_TENSIUNI_CELULE(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint8_t index = ((uint8_t)(PduInfoPtr->SduDataPtr[0])) & (0x07);
	index = index * 5;
	CanMessaging_SetCellVoltageErrors(((PduInfoPtr->SduDataPtr[0] & (1<<7)) >> 7), index + 0);
	CanMessaging_SetCellVoltageErrors(((PduInfoPtr->SduDataPtr[0] & (1<<6)) >> 6), index + 1);
	CanMessaging_SetCellVoltageErrors(((PduInfoPtr->SduDataPtr[0] & (1<<5)) >> 5), index + 2);
	CanMessaging_SetCellVoltageErrors(((PduInfoPtr->SduDataPtr[0] & (1<<4)) >> 4), index + 3);
	CanMessaging_SetCellVoltageErrors(((PduInfoPtr->SduDataPtr[0] & (1<<3)) >> 3), index + 4);
	CanMessaging_SetCellVoltage((((((uint16_t)PduInfoPtr->SduDataPtr[1]) << 8) | PduInfoPtr->SduDataPtr[2]) & (0x03FF)), index + 0);
	CanMessaging_SetCellVoltage(((((((uint16_t)PduInfoPtr->SduDataPtr[3]) << 8) | PduInfoPtr->SduDataPtr[4]) >> 6) & (0x03FF)), index + 1);
	CanMessaging_SetCellVoltage(((((((uint16_t)PduInfoPtr->SduDataPtr[4]) << 8) | PduInfoPtr->SduDataPtr[5]) >> 4) & (0x03FF)), index + 2);
	CanMessaging_SetCellVoltage(((((((uint16_t)PduInfoPtr->SduDataPtr[5]) << 8) | PduInfoPtr->SduDataPtr[6]) >> 2) & (0x03FF)), index + 3);
	CanMessaging_SetCellVoltage((((((uint16_t)PduInfoPtr->SduDataPtr[6]) << 8) | PduInfoPtr->SduDataPtr[7]) & (0x03FF)), index + 4);
	(void)RxPduId;
}

void Can_Receive_Interrupt_BATERIE_TEMPERATURI_CELULE(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint16_t index = (uint16_t)((PduInfoPtr->SduDataPtr[0] << 2) | (PduInfoPtr->SduDataPtr[1] >> 6)) & (0x001F);
	index = index * 5;
	CanMessaging_SetCellTemperatureErrors(((PduInfoPtr->SduDataPtr[0] & (1<<7)) >> 7), index + 0);
	CanMessaging_SetCellTemperatureErrors(((PduInfoPtr->SduDataPtr[0] & (1<<6)) >> 6), index + 1);
	CanMessaging_SetCellTemperatureErrors(((PduInfoPtr->SduDataPtr[0] & (1<<5)) >> 5), index + 2);
	CanMessaging_SetCellTemperatureErrors(((PduInfoPtr->SduDataPtr[0] & (1<<4)) >> 4), index + 3);
	CanMessaging_SetCellTemperatureErrors(((PduInfoPtr->SduDataPtr[0] & (1<<3)) >> 3), index + 4);
	CanMessaging_SetCellTemperature((((((uint16_t)PduInfoPtr->SduDataPtr[1]) << 8) | PduInfoPtr->SduDataPtr[2]) & (0x03FF)), index + 0);
	CanMessaging_SetCellTemperature(((((((uint16_t)PduInfoPtr->SduDataPtr[3]) << 8) | PduInfoPtr->SduDataPtr[4]) >> 6) & (0x03FF)), index + 1);
	CanMessaging_SetCellTemperature(((((((uint16_t)PduInfoPtr->SduDataPtr[4]) << 8) | PduInfoPtr->SduDataPtr[5]) >> 4) & (0x03FF)), index + 2);
	CanMessaging_SetCellTemperature(((((((uint16_t)PduInfoPtr->SduDataPtr[5]) << 8) | PduInfoPtr->SduDataPtr[6]) >> 2) & (0x03FF)), index + 3);
	CanMessaging_SetCellTemperature((((((uint16_t)PduInfoPtr->SduDataPtr[6]) << 8) | PduInfoPtr->SduDataPtr[7]) & (0x03FF)), index + 4);
	(void)RxPduId;
}

void Can_Receive_Interrupt_BATERIE_2(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint64_t data_merged;
	data_merged = (((uint64_t)PduInfoPtr->SduDataPtr[0]) << 56) + (((uint64_t)PduInfoPtr->SduDataPtr[1]) << 48) + (((uint64_t)PduInfoPtr->SduDataPtr[2]) << 40) + (((uint64_t)PduInfoPtr->SduDataPtr[3]) << 32) + (((uint64_t)PduInfoPtr->SduDataPtr[4]) << 24) + (((uint64_t)PduInfoPtr->SduDataPtr[5]) << 16) + (((uint64_t)PduInfoPtr->SduDataPtr[6]) << 8) + (uint64_t)PduInfoPtr->SduDataPtr[7];
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.MedianCellTemperature);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.MedianCellVoltage);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.ShuntError);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.TransceiverError);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.Bms0Error);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.Bms1Error);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.ThermistorsError);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.AmsError);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.DesiredChargingCurrent);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.DesiredChargingVoltage);
	(void)RxPduId;
}

void Can_Receive_Interrupt_BATERIE_CHARGER(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint64_t data_merged;
	data_merged = (((uint64_t)PduInfoPtr->SduDataPtr[0]) << 56) + (((uint64_t)PduInfoPtr->SduDataPtr[1]) << 48) + (((uint64_t)PduInfoPtr->SduDataPtr[2]) << 40) + (((uint64_t)PduInfoPtr->SduDataPtr[3]) << 32) + (((uint64_t)PduInfoPtr->SduDataPtr[4]) << 24) + (((uint64_t)PduInfoPtr->SduDataPtr[5]) << 16) + (((uint64_t)PduInfoPtr->SduDataPtr[6]) << 8) + (uint64_t)PduInfoPtr->SduDataPtr[7];
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.ReportedChargingCurrent);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.ReportedChargingVolts);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.TsacMonitoredValues.ChargerCommand);
	(void)RxPduId;
}

void Can_Receive_Interrupt_COMUNICATII(PduIdType RxPduId, const PduInfoType * PduInfoPtr){
	uint64_t data_merged;
	data_merged = (((uint64_t)PduInfoPtr->SduDataPtr[0]) << 56) + (((uint64_t)PduInfoPtr->SduDataPtr[1]) << 48) + (((uint64_t)PduInfoPtr->SduDataPtr[2]) << 40) + (((uint64_t)PduInfoPtr->SduDataPtr[3]) << 32) + (((uint64_t)PduInfoPtr->SduDataPtr[4]) << 24) + (((uint64_t)PduInfoPtr->SduDataPtr[5]) << 16) + (((uint64_t)PduInfoPtr->SduDataPtr[6]) << 8) + (uint64_t)PduInfoPtr->SduDataPtr[7];
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.CommunicationsMonitoredValues.IsInvertersVCUSimulated);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.CommunicationsMonitoredValues.IsTsacVCUSimulated);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.CommunicationsMonitoredValues.IsDashboardVCUSimulated);
	WriteCanDataFromRawBufferAtAddress(data_merged, &MonitoredValues.CommunicationsMonitoredValues.IsPedalsVCUSimulated);
	(void)RxPduId;
}

/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/
void CanMessaging_Init(void){
	Dio_WriteChannel(CAN_CHANNEL_EN, STD_HIGH); //CAN0_EN
	volatile uint64 i = 1000000;
	while(i--);
	Dio_WriteChannel(CAN_CHANNEL_STB_N, STD_HIGH); //CAN0_STB_N
	i = 1000000;
	while(i--);
	Can_43_FLEXCAN_SetControllerMode(CAN_CONTROLLER_ID, CAN_CS_STARTED);
	Can_43_FLEXCAN_EnableControllerInterrupts(CAN_CONTROLLER_ID);
}

void CanMessaging_Test(void){
	uint64_t cnt = 0;
	volatile uint64_t i;
	while(1){
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.MedianCellTemperature.maxValue+1), &MonitoredValues.TsacMonitoredValues.MedianCellTemperature);
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.HighestCellTemperature.maxValue+1), &MonitoredValues.TsacMonitoredValues.HighestCellTemperature);
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.LowestCellTemperature.maxValue+1), &MonitoredValues.TsacMonitoredValues.LowestCellTemperature);
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.MedianCellVoltage.maxValue+1), &MonitoredValues.TsacMonitoredValues.MedianCellVoltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.HighestCellVoltage.maxValue+1), &MonitoredValues.TsacMonitoredValues.HighestCellVoltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.LowestCellVoltage.maxValue+1), &MonitoredValues.TsacMonitoredValues.LowestCellVoltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.OverallVoltage.maxValue+1), &MonitoredValues.TsacMonitoredValues.OverallVoltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.OverallCurrent.maxValue+1), &MonitoredValues.TsacMonitoredValues.OverallCurrent);

		WriteCanDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.AmsError);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.TransceiverError);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.ShuntError);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.Bms0Error);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.Bms1Error);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.ThermistorsError);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.ChargerStatus);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.TsacMonitoredValues.ChargerCommand);

		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.ReportedChargingCurrent.maxValue+1), &MonitoredValues.TsacMonitoredValues.ReportedChargingCurrent);
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.ReportedChargingVolts.maxValue+1), &MonitoredValues.TsacMonitoredValues.ReportedChargingVolts);
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.DesiredChargingCurrent.maxValue+1), &MonitoredValues.TsacMonitoredValues.DesiredChargingCurrent);
		WriteCanDataAtAddress(cnt%(MonitoredValues.TsacMonitoredValues.DesiredChargingVoltage.maxValue+1), &MonitoredValues.TsacMonitoredValues.DesiredChargingVoltage);

		for(uint16_t index = 0; index < CELLS_NUM; index++){
			CanMessaging_SetCellVoltageErrors(cnt & 1, index);
			CanMessaging_SetCellVoltage(cnt%1024, index);
		}

		for(uint16_t index = 0; index < THERMISTORS_NUM; index++){
			CanMessaging_SetCellTemperatureErrors(cnt & 1, index);
			CanMessaging_SetCellTemperature(cnt%1024, index);
		}

		WriteCanDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1Voltage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1Voltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2Voltage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2Voltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.BrakeSensor1Voltage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.BrakeSensor1Voltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.BrakeSensor2Voltage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.BrakeSensor2Voltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.BrakeSensor2TravelPercentage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.BrakeSensor2TravelPercentage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.PressureSensorVoltage.maxValue+1), &MonitoredValues.PedalsMonitoredValues.PressureSensorVoltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.PedalsMonitoredValues.PressureSensorBars.maxValue+1), &MonitoredValues.PedalsMonitoredValues.PressureSensorBars);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToGnd);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToVcc);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_OutOfRangeOutput);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToGnd);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToVcc);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_OutOfRangeOutput);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Accel_Implausibility);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToGnd);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToVcc);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_OutOfRangeOutput);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToGnd);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToVcc);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_OutOfRangeOutput);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.PedalsMonitoredValues.Brake_Implausibility);

		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftInverterTemperature.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftInverterTemperature);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftMotorTemperature.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftMotorTemperature);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftInverterCurrent.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftInverterCurrent);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftMotorRpm.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftMotorRpm);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftMotorSpeedKmh.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftMotorSpeedKmh);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.LeftInverterThrottleFeedback.maxValue+1), &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottleFeedback);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightInverterTemperature.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightInverterTemperature);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightMotorTemperature.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightMotorTemperature);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightInverterInputVoltage.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightInverterInputVoltage);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightInverterCurrent.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightInverterCurrent);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightMotorRpm.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightMotorRpm);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightMotorSpeedKmh.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightMotorSpeedKmh);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightInverterThrottle.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightInverterThrottle);
		WriteCanDataAtAddress(cnt%(MonitoredValues.InvertersMonitoredValues.RightInverterThrottleFeedback.maxValue+1), &MonitoredValues.InvertersMonitoredValues.RightInverterThrottleFeedback);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.InvertersMonitoredValues.IsCarInReverse);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.InvertersMonitoredValues.IsCarRunning);

		WriteCanDataAtAddress(cnt&1, &MonitoredValues.DashboardMonitoredValues.ActivationButtonPressed);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.DashboardMonitoredValues.CarReverseCommandPressed);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.DashboardMonitoredValues.IsDisplayWorking);
		WriteCanDataAtAddress(cnt&1, &MonitoredValues.DashboardMonitoredValues.IsSegmentsDriverWorking);

		cnt++;
		CanMessaging_Update();
		i=10000000;
		while(i--);
	}
}

void CanMessaging_Update(void){
	CanMessaging_CreateBuffer(ID_CAN_INVERTOR_STANGA, bufferCan_INVERTOR_STANGA);
	Can_43_FLEXCAN_Write(CAN_HTH_INVERTOR_STANGA, &pduInfo_INVERTOR_STANGA);
	//Can_43_FLEXCAN_AbortMb(CAN_HTH_INVERTOR_STANGA);

	CanMessaging_CreateBuffer(ID_CAN_INVERTOR_DREAPTA, bufferCan_INVERTOR_DREAPTA);
	Can_43_FLEXCAN_Write(CAN_HTH_INVERTOR_DREAPTA, &pduInfo_INVERTOR_DREAPTA);

	CanMessaging_CreateBuffer(ID_CAN_INVERTOARE, bufferCan_INVERTOARE);
	Can_43_FLEXCAN_Write(CAN_HTH_INVERTOARE, &pduInfo_INVERTOARE);

	CanMessaging_CreateBuffer(ID_CAN_BORD, bufferCan_BORD);
	Can_43_FLEXCAN_Write(CAN_HTH_BORD, &pduInfo_BORD);

	CanMessaging_CreateBuffer(ID_CAN_ACCELERATIE, bufferCan_ACCELERATIE);
	Can_43_FLEXCAN_Write(CAN_HTH_ACCELERATIE, &pduInfo_ACCELERATIE);

	CanMessaging_CreateBuffer(ID_CAN_FRANA, bufferCan_FRANA);
	Can_43_FLEXCAN_Write(CAN_HTH_FRANA, &pduInfo_FRANA);

	CanMessaging_CreateBuffer(ID_CAN_BATERIE, bufferCan_BATERIE);
	Can_43_FLEXCAN_Write(CAN_HTH_BATERIE, &pduInfo_BATERIE);

	for(uint16_t index = 0; index < CELLS_LINES; index++){
		CanMessaging_CreateCellVoltageBuffer(index, bufferCan_BATERIE_TENSIUNI_CELULE);
		Can_43_FLEXCAN_Write(CAN_HTH_BATERIE_TENSIUNI_CELULE, &pduInfo_BATERIE_TENSIUNI_CELULE);
	}

	for(uint16_t index = 0; index < THERMISTORS_LINES; index++){
		CanMessaging_CreateCellTemperatureBuffer(index, bufferCan_BATERIE_TEMPERATURI_CELULE);
		Can_43_FLEXCAN_Write(CAN_HTH_BATERIE_TEMPERATURI_CELULE, &pduInfo_BATERIE_TEMPERATURI_CELULE);
	}

	CanMessaging_CreateBuffer(ID_CAN_BATERIE_2, bufferCan_BATERIE_2);
	Can_43_FLEXCAN_Write(CAN_HTH_BATERIE_2, &pduInfo_BATERIE_2);

	CanMessaging_CreateBuffer(ID_CAN_BATERIE_CHARGER, bufferCan_BATERIE_CHARGER);
	Can_43_FLEXCAN_Write(CAN_HTH_BATERIE_CHARGER, &pduInfo_BATERIE_CHARGER);

	CanMessaging_CreateBuffer(ID_CAN_COMUNICATII, bufferCan_COMUNICATII);
	Can_43_FLEXCAN_Write(CAN_HTH_COMUNICATII, &pduInfo_COMUNICATII);
}

uint16_t CanMessaging_ReadCellVoltage(uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < CELLS_NUM)
		return MonitoredValues.TsacMonitoredValues.CellVoltage[index].valueCan;
	else
		return 0;
}
boolean CanMessaging_ReadCellVoltageErrors(uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < CELLS_NUM)
		return MonitoredValues.TsacMonitoredValues.CellVoltage[index].errorCan;
	else
		return 0;
}
uint16_t CanMessaging_ReadCellTemperature(uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < THERMISTORS_NUM)
		return MonitoredValues.TsacMonitoredValues.ThermistorTemperature[index].valueCan;
	else
		return 0;
}
boolean CanMessaging_ReadCellTemperatureErrors(uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < THERMISTORS_NUM)
		return MonitoredValues.TsacMonitoredValues.ThermistorTemperature[index].errorCan;
	else
		return 0;
}

void CanMessaging_SetCellVoltage(uint16_t Value, uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < CELLS_NUM){
		if(Value <= 1023)
			MonitoredValues.TsacMonitoredValues.CellVoltage[index].valueCan = Value;
		else
			MonitoredValues.TsacMonitoredValues.CellVoltage[index].valueCan = 1023;
	}
}
void CanMessaging_SetCellVoltageErrors(boolean Value, uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < CELLS_NUM)
		MonitoredValues.TsacMonitoredValues.CellVoltage[index].errorCan = Value;
}
void CanMessaging_SetCellTemperature(uint16_t Value, uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < THERMISTORS_NUM){
		if(Value <= 1023)
			MonitoredValues.TsacMonitoredValues.ThermistorTemperature[index].valueCan = Value;
		else
			MonitoredValues.TsacMonitoredValues.ThermistorTemperature[index].valueCan = 1023;
	}
}
void CanMessaging_SetCellTemperatureErrors(boolean Value, uint16_t index){
	//NU SCOATE IF-URILE: SUNT DE SIGURANTA
	if(index < THERMISTORS_NUM)
		MonitoredValues.TsacMonitoredValues.ThermistorTemperature[index].errorCan = Value;
}


void CanMessaging_AppTest(void){
	while(1){
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.MedianCellTemperature), &MonitoredValues.TsacMonitoredValues.MedianCellTemperature);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.HighestCellTemperature), &MonitoredValues.TsacMonitoredValues.HighestCellTemperature);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.LowestCellTemperature), &MonitoredValues.TsacMonitoredValues.LowestCellTemperature);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.MedianCellVoltage), &MonitoredValues.TsacMonitoredValues.MedianCellVoltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.HighestCellVoltage), &MonitoredValues.TsacMonitoredValues.HighestCellVoltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.LowestCellVoltage), &MonitoredValues.TsacMonitoredValues.LowestCellVoltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.OverallVoltage), &MonitoredValues.TsacMonitoredValues.OverallVoltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.OverallCurrent), &MonitoredValues.TsacMonitoredValues.OverallCurrent);

		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.AmsError), &MonitoredValues.TsacMonitoredValues.AmsError);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.TransceiverError), &MonitoredValues.TsacMonitoredValues.TransceiverError);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.ShuntError), &MonitoredValues.TsacMonitoredValues.ShuntError);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.Bms0Error), &MonitoredValues.TsacMonitoredValues.Bms0Error);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.Bms1Error), &MonitoredValues.TsacMonitoredValues.Bms1Error);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.ThermistorsError), &MonitoredValues.TsacMonitoredValues.ThermistorsError);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.ChargerStatus), &MonitoredValues.TsacMonitoredValues.ChargerStatus);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.ReportedChargingCurrent), &MonitoredValues.TsacMonitoredValues.ReportedChargingCurrent);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.ReportedChargingVolts), &MonitoredValues.TsacMonitoredValues.ReportedChargingVolts);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.ChargerCommand), &MonitoredValues.TsacMonitoredValues.ChargerCommand);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.DesiredChargingCurrent), &MonitoredValues.TsacMonitoredValues.DesiredChargingCurrent);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.TsacMonitoredValues.DesiredChargingVoltage), &MonitoredValues.TsacMonitoredValues.DesiredChargingVoltage);

		for(uint16_t index = 0; index < CELLS_NUM; index++){
			UartMessaging_SetCellVoltageErrors(CanMessaging_ReadCellVoltageErrors(index), index);
			UartMessaging_SetCellVoltage(CanMessaging_ReadCellVoltage(index), index);
		}

		for(uint16_t index = 0; index < THERMISTORS_NUM; index++){
			UartMessaging_SetCellTemperatureErrors(CanMessaging_ReadCellTemperatureErrors(index), index);
			UartMessaging_SetCellTemperature(CanMessaging_ReadCellTemperature(index), index);
		}

		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1Voltage), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1Voltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2Voltage), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2Voltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage), &MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.BrakeSensor1Voltage), &MonitoredValues.PedalsMonitoredValues.BrakeSensor1Voltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.BrakeSensor2Voltage), &MonitoredValues.PedalsMonitoredValues.BrakeSensor2Voltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage), &MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.BrakeSensor2TravelPercentage), &MonitoredValues.PedalsMonitoredValues.BrakeSensor2TravelPercentage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.PressureSensorVoltage), &MonitoredValues.PedalsMonitoredValues.PressureSensorVoltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.PressureSensorBars), &MonitoredValues.PedalsMonitoredValues.PressureSensorBars);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToGnd), &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToGnd);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToVcc), &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToVcc);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_OutOfRangeOutput), &MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_OutOfRangeOutput);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToGnd), &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToGnd);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToVcc), &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToVcc);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_OutOfRangeOutput), &MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_OutOfRangeOutput);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Accel_Implausibility), &MonitoredValues.PedalsMonitoredValues.Accel_Implausibility);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToGnd), &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToGnd);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToVcc), &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToVcc);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_OutOfRangeOutput), &MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_OutOfRangeOutput);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToGnd), &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToGnd);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToVcc), &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToVcc);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_OutOfRangeOutput), &MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_OutOfRangeOutput);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.PedalsMonitoredValues.Brake_Implausibility), &MonitoredValues.PedalsMonitoredValues.Brake_Implausibility);

		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.LeftInverterTemperature), &MonitoredValues.InvertersMonitoredValues.LeftInverterTemperature);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.LeftMotorTemperature), &MonitoredValues.InvertersMonitoredValues.LeftMotorTemperature);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage), &MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.LeftInverterCurrent), &MonitoredValues.InvertersMonitoredValues.LeftInverterCurrent);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.LeftMotorRpm), &MonitoredValues.InvertersMonitoredValues.LeftMotorRpm);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.LeftMotorSpeedKmh), &MonitoredValues.InvertersMonitoredValues.LeftMotorSpeedKmh);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle), &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.LeftInverterThrottleFeedback), &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottleFeedback);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.RightInverterTemperature), &MonitoredValues.InvertersMonitoredValues.RightInverterTemperature);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.RightMotorTemperature), &MonitoredValues.InvertersMonitoredValues.RightMotorTemperature);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.RightInverterInputVoltage), &MonitoredValues.InvertersMonitoredValues.RightInverterInputVoltage);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.RightInverterCurrent), &MonitoredValues.InvertersMonitoredValues.RightInverterCurrent);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.RightMotorRpm), &MonitoredValues.InvertersMonitoredValues.RightMotorRpm);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.RightMotorSpeedKmh), &MonitoredValues.InvertersMonitoredValues.RightMotorSpeedKmh);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.RightInverterThrottle), &MonitoredValues.InvertersMonitoredValues.RightInverterThrottle);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.RightInverterThrottleFeedback), &MonitoredValues.InvertersMonitoredValues.RightInverterThrottleFeedback);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.IsCarInReverse), &MonitoredValues.InvertersMonitoredValues.IsCarInReverse);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.InvertersMonitoredValues.IsCarRunning), &MonitoredValues.InvertersMonitoredValues.IsCarRunning);

		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.DashboardMonitoredValues.ActivationButtonPressed), &MonitoredValues.DashboardMonitoredValues.ActivationButtonPressed);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.DashboardMonitoredValues.CarReverseCommandPressed), &MonitoredValues.DashboardMonitoredValues.CarReverseCommandPressed);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.DashboardMonitoredValues.IsDisplayWorking), &MonitoredValues.DashboardMonitoredValues.IsDisplayWorking);
		WriteUartDataAtAddress(ReadCanDataFromAddress(&MonitoredValues.DashboardMonitoredValues.IsSegmentsDriverWorking), &MonitoredValues.DashboardMonitoredValues.IsSegmentsDriverWorking);

		//Send data
		UartMessaging_Update();
	}
}

#ifdef __cplusplus
}
#endif
