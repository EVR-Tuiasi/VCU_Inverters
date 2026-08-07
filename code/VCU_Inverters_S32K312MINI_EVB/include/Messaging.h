#ifndef MESSAGING_H
#define MESSAGING_H

#ifdef __cplusplus
extern "C"{
#endif


/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
#include "stdbool.h"
#include "stdint.h"

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
#define CELLS_NUM 24U
#define THERMISTORS_NUM 128U
#define CELLS_LINES 5U
#define THERMISTORS_LINES 26U

typedef enum{
	ID_CAN_INVERTOR_STANGA = 	0x00000110,
	ID_UART_INVERTOR_STANGA =   0x10,
	ID_CAN_INVERTOR_DREAPTA = 	0x00000111,
	ID_UART_INVERTOR_DREAPTA =  0x11,
	ID_CAN_INVERTOARE = 		0x00000112,
	ID_UART_INVERTOARE = 		0x12,

	ID_CAN_BORD = 				0x00000113,
	ID_UART_BORD = 				0x13,

	ID_CAN_ACCELERATIE = 		0x00000100,
	ID_UART_ACCELERATIE = 		0x30,
	ID_CAN_FRANA = 				0x00000101,
	ID_UART_FRANA =				0x31,

	ID_CAN_BATERIE = 			0x00000114,
	ID_UART_BATERIE = 			0x14,
	ID_CAN_BATERIE_TENSIUNI_CELULE = 0x00000115,
	ID_UART_BATERIE_TENSIUNI_CELULE = 0x15,
	ID_CAN_BATERIE_TEMPERATURI_CELULE = 0x00000116,
	ID_UART_BATERIE_TEMPERATURI_CELULE = 0x16,
	ID_CAN_BATERIE_2 = 			0x00000117,
	ID_UART_BATERIE_2 = 		0x0000017,
	ID_CAN_BATERIE_CHARGER =    0x00000119,
	ID_UART_BATERIE_CHARGER = 	0x0000019,

	ID_CAN_COMUNICATII = 		0x00000118,
	ID_UART_COMUNICATII = 		0x18
}MessageId_t;

typedef struct{
	uint8_t valueCan;
	uint8_t valueUart;
	const uint8_t nrOfBits;
	const uint8_t shift;
	const uint8_t maxValue;
}U8MonitoredValue_t;

typedef struct{
	uint16_t valueCan;
	uint16_t valueUart;
	const uint8_t nrOfBits;
	const uint8_t shift;
	const uint16_t maxValue;
}U16MonitoredValue_t;

typedef struct{
	uint32_t valueCan;
	uint32_t valueUart;
	const uint8_t nrOfBits;
	const uint8_t shift;
	const uint32_t maxValue;
}U32MonitoredValue_t;

typedef struct{
	uint64_t valueCan;
	uint64_t valueUart;
	const uint8_t nrOfBits;
	const uint8_t shift;
	const uint64_t maxValue;
}U64MonitoredValue_t;

typedef struct{
	bool valueCan;
	bool valueUart;
	const uint8_t nrOfBits;
	const uint8_t shift;
	const bool maxValue;
}BoolMonitoredValue_t;

typedef struct{
	uint16_t valueCan;
	bool errorCan;												   /* 1 bit, 0 means safe, 1 means errors */
	uint16_t valueUart;
	bool errorUart;												   /* 1 bit, 0 means safe, 1 means errors */
}CellsMonitoredValue_t;

typedef struct{
    /* General values */
	U16MonitoredValue_t MedianCellTemperature;                     /* 10 bits, 0-1023, 0 to 102.3 degrees C, 0.1 degrees C per bit */
	U16MonitoredValue_t HighestCellTemperature;                    /* 10 bits, 0-1023, 0 to 102.3 degrees C, 0.1 degrees C per bit */
	U16MonitoredValue_t LowestCellTemperature;                     /* 10 bits, 0-1023, 0 to 102.3 degrees C, 0.1 degrees C per bit */
	U16MonitoredValue_t MedianCellVoltage;                         /* 10 bits, 0-1023, 0 to 10.23 Volts, 0.01 Volts per bit */
	U16MonitoredValue_t HighestCellVoltage;                        /* 10 bits, 0-1023, 0 to 10.23 Volts, 0.01 Volts per bit */
	U16MonitoredValue_t LowestCellVoltage;                         /* 10 bits, 0-1023, 0 to 10.23 Volts, 0.01 Volts per bit */
	U16MonitoredValue_t OverallVoltage;                            /* 11 bits, 0-2047, 0 to 204.7 Volts, 0.1 Volts per bit */
	U16MonitoredValue_t OverallCurrent;                            /* 13 bits, 0-8095, 0 to 809.5 Amps, 0.1 Amps per bit */
    /* Cell voltages and temperatures*/
	CellsMonitoredValue_t CellVoltage[CELLS_NUM];                  /* 10 bits, 0-1023, 0 to 10.23 Volts, 0.01 Volts per bit */

	CellsMonitoredValue_t ThermistorTemperature[THERMISTORS_NUM];  /* 10 bits, 0-1023, 0 to 102.3 degrees C, 0.1 degrees C per bit */
    /* Status and errors */
	BoolMonitoredValue_t AmsError;                                 /* 1 bit, 0 means safe, 1 means errors */
	BoolMonitoredValue_t TransceiverError;                         /* 1 bit, 0 means safe, 1 means errors */
	BoolMonitoredValue_t ShuntError;                               /* 1 bit, 0 means safe, 1 means errors */
	BoolMonitoredValue_t Bms0Error;                                /* 1 bit, 0 means safe, 1 means errors */
	BoolMonitoredValue_t Bms1Error;                                /* 1 bit, 0 means safe, 1 means errors */
	BoolMonitoredValue_t ThermistorsError;						   /* 1 bit, 0 means safe, 1 means errors */
	BoolMonitoredValue_t ChargerStatus;							   /* 1 bit, 0 means safe, 1 means errors */
	U16MonitoredValue_t ReportedChargingCurrent;				   /* 16 bits, 0-65535, 0 to 6553.5A, 0.1 Amps per bit */
	U16MonitoredValue_t ReportedChargingVolts;  				   /* 16 bits, 0-65535, 0 to 6553.5V, 0.1 Volts per bit */
	BoolMonitoredValue_t ChargerCommand;						   /* 1 bit, 0 means charger should be charging, 1 means charger should NOT be charging */
	U16MonitoredValue_t DesiredChargingCurrent;					   /* 9 bits, 0-320, 0 to 32.0A, 0.1 Amps per bit */
	U16MonitoredValue_t DesiredChargingVoltage;					   /* 10 bits, 0-1008, 0 to 100.8V, 0.1 Volts per bit */
}TsacMonitoredValues_t;

typedef struct{
    /* Accelerator */
	U16MonitoredValue_t AcceleratorSensor1Voltage;                 /* 14 bits, 0-16383, 0 to 5.00 Volts */
	U16MonitoredValue_t AcceleratorSensor2Voltage;                 /* 14 bits, 0-16383, 0 to 5.00 Volts */
	U8MonitoredValue_t AcceleratorSensor1TravelPercentage;         /* 7 bits, 0-100, 0 to 100 Percent, 1 Percent per bit */
	U8MonitoredValue_t AcceleratorSensor2TravelPercentage;         /* 7 bits, 0-100, 0 to 100 Percent, 1 Percent per bit */
    /* Brake */
	U16MonitoredValue_t BrakeSensor1Voltage;                       /* 14 bits, 0-16383, 0 to 5.00 Volts */
	U16MonitoredValue_t BrakeSensor2Voltage;                       /* 14 bits, 0-16383, 0 to 5.00 Volts */
	U8MonitoredValue_t BrakeSensor1TravelPercentage;               /* 7 bits, 0-100, 0 to 100 Percent, 1 Percent per bit */
	U8MonitoredValue_t BrakeSensor2TravelPercentage;               /* 7 bits, 0-100, 0 to 100 Percent, 1 Percent per bit */
    /* Pressure */
	U16MonitoredValue_t PressureSensorVoltage;                     /* 9 bits, 0-500, 0 to 5.00 Volts, 0.1 Volts per bit */
    U8MonitoredValue_t PressureSensorBars;                         /* 8 bits, 0-255, 0 to 255 Bars, 1 Bar per bit */
    /* Status and errors */
    BoolMonitoredValue_t Accel_Sensor1_ShortToGnd;                      /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Accel_Sensor1_ShortToVcc;                      /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Accel_Sensor1_OutOfRangeOutput;                /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Accel_Sensor2_ShortToGnd;                      /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Accel_Sensor2_ShortToVcc;                      /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Accel_Sensor2_OutOfRangeOutput;                /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Accel_Implausibility;                          /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Brake_Sensor1_ShortToGnd;                      /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Brake_Sensor1_ShortToVcc;                      /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Brake_Sensor1_OutOfRangeOutput;                /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Brake_Sensor2_ShortToGnd;                      /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Brake_Sensor2_ShortToVcc;                      /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Brake_Sensor2_OutOfRangeOutput;                /* 1 bit, 0 means safe, 1 means errors */
    BoolMonitoredValue_t Brake_Implausibility;                          /* 1 bit, 0 means safe, 1 means errors */
}PedalsMonitoredValues_t;

typedef struct{
    /* Left inverter and motor */
	U8MonitoredValue_t LeftInverterTemperature;                    /* 8 bits, 0-255, -40 to 215 degrees C, 1 degree C per bit */
	U8MonitoredValue_t LeftMotorTemperature;                       /* 8 bits, 0-255, -30 to 225 degrees C, 1 degree C per bit */
	U16MonitoredValue_t LeftInverterInputVoltage;                  /* 11 bits, 0-1800, 0 to 180.0 Volts, 0.1 Volts per bit */
	U16MonitoredValue_t LeftInverterCurrent;                       /* 12 bits, 0-4000, 0 to 400.0 Amps, 0.1 Amps per bit */
	U16MonitoredValue_t LeftMotorRpm;                              /* 13 bits, 0-6000, 0 to 6000 RPM, 1 RPM per bit */
	U8MonitoredValue_t LeftMotorSpeedKmh;                          /* 8 bits, 0-255, 0 to 255 Km/h, 1 Km/h per bit */
	U8MonitoredValue_t LeftInverterThrottle;                       /* 8 bits, 0-250, 0 to 5.00 Volts, 0.02 Volts per bit */
	U8MonitoredValue_t LeftInverterThrottleFeedback;               /* 8 bits, 0-250, 0 to 5.00 Volts, 0.02 Volts per bit */
    /* Right inverter and motor */
	U8MonitoredValue_t RightInverterTemperature;                   /* 8 bits, 0-255, -40 to 215 degrees C, 1 degree C per bit */
	U8MonitoredValue_t RightMotorTemperature;                      /* 8 bits, 0-255, -30 to 225 degrees C, 1 degree C per bit */
	U16MonitoredValue_t RightInverterInputVoltage;                 /* 11 bits, 0-1800, 0 to 180.0 Volts, 0.1 Volts per bit */
	U16MonitoredValue_t RightInverterCurrent;                      /* 12 bits, 0-4000, 0 to 400.0 Amps, 0.1 Amps per bit */
	U16MonitoredValue_t RightMotorRpm;                             /* 13 bits, 0-6000, 0 to 6000 RPM, 1 RPM per bit */
	U8MonitoredValue_t RightMotorSpeedKmh;                         /* 8 bits, 0-255, 0 to 255 Km/h, 1 Km/h per bit */
	U8MonitoredValue_t RightInverterThrottle;                      /* 8 bits, 0-250, 0 to 5.00 Volts, 0.02 Volts per bit */
	U8MonitoredValue_t RightInverterThrottleFeedback;              /* 8 bits, 0-250, 0 to 5.00 Volts, 0.02 Volts per bit */
    /* Status and errors */ 
	BoolMonitoredValue_t IsCarInReverse;                                /* 1 bit, 0 means car is in FORWARD, 1 means car is in REVERSE */
	BoolMonitoredValue_t IsCarRunning;                                  /* 1 bit, 0 means car is RUNNING (Responding to accelerator pedal), 1 means car is IDLE (NOT responding to accelerator pedal)*/
	BoolMonitoredValue_t InvertersError;								/* 1 bit, 0 means SAFE, 1 means ERROR */
}InvertersMonitoredValues_t;

typedef struct{
	BoolMonitoredValue_t ActivationButtonPressed;                       /* 1 bit, 0 means button is NOT ACTIVATED, 1 means button is ACTIVATED */
	BoolMonitoredValue_t CarReverseCommandPressed;                      /* 1 bit, 0 means reverse command is NOT ACTIVATED, 1 means reverse command is ACTIVATED */
	BoolMonitoredValue_t IsDisplayWorking;                              /* 1 bit, 0 means display is WORKING, 1 means display is NOT WORKING */
	BoolMonitoredValue_t IsSegmentsDriverWorking;                       /* 1 bit, 0 means segments driver is WORKING, 1 means segments driver is NOT WORKING */
}DashboardMonitoredValues_t;

typedef struct{
	BoolMonitoredValue_t IsInvertersVCUSimulated;						/* 1 bit, 0 means NOT simulated, 1 means simulated */
	BoolMonitoredValue_t IsTsacVCUSimulated;							/* 1 bit, 0 means NOT simulated, 1 means simulated */
	BoolMonitoredValue_t IsDashboardVCUSimulated;						/* 1 bit, 0 means NOT simulated, 1 means simulated */
	BoolMonitoredValue_t IsPedalsVCUSimulated;							/* 1 bit, 0 means NOT simulated, 1 means simulated */
}CommunicationsMonitoredValues_t;

typedef struct{
	TsacMonitoredValues_t TsacMonitoredValues;
	PedalsMonitoredValues_t PedalsMonitoredValues;
	InvertersMonitoredValues_t InvertersMonitoredValues;
	DashboardMonitoredValues_t DashboardMonitoredValues;
	CommunicationsMonitoredValues_t CommunicationsMonitoredValues;
}MonitoredValues_t;


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
/*Takes a uint64_t argument and any xMonitoredValue_t type of argument.*/
#define WriteCanDataFromRawBufferAtAddress(rawBufferU64, xMonitoredValue_t_Address) \
	(xMonitoredValue_t_Address)->valueCan = ((((rawBufferU64) >> (xMonitoredValue_t_Address)->shift) & (~(0xFFFFFFFFFFFFFFFF << (xMonitoredValue_t_Address)->nrOfBits))) <= ((xMonitoredValue_t_Address)->maxValue)) ? \
			(((rawBufferU64) >> (xMonitoredValue_t_Address)->shift) & (~(0xFFFFFFFFFFFFFFFF << (xMonitoredValue_t_Address)->nrOfBits))) : \
			((xMonitoredValue_t_Address)->maxValue)

#define WriteUartDataFromRawBufferAtAddress(rawBufferU64, xMonitoredValue_t_Address) \
		(xMonitoredValue_t_Address)->valueUart = ((((rawBufferU64) >> (xMonitoredValue_t_Address)->shift) & (~(0xFFFFFFFFFFFFFFFF << (xMonitoredValue_t_Address)->nrOfBits))) <= ((xMonitoredValue_t_Address)->maxValue)) ? \
			(((rawBufferU64) >> (xMonitoredValue_t_Address)->shift) & (~(0xFFFFFFFFFFFFFFFF << (xMonitoredValue_t_Address)->nrOfBits))) : \
			((xMonitoredValue_t_Address)->maxValue)

#ifdef __cplusplus
}
#endif

#endif
