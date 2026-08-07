#ifdef __cplusplus
extern "C"{
#endif


/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/

#include "Messaging.h"

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
MonitoredValues_t MonitoredValues ={
		{/* START TsacMonitoredValues */
				{/* MedianCellTemperature */
						0,	   /* value */
						0,	   /* value */
						10U,   /* nrOfBits */
						54U,   /* shift */
						1023U  /* maxValue */
				},
				{/* HighestCellTemperature */
						0,	   /* value */
						0,	   /* value */
						10U,   /* nrOfBits */
						24U,   /* shift */
						1023U  /* maxValue */
				},
				{/* LowestCellTemperature */
						0,	   /* value */
						0,	   /* value */
						10U,   /* nrOfBits */
						54U,   /* shift */
						1023U  /* maxValue */
				},
				{/* MedianCellVoltage */
						0,	   /* value */
						0,	   /* value */
						10U,   /* nrOfBits */
						44U,   /* shift */
						1023U  /* maxValue */
				},
				{/* HighestCellVoltage */
						0,	   /* value */
						0,	   /* value */
						10U,   /* nrOfBits */
						34U,   /* shift */
						1023U  /* maxValue */
				},
				{/* LowestCellVoltage */
						0,	   /* value */
						0,	   /* value */
						10U,   /* nrOfBits */
						44U,   /* shift */
						1023U  /* maxValue */
				},
				{/* OverallVoltage */
						0,	   /* value */
						0,	   /* value */
						11,	   /* nrOfBits */
						13U,   /* shift */
						2047U  /* maxValue */
				},
				{/* OverallCurrent */
						0,	   /* value */
						0,	   /* value */
						13U,   /* nrOfBits */
						0U,    /* shift */
						8095U  /* maxValue */
				},
				{/* CellVoltage */
						{0}    /* zero-initializing array */
				},
				{/* ThermistorTemperature */
						{0}	   /* zero-initializing array */
				},
				{/* AmsError */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						38U,   /* shift */
						1U     /* maxValue */
				},
				{/* TransceiverError */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						42U,   /* shift */
						1U     /* maxValue */
				},
				{/* ShuntError */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						43U,   /* shift */
						1U     /* maxValue */
				},
				{/* Bms0Error */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						41U,   /* shift */
						1U     /* maxValue */
				},
				{/* Bms1Error */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						40U,   /* shift */
						1U     /* maxValue */
				},
				{/* ThermistorsError */
						0,	   /* value */
						0,	   /* value */
						1U,    /* nrOfBits */
						39U,   /* shift */
						1U     /* maxValue */
				},
				{/* ChargerStatus */
						0,	   /* value */
						0,	   /* value */
						1U,    /* nrOfBits */
						32U,   /* shift */
						1U     /* maxValue */
				},
				{/* ReportedChargingCurrent */
						0,	   /* value */
						0,	   /* value */
						16U,   /* nrOfBits */
						16U,   /* shift */
						65535U /* maxValue */
				},
				{/* ReportedChargingVolts */
						0,	   /* value */
						0,	   /* value */
						16U,   /* nrOfBits */
						0U,	   /* shift */
						65535U /* maxValue */
				},
				{/* ChargerCommand */
						0,	   /* value */
						0,	   /* value */
						1U,    /* nrOfBits */
						63U,   /* shift */
						1U     /* maxValue */
				},
				{/* DesiredChargingCurrent */
						0,	   /* value */
						0,	   /* value */
						9U,    /* nrOfBits */
						10U,   /* shift */
						320U   /* maxValue */
				},
				{/* DesiredChargingVoltage */
						0,	   /* value */
						0,	   /* value */
						10U,   /* nrOfBits */
						0U,	   /* shift */
						1008U  /* maxValue */
				}
		},/* END TsacMonitoredValues */
		{/* START PedalsMonitoredValues*/
				{/* AcceleratorSensor1Voltage */
						0,	   /* value */
						0,	   /* value */
						14U,   /* nrOfBits */
						0U,	   /* shift */
						16383U /* maxValue */
				},
				{/* AcceleratorSensor2Voltage */
						0,	   /* value */
						0,	   /* value */
						14U,   /* nrOfBits */
						14U,   /* shift */
						16383U /* maxValue */
				},
				{/* AcceleratorSensor1TravelPercentage */
						0,	   /* value */
						0,	   /* value */
						7U,	   /* nrOfBits */
						28U,   /* shift */
						100U   /* maxValue */
				},
				{/* AcceleratorSensor2TravelPercentage */
						0,	   /* value */
						0,	   /* value */
						7U,	   /* nrOfBits */
						35U,   /* shift */
						100U   /* maxValue */
				},
				{/* BrakeSensor1Voltage */
						0,	   /* value */
						0,	   /* value */
						14U,   /* nrOfBits */
						0U,	   /* shift */
						16383U /* maxValue */
				},
				{/* BrakeSensor2Voltage */
						0,	   /* value */
						0,	   /* value */
						14U,   /* nrOfBits */
						14U,   /* shift */
						16383U /* maxValue */
				},
				{/* BrakeSensor1TravelPercentage */
						0,	   /* value */
						0,	   /* value */
						7U,	   /* nrOfBits */
						28U,   /* shift */
						100U   /* maxValue */
				},
				{/* BrakeSensor2TravelPercentage */
						0,	   /* value */
						0,	   /* value */
						7U,	   /* nrOfBits */
						35U,   /* shift */
						100U   /* maxValue */
				},
				{/* PressureSensorVoltage */
						0,	   /* value */
						0,	   /* value */
						9U,	   /* nrOfBits */
						42U,   /* shift */
						500U   /* maxValue */
				},
				{/* PressureSensorBars */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						42U,   /* shift */
						255U   /* maxValue */
				},
				{/* Accel_Sensor1_ShortToGnd */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						63U,   /* shift */
						1U     /* maxValue */
				},
				{/* Accel_Sensor1_ShortToVcc */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						62U,   /* shift */
						1U     /* maxValue */
				},
				{/* Accel_Sensor1_OutOfRangeOutput */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						61U,   /* shift */
						1U     /* maxValue */
				},
				{/* Accel_Sensor2_ShortToGnd */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						60U,   /* shift */
						1U     /* maxValue */
				},
				{/* Accel_Sensor2_ShortToVcc */
						0,     /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						59U,   /* shift */
						1U     /* maxValue */
				},
				{/* Accel_Sensor2_OutOfRangeOutput */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						58U,   /* shift */
						1U     /* maxValue */
				},
				{/* Accel_Implausibility */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						57U,   /* shift */
						1U     /* maxValue */
				},
				{/* Brake_Sensor1_ShortToGnd */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						63U,   /* shift */
						1U     /* maxValue */
				},
				{/* Brake_Sensor1_ShortToVcc */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						62U,   /* shift */
						1U     /* maxValue */
				},
				{/* Brake_Sensor1_OutOfRangeOutput */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						61U,   /* shift */
						1U     /* maxValue */
				},
				{/* Brake_Sensor2_ShortToGnd */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						60U,   /* shift */
						1U     /* maxValue */
				},
				{/* Brake_Sensor2_ShortToVcc */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						59U,   /* shift */
						1U     /* maxValue */
				},
				{/* Brake_Sensor2_OutOfRangeOutput */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						58U,   /* shift */
						1U     /* maxValue */
				},
				{/* Brake_Implausibility */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						57U,   /* shift */
						1U     /* maxValue */
				}
		},/* END PedalsMonitoredValues*/
		{/* START InvertersMonitoredValues */
				{/* LeftInverterTemperature */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						8U,	   /* shift */
						255U   /* maxValue */
				},
				{/* LeftMotorTemperature */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						0U,	   /* shift */
						255U   /* maxValue */
				},
				{/* LeftInverterInputVoltage */
						0,	   /* value */
						0,	   /* value */
						11U,   /* nrOfBits */
						40U,   /* shift */
						1800U  /* maxValue */
				},
				{/* LeftInverterCurrent */
						0,	   /* value */
						0,	   /* value */
						12U,   /* nrOfBits */
						0U,	   /* shift */
						4000U  /* maxValue */
				},
				{/* LeftMotorRpm */
						0,	   /* value */
						0,	   /* value */
						13U,   /* nrOfBits */
						51U,   /* shift */
						6000U  /* maxValue */
				},
				{/* LeftMotorSpeedKmh */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						24U,   /* shift */
						255U   /* maxValue */
				},
				{/* LeftInverterThrottle */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						16U,   /* shift */
						250U   /* maxValue */
				},
				{/* LeftInverterThrottleFeedback */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						32U,   /* shift */
						250U   /* maxValue */
				},
				{/* RightInverterTemperature */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						8U,	   /* shift */
						255U   /* maxValue */
				},
				{/* RightMotorTemperature */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						0U,	   /* shift */
						255U   /* maxValue */
				},
				{/* RightInverterInputVoltage */
						0,	   /* value */
						0,	   /* value */
						11U,   /* nrOfBits */
						40U,   /* shift */
						1800U  /* maxValue */
				},
				{/* RightInverterCurrent */
						0,	   /* value */
						0,	   /* value */
						12U,   /* nrOfBits */
						12U,   /* shift */
						4000U  /* maxValue */
				},
				{/* RightMotorRpm */
						0,	   /* value */
						0,	   /* value */
						13U,   /* nrOfBits */
						51U,   /* shift */
						6000U  /* maxValue */
				},
				{/* RightMotorSpeedKmh */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						24U,   /* shift */
						255U   /* maxValue */
				},
				{/* RightInverterThrottle */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						16U,   /* shift */
						250U   /* maxValue */
				},
				{/* RightInverterThrottleFeedback */
						0,	   /* value */
						0,	   /* value */
						8U,	   /* nrOfBits */
						32U,   /* shift */
						250U   /* maxValue */
				},
				{/* IsCarInReverse */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						62U,   /* shift */
						1U     /* maxValue */
				},
				{/* IsCarRunning */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						63U,   /* shift */
						1U     /* maxValue */
				},
				{/* InvertersError */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						61U,   /* shift */
						1U     /* maxValue */
				}
		},/* END InvertersMonitoredValues */
		{/* START DashboardMonitoredValues */
				{/* ActivationButtonPressed */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						63U,   /* shift */
						1U     /* maxValue */
				},
				{/* CarReverseCommandPressed */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						62U,   /* shift */
						1U     /* maxValue */
				},
				{/* IsDisplayWorking */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						61U,   /* shift */
						1U     /* maxValue */
				},
				{/* IsSegmentsDriverWorking */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						60U,   /* shift */
						1U     /* maxValue */
				}
		},/* END DashboardMonitoredValues */
		{/* START CommunicationsMonitoredValues */
				{/* IsInvertersVCUSimulated */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						63U,   /* shift */
						1U     /* maxValue */
				},
				{/* IsTsacVCUSimulated */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						62U,   /* shift */
						1U     /* maxValue */
				},
				{/* IsDashboardVCUSimulated */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						61U,   /* shift */
						1U     /* maxValue */
				},
				{/* IsPedalsVCUSimulated */
						0,	   /* value */
						0,	   /* value */
						1U,	   /* nrOfBits */
						60U,   /* shift */
						1U     /* maxValue */
				}
		}/* END CommunicationsMonitoredValues */
};

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/


/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/


#ifdef __cplusplus
}
#endif
