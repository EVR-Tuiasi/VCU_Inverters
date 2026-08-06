
#ifdef __cplusplus
extern "C" {
#endif


/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
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
#include "Adc.h"
#include "Pwm.h"
#include "Gpt.h"
#include "CanMessaging.h"
#include "UartMessaging.h"
#include "Messaging.h"
#include "Inverters.h"

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
#define BRAKE_PERCENTAGE_THRESHOLD 10U

typedef enum{
	INVERTERS_ERROR,
	INVERTERS_FORWARD,
	INVERTERS_REVERSE
} InvertersDirectionState_t;
/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/


/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/
bool shutdown_error_flag = 0;

/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/
/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
extern MonitoredValues_t MonitoredValues;
InvertersDirectionState_t directionState = INVERTERS_FORWARD;

/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


/*==================================================================================================
*                                       LOCAL FUNCTIONS
==================================================================================================*/


/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/


int main(void)
{
	Mcu_Init(NULL_PTR);
	Mcu_InitClock(McuModeSettingConf_0);
	while(MCU_PLL_LOCKED != Mcu_GetPllStatus())
	{
		 ;
	}
	Mcu_DistributePllClock();
	Mcu_SetMode(McuModeSettingConf_0);
	Mcl_Init(NULL_PTR);
	Port_Init(NULL_PTR);
	Platform_Init(NULL_PTR);
	Adc_Init(NULL_PTR);
	Can_43_FLEXCAN_Init(NULL_PTR);
	CanIf_Init(NULL_PTR);
	Pwm_Init(NULL_PTR);
	Gpt_Init(NULL_PTR);

	Cooling_Init();
	CanMessaging_Init();
	UartMessaging_Init();

	Inverters_Init();

	//BrakeLight_Test();
	//Cooling_Test();
	//Inverters_Test();

	bool errors, tsac_errors, pedals_errors, dashboard_errors;
	bool tsac_timeout, pedals_timeout, dashboard_timeout;
	bool activation_logic_pressed, reverse_command;

	uint8_t left_motor_speed, right_motor_speed, acc_sensor_1_travel_percentage, acc_sensor_2_travel_percentage, brake_sensor_1_travel_percentage, brake_sensor_2_travel_percentage, travel_percentage;
	volatile uint16_t tmp1, tmp2, pres1, pres2;

	while(1){
		tmp1 = Cooling_ReadTemp(ONE);
		tmp2 = Cooling_ReadTemp(TWO);

		pres1 = Cooling_ReadPressure(ONE);
		pres2 = Cooling_ReadPressure(TWO);

		reverse_command = MonitoredValues.DashboardMonitoredValues.CarReverseCommandPressed.valueCan;

		tsac_errors =
				MonitoredValues.TsacMonitoredValues.AmsError.valueCan |
				MonitoredValues.TsacMonitoredValues.Bms0Error.valueCan |
				MonitoredValues.TsacMonitoredValues.Bms1Error.valueCan |
				MonitoredValues.TsacMonitoredValues.ShuntError.valueCan |
				MonitoredValues.TsacMonitoredValues.ThermistorsError.valueCan |
				MonitoredValues.TsacMonitoredValues.TransceiverError.valueCan;

		pedals_errors =
				MonitoredValues.PedalsMonitoredValues.Accel_Implausibility.valueCan |
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_OutOfRangeOutput.valueCan |
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToGnd.valueCan |
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToVcc.valueCan |
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_OutOfRangeOutput.valueCan |
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToGnd.valueCan |
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToVcc.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Implausibility.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_OutOfRangeOutput.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToGnd.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToVcc.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_OutOfRangeOutput.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToGnd.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToVcc.valueCan;

		activation_logic_pressed = MonitoredValues.DashboardMonitoredValues.ActivationButtonPressed.valueCan;
		dashboard_errors = !activation_logic_pressed;

		tsac_timeout = CanMessaging_GetBatteryReceiveTimeout();
		pedals_timeout = CanMessaging_GetPedalsReceiveTimeout();
		dashboard_timeout = CanMessaging_GetDashboardReceiveTimeout();

		errors = /*tsac_errors | tsac_timeout |*/ pedals_errors | pedals_timeout | dashboard_errors | dashboard_timeout;

		switch(directionState){
			case INVERTERS_FORWARD:
				if((errors == 1) || (reverse_command == 1)){
					Inverters_Shutdown();
					directionState = INVERTERS_ERROR;
				}
				break;
			case INVERTERS_REVERSE:
				if((errors == 1) || (reverse_command == 0)){
					Inverters_Shutdown();
					directionState = INVERTERS_ERROR;
				}
				break;
			case INVERTERS_ERROR:
				left_motor_speed = MonitoredValues.InvertersMonitoredValues.LeftMotorSpeedKmh.valueCan;
				right_motor_speed = MonitoredValues.InvertersMonitoredValues.RightMotorSpeedKmh.valueCan;
				if((errors == 0) && (reverse_command == 0) && (left_motor_speed == 0) && (right_motor_speed == 0)){
					Inverters_SetDirection(INVERTERS_DIRECTION_FORWARD);
					directionState = INVERTERS_FORWARD;
				}
				if((errors == 0) && (reverse_command == 1) && (left_motor_speed == 0) && (right_motor_speed == 0)){
					Inverters_SetDirection(INVERTERS_DIRECTION_REVERSE);
					directionState = INVERTERS_REVERSE;
				}
				break;
		}

		WriteCanDataAtAddress((uint8_t)((((uint32_t)Inverters_ReadAcceleration(LEFT_INVERTER)) * (uint32_t)250U) / (uint32_t)16383U), &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle);
		WriteCanDataAtAddress((uint8_t)((((uint32_t)Inverters_ReadAcceleration(RIGHT_INVERTER)) * (uint32_t)250U) / (uint32_t)16383U), &MonitoredValues.InvertersMonitoredValues.RightInverterThrottle);

		acc_sensor_1_travel_percentage = MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage.valueCan;
		acc_sensor_2_travel_percentage = MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage.valueCan;
		brake_sensor_1_travel_percentage = MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage.valueCan;
		brake_sensor_2_travel_percentage = MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage.valueCan;

		if((brake_sensor_1_travel_percentage >= BRAKE_PERCENTAGE_THRESHOLD) || (brake_sensor_2_travel_percentage >= BRAKE_PERCENTAGE_THRESHOLD)){
			travel_percentage = 0U;
			BrakeLight_SetState(STD_ON);
		}
		else{
			BrakeLight_SetState(STD_OFF);
			travel_percentage = acc_sensor_1_travel_percentage;
			if(travel_percentage > acc_sensor_2_travel_percentage){
				travel_percentage = acc_sensor_2_travel_percentage;
			}
			if(directionState == INVERTERS_REVERSE){
				travel_percentage /= 2U;
			}
		}

		if((directionState == INVERTERS_FORWARD) || (directionState == INVERTERS_REVERSE)){
			Inverters_SetThrottle(LEFT_INVERTER, travel_percentage);
			Inverters_SetThrottle(RIGHT_INVERTER, travel_percentage);
		}

		Inverters_Update();
		CanMessaging_Update();
	}
}


#ifdef __cplusplus
}
#endif

/** @} */
