
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
#define LOWEST_PUMP_PRESSURE_LIMIT 10000U
#define HIGHEST_PUMP_PRESSURE_LIMIT 30000U
#define COOLING_TEMP_START 40
#define MAX_TEMP 55
#define SHUTDOWN_TEMP 60
#define PUMP_START_PERCENTAGE 50U

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
	uint16_t pres1, pres2;
	volatile uint8_t cooling_percentage;
	volatile int8_t left_motor_temp, right_motor_temp, left_inverter_temp, right_inverter_temp, max_temp, tmp1, tmp2;
	while(1){
		tmp1 = Cooling_ReadTemp(ONE);
		tmp2 = Cooling_ReadTemp(TWO);

		if(tmp1 < 0){
			tmp1 = 0;
		}
		if(tmp2 < 0){
			tmp2 = 0;
		}

		pres1 = Cooling_ReadPressure(ONE);
		pres2 = Cooling_ReadPressure(TWO);

		left_motor_temp = MonitoredValues.InvertersMonitoredValues.LeftMotorTemperature.valueCan - 30;
		right_motor_temp = MonitoredValues.InvertersMonitoredValues.RightMotorTemperature.valueCan - 30;
		left_inverter_temp = MonitoredValues.InvertersMonitoredValues.LeftInverterTemperature.valueCan - 40;
		right_inverter_temp = MonitoredValues.InvertersMonitoredValues.RightInverterTemperature.valueCan - 40;

		max_temp = left_motor_temp;
		if(max_temp < right_motor_temp){
			max_temp = right_motor_temp;
		}
		if(max_temp < left_inverter_temp){
			max_temp = left_inverter_temp;
		}
		if(max_temp < right_inverter_temp){
			max_temp = right_inverter_temp;
		}
		if(max_temp < tmp1){
			max_temp = tmp1;
		}
		if(max_temp < tmp2){
			max_temp = tmp2;
		}

		if(max_temp > COOLING_TEMP_START){
			cooling_percentage = ((max_temp - COOLING_TEMP_START) * 100U) / (MAX_TEMP - COOLING_TEMP_START);
			Cooling_SetFanSpeed(ONE, cooling_percentage);
			Cooling_SetFanSpeed(TWO, cooling_percentage);
			if(pres1 < LOWEST_PUMP_PRESSURE_LIMIT || pres2 < LOWEST_PUMP_PRESSURE_LIMIT || pres1 > HIGHEST_PUMP_PRESSURE_LIMIT || pres2 > HIGHEST_PUMP_PRESSURE_LIMIT){
				Cooling_SetPumpSpeed(ONE, 0);
				Cooling_SetPumpSpeed(TWO, 0);
			}
			else{
				if(cooling_percentage <= PUMP_START_PERCENTAGE){
					Cooling_SetPumpSpeed(ONE, PUMP_START_PERCENTAGE);
					Cooling_SetPumpSpeed(TWO, PUMP_START_PERCENTAGE);
				}
				else{
					Cooling_SetPumpSpeed(ONE, cooling_percentage);
					Cooling_SetPumpSpeed(TWO, cooling_percentage);
				}
			}
		}
		else{
			Cooling_SetFanSpeed(ONE, 0);
			Cooling_SetFanSpeed(TWO, 0);
		}

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
				//MonitoredValues.PedalsMonitoredValues.Brake_Implausibility.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_OutOfRangeOutput.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToGnd.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToVcc.valueCan /*|
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_OutOfRangeOutput.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToGnd.valueCan |
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToVcc.valueCan*/;

		activation_logic_pressed = MonitoredValues.DashboardMonitoredValues.ActivationButtonPressed.valueCan;
		dashboard_errors = !activation_logic_pressed;

		tsac_timeout = CanMessaging_GetBatteryReceiveTimeout();
		pedals_timeout = CanMessaging_GetPedalsReceiveTimeout();
		dashboard_timeout = CanMessaging_GetDashboardReceiveTimeout();

		errors = tsac_errors | tsac_timeout | pedals_errors | pedals_timeout | dashboard_errors | dashboard_timeout | (max_temp >= SHUTDOWN_TEMP);

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

		WriteCanDataAtAddress(Inverters_ReadAcceleration(LEFT_INVERTER), &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle);
		WriteCanDataAtAddress(Inverters_ReadAcceleration(RIGHT_INVERTER), &MonitoredValues.InvertersMonitoredValues.RightInverterThrottle);

		acc_sensor_1_travel_percentage = MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage.valueCan;
		acc_sensor_2_travel_percentage = MonitoredValues.PedalsMonitoredValues.AcceleratorSensor2TravelPercentage.valueCan;
		brake_sensor_1_travel_percentage = MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage.valueCan;
		//brake_sensor_2_travel_percentage = MonitoredValues.PedalsMonitoredValues.BrakeSensor1TravelPercentage.valueCan;

		if((brake_sensor_1_travel_percentage >= BRAKE_PERCENTAGE_THRESHOLD)/* || (brake_sensor_2_travel_percentage >= BRAKE_PERCENTAGE_THRESHOLD)*/){
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

		WriteCanDataAtAddress(((Inverters_GetState() == INVERTERS_OFF) | (directionState == INVERTERS_ERROR)), &MonitoredValues.InvertersMonitoredValues.InvertersError);
		WriteCanDataAtAddress(((directionState == INVERTERS_FORWARD) | (directionState == INVERTERS_REVERSE)), &MonitoredValues.InvertersMonitoredValues.IsCarRunning);
		WriteCanDataAtAddress((directionState == INVERTERS_REVERSE), &MonitoredValues.InvertersMonitoredValues.IsCarInReverse);
		Inverters_Update();
		CanMessaging_Update();
	}
}


#ifdef __cplusplus
}
#endif

/** @} */
