
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

	while(1){
		volatile uint16_t tmp1 = Cooling_ReadTemp(ONE);
		volatile uint16_t tmp2 = Cooling_ReadTemp(TWO);

		volatile uint16_t pres1 = Cooling_ReadPressure(ONE);
		volatile uint16_t pres2 = Cooling_ReadPressure(TWO);

		reverse_command = MonitoredValues.DashboardMonitoredValues.CarReverseCommandPressed.valueCan;
		if(reverse_command){
			Inverters_Reverse();
		}
		else{
			Inverters_Forward();
		}

		if(
				MonitoredValues.TsacMonitoredValues.AmsError.valueCan ||
				MonitoredValues.TsacMonitoredValues.Bms0Error.valueCan ||
				MonitoredValues.TsacMonitoredValues.Bms1Error.valueCan ||
				MonitoredValues.TsacMonitoredValues.ShuntError.valueCan ||
				MonitoredValues.TsacMonitoredValues.ThermistorsError.valueCan ||
				MonitoredValues.TsacMonitoredValues.TransceiverError.valueCan
				){
			tsac_errors = 1;
		}
		else{
			tsac_errors = 0;
		}

		if(
				MonitoredValues.PedalsMonitoredValues.Accel_Implausibility.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_OutOfRangeOutput.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToGnd.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor1_ShortToVcc.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_OutOfRangeOutput.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToGnd.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Accel_Sensor2_ShortToVcc.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Brake_Implausibility.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_OutOfRangeOutput.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToGnd.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor1_ShortToVcc.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_OutOfRangeOutput.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToGnd.valueCan ||
				MonitoredValues.PedalsMonitoredValues.Brake_Sensor2_ShortToVcc.valueCan
				){
			pedals_errors = 1;
		}
		else{
			pedals_errors = 0;
		}

		activation_logic_pressed = MonitoredValues.DashboardMonitoredValues.ActivationButtonPressed.valueCan;
		if(!activation_logic_pressed){
			dashboard_errors = 1;
		}
		else{
			dashboard_errors = 0;
		}

		tsac_timeout = CanMessaging_GetBatteryReceiveTimeout();
		pedals_timeout = CanMessaging_GetPedalsReceiveTimeout();
		dashboard_timeout = CanMessaging_GetDashboardReceiveTimeout();

		if(
				tsac_errors || tsac_timeout ||
				pedals_errors || pedals_timeout ||
				dashboard_errors || dashboard_timeout
				){
			errors = 1;
		}
		else{
			errors = 0;
		}

		if(errors){
			shutdown_error_flag = 1;
			Shutdown();
		}
		else{
			if(shutdown_error_flag){
				shutdown_error_flag = 0;
				Recover(reverse_command);
			}
		}


		WriteCanDataAtAddress((uint8_t)((((uint32_t)Inverters_ReadAcceleration(LEFT_INVERTER)) * (uint32_t)250U) / (uint32_t)16383U), &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle);
		WriteCanDataAtAddress((uint8_t)((((uint32_t)Inverters_ReadAcceleration(RIGHT_INVERTER)) * (uint32_t)250U) / (uint32_t)16383U), &MonitoredValues.InvertersMonitoredValues.RightInverterThrottle);
		Inverters_SetThrottle(LEFT_INVERTER, MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage.valueCan);
		Inverters_SetThrottle(RIGHT_INVERTER, MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage.valueCan);

		Inverters_Update();
		CanMessaging_Update();
	}
}


#ifdef __cplusplus
}
#endif

/** @} */
