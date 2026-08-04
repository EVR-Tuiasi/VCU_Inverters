
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
	CanMessaging_Test();
	UartMessaging_Init();

	Inverters_Init();

	//BrakeLight_Test();
	//Cooling_Test();
	//Inverters_Test();
	volatile uint64_t i;
	while(1){
		/*volatile uint16_t tmp1 = Cooling_ReadTemp(ONE);
		volatile uint16_t tmp2 = Cooling_ReadTemp(TWO);

		volatile uint16_t pres1 = Cooling_ReadPressure(ONE);
		volatile uint16_t pres2 = Cooling_ReadPressure(TWO);
		*/
		WriteCanDataAtAddress((uint8_t)((((uint32_t)Inverters_ReadAcceleration(LEFT_INVERTER)) * (uint32_t)250U) / (uint32_t)16383U), &MonitoredValues.InvertersMonitoredValues.LeftInverterThrottle);
		WriteCanDataAtAddress((uint8_t)((((uint32_t)Inverters_ReadAcceleration(RIGHT_INVERTER)) * (uint32_t)250U) / (uint32_t)16383U), &MonitoredValues.InvertersMonitoredValues.RightInverterThrottle);
		Inverters_SetThrottle(LEFT_INVERTER, MonitoredValues.PedalsMonitoredValues.AcceleratorSensor1TravelPercentage.valueCan);

		Inverters_Update();
		CanMessaging_Update();
		i=2000000;
		while(i--);
	}
}


#ifdef __cplusplus
}
#endif

/** @} */
