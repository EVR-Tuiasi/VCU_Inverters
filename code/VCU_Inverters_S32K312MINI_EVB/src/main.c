
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

#define ID_MASK 			0x3FFFFFFF
#define SEND_MASK 			0x80000000
#define CAN_HTH_HANDLE  	0x00000001

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
uint8_t dataDeTrimisInvStDr[8]={0xBB, 0x7F, 0x08, 0xFA, 0xFB, 0x32, 0xFD, 0xFE}; //invertoare stanga si dreapta
uint8_t dataDeTrimisInv[8]={0x80, 0x00, 0x00, 0x00, 0x00, 0xFa, 0x0E, 0xD9}; //invert
uint8_t dataDeTrimisBord[8]={0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //bord
uint8_t dataDeTrimisAcc[8]={0x54, 0x07, 0x2B, 0x23, 0x2F, 0x23, 0x3E, 0x80}; //acceleratie
uint8_t dataDeTrimisFrana[8]={0x54, 0x03, 0x2B, 0x23, 0x2F, 0x23, 0x3E, 0x80}; //frana
uint8_t dataDeTrimisTSAC1[8]={0xFF, 0x3F, 0xBF, 0x67, 0x40, 0xBB, 0xBD, 0xE8}; //TSAC1
uint8_t dataDeTrimisTSAC2_0[8]={0xF8, 0x43, 0xE7, 0xF9, 0xFE, 0x7F, 0x9F, 0xE7}; //TSAC2 index 0
uint8_t dataDeTrimisTSAC2_1[8]={0xF9, 0x43, 0xE7, 0xF9, 0xFE, 0x7F, 0x9F, 0xE7}; //TSAC2 index 1
uint8_t dataDeTrimisTSAC3[8]={0xF8, 0x13, 0xE7, 0xF9, 0xFE, 0x7F, 0x9F, 0xE7}; //TSAC3
uint8_t dataDeTrimisTSAC4[8]={0x8A, 0xE2, 0xBF, 0xC1, 0xF8, 0x2F, 0xF8, 0x2F}; //TSAC4
uint8_t dataDeTrimisComm[8]={0xF0, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xFF, 0xE8}; //comunicatii

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
	CanMessaging_Test();
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
