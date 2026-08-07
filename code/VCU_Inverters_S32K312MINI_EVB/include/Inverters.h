#ifndef INVERTERS_H_
#define INVERTERS_H_

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
#include"Mcu.h"
/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
typedef enum{
	ONE,
	TWO
}PartNum_t;

typedef enum{
	LEFT_INVERTER,
	RIGHT_INVERTER
}Inverter_t;

typedef enum{
	FORWARD,
	REVERSE,
	ECO,
	ACCELERATE,
	BRAKE
}Function_t;

typedef enum{
	INVERTERS_OFF,
	INVERTERS_WAITING_FOR_VOLTAGE,
	INVERTERS_STARTING,
	INVERTERS_DELAY,
	INVERTERS_ON
}InvertersState_t;

typedef enum{
	INVERTERS_DIRECTION_FORWARD,
	INVERTERS_DIRECTION_REVERSE
}InvertersDirection_t;
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
void BrakeLight_SetState(bool value);
void BrakeLight_Test(void);

void Inverters_Init(void);
void Inverters_Test(void);
void Inverters_SetThrottle(Inverter_t inverter, uint8_t percentage);
void Inverters_SetBrake(Inverter_t inverter, uint8_t percentage);
void Inverters_SetPower(bool value);
void Inverters_SetFunction(Function_t name, Inverter_t inverter, bool value);
uint8_t Inverters_ReadAcceleration(Inverter_t inverter);
void Inverters_Timer_Timeout(void);
void Inverters_Update(void);
InvertersState_t Inverters_GetState(void);
void Inverters_ResetTimer(void);
void Inverters_ResetCanTimer(void);
void Inverters_Forward(void);
void Inverters_Reverse(void);
void Inverters_Shutdown(void);
void Inverters_SetDirection(InvertersDirection_t direction);

void Cooling_Init(void);
void Cooling_Test(void);
void Cooling_SetFanSpeed(PartNum_t num, uint8_t percentage);
void Cooling_SetPumpSpeed(PartNum_t num, uint8_t percentage);
int8_t Cooling_ReadTemp(PartNum_t num);
uint32_t Cooling_ReadPressure(PartNum_t num);

void CanMessaging_Inverters_Timeout(void);

#ifdef __cplusplus
}
#endif

#endif
