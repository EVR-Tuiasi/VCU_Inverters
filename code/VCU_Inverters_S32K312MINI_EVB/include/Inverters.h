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
}PartNum;

typedef enum{
	LEFT_INVERTER,
	RIGHT_INVERTER
}Inverter;

typedef enum{
	FORWARD,
	REVERSE,
	ECO,
	ACCELERATE,
	BRAKE
}Function;
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
void Inverters_SetThrottle(Inverter inverter, uint8_t percentage);
void Inverters_SetBrake(Inverter inverter, uint8_t percentage);
void Inverters_SetPower(bool value);
void Inverters_SetFunction(Function name, Inverter inverter, bool value);
uint16_t Inverters_ReadAcceleration(Inverter inverter);

void Cooling_Init(void);
void Cooling_Test(void);
void Cooling_SetFanSpeed(PartNum num, uint8_t percentage);
void Cooling_SetPumpSpeed(PartNum num, uint8_t percentage);
uint16_t Cooling_ReadTemp(PartNum num);
uint16_t Cooling_ReadPressure(PartNum num);

#ifdef __cplusplus
}
#endif

#endif
