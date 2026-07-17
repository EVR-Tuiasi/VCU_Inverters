
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
#include "Dio.h"
#include "Mcl.h"
#include "Pwm.h"
#include "Adc.h"
#include "Inverters.h"

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/

/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/
#define BRAKE_LIGHT_PIN 89
#define DAC_WAKE_UP_PIN 139
#define MAX_DUTY_CYCLE 32768
#define THROTTLE1_CHANNEL 5
#define THROTTLE2_CHANNEL 7
#define BRAKE1_CHANNEL 4
#define BRAKE2_CHANNEL 6
#define FAN1_CHANNEL 0
#define FAN2_CHANNEL 1
#define PUMP1_CHANNEL 2
#define PUMP2_CHANNEL 3
#define RELAY_PIN 90
#define FWD1 145
#define FWD2 137
#define ECO1 123
#define ECO2 80
#define ACC1 58
#define ACC2 102
#define REV1 149
#define REV2 109
#define BRK1 135
#define BRK2 64
#define MAX_VOLTAGE 16383U //5 V pt adc de 14 biti
/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/
static Adc_ValueGroupType buffer0[4];
static Adc_ValueGroupType buffer1[2];

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
static void FanInit(void){
	Pwm_SetDutyCycle(FAN1_CHANNEL, 0);
	Pwm_SetDutyCycle(FAN2_CHANNEL, 0);
}
static void PumpInit(void){
	Pwm_SetDutyCycle(PUMP1_CHANNEL, 0);
	Pwm_SetDutyCycle(PUMP2_CHANNEL, 0);
}
static void Adc_ReadAll(void){
	volatile uint32_t i = 50000;

	Adc_StartGroupConversion(AdcGroup_0);
	Adc_StartGroupConversion(AdcGroup_1);

	while(Adc_GetGroupStatus(AdcGroup_0) == ADC_BUSY);
	while(Adc_GetGroupStatus(AdcGroup_1) == ADC_BUSY);

	Adc_ReadGroup(AdcGroup_0, buffer0);
	Adc_ReadGroup(AdcGroup_1, buffer1);
	while(i--);
	i = 50000;
}

/*==================================================================================================
*                                       GLOBAL FUNCTIONS
==================================================================================================*/
void BrakeLightTurnOn(void){
	Dio_WriteChannel(BRAKE_LIGHT_PIN, STD_HIGH);
}
void BrakeLightTurnOff(void){
	Dio_WriteChannel(BRAKE_LIGHT_PIN, STD_LOW);
}
void Dac_Init(void){
	Dio_WriteChannel(DAC_WAKE_UP_PIN, STD_HIGH);
	Pwm_SetDutyCycle(THROTTLE1_CHANNEL, 0);
	Pwm_SetDutyCycle(THROTTLE2_CHANNEL, 0);
	Pwm_SetDutyCycle(BRAKE1_CHANNEL, 0);
	Pwm_SetDutyCycle(BRAKE2_CHANNEL, 0);
}
void SetThrottle(PartNum num, uint8_t percentage){
	uint32_t value = ((uint32_t)percentage * 32768U) / 100U;
	switch(num){
		case ONE:
			Pwm_SetDutyCycle(THROTTLE1_CHANNEL, value);
			break;
		case TWO:
			Pwm_SetDutyCycle(THROTTLE2_CHANNEL, value);
			break;
	}
}
void SetBrake(PartNum num, uint8_t percentage){
	uint32_t value = ((uint32_t)percentage * 32768U) / 100U;
	switch(num){
		case ONE:
			Pwm_SetDutyCycle(BRAKE1_CHANNEL, value);
			break;
		case TWO:
			Pwm_SetDutyCycle(BRAKE2_CHANNEL, value);
			break;
	}
}
void SetFanSpeed(PartNum num, uint8_t percentage){
	uint32_t value = ((uint32_t)percentage * 32768U) / 100U;
	switch(num){
		case ONE:
			Pwm_SetDutyCycle(FAN1_CHANNEL, value);
			break;
		case TWO:
			Pwm_SetDutyCycle(FAN2_CHANNEL, value);
			break;
	}
}
void SetPumpSpeed(PartNum num, uint8_t percentage){
	uint32_t value = ((uint32_t)percentage * 32768U) / 100U;
	switch(num){
		case ONE:
			Pwm_SetDutyCycle(PUMP1_CHANNEL, value);
			break;
		case TWO:
			Pwm_SetDutyCycle(PUMP2_CHANNEL, value);
			break;
	}
}
void TurnRelayOn(void){
	Dio_WriteChannel(RELAY_PIN, STD_HIGH);
}
void TurnRelayOff(void){
	Dio_WriteChannel(RELAY_PIN, STD_LOW);
}

void FunctionOn(Function name, PartNum num){
	switch(name){
		case FORWARD:
			if(num == ONE)
				Dio_WriteChannel(FWD1, STD_LOW);
			else
				Dio_WriteChannel(FWD2, STD_LOW);
			break;
		case REVERSE:
			if(num == ONE)
				Dio_WriteChannel(REV1, STD_LOW);
			else
				Dio_WriteChannel(REV2, STD_LOW);
			break;
		case ECO:
			if(num == ONE)
				Dio_WriteChannel(ECO1, STD_LOW);
			else
				Dio_WriteChannel(ECO2, STD_LOW);
			break;
		case ACCELERATE:
			if(num == ONE)
				Dio_WriteChannel(ACC1, STD_LOW);
			else
				Dio_WriteChannel(ACC2, STD_LOW);
			break;
		case BRAKE:
			if(num == ONE)
				Dio_WriteChannel(BRK1, STD_LOW);
			else
				Dio_WriteChannel(BRK2, STD_LOW);
			break;
	}
}
void FunctionOff(Function name, PartNum num){
	switch(name){
		case FORWARD:
			if(num == ONE)
				Dio_WriteChannel(FWD1, STD_HIGH);
			else
				Dio_WriteChannel(FWD2, STD_HIGH);
			break;
		case REVERSE:
			if(num == ONE)
				Dio_WriteChannel(REV1, STD_HIGH);
			else
				Dio_WriteChannel(REV2, STD_HIGH);
			break;
		case ECO:
			if(num == ONE)
				Dio_WriteChannel(ECO1, STD_HIGH);
			else
				Dio_WriteChannel(ECO2, STD_HIGH);
			break;
		case ACCELERATE:
			if(num == ONE)
				Dio_WriteChannel(ACC1, STD_HIGH);
			else
				Dio_WriteChannel(ACC2, STD_HIGH);
			break;
		case BRAKE:
			if(num == ONE)
				Dio_WriteChannel(BRK1, STD_HIGH);
			else
				Dio_WriteChannel(BRK2, STD_HIGH);
			break;
		}
}
void Cooling_Init(){
	Adc_SetupResultBuffer(AdcGroup_0, buffer0);
	Adc_SetupResultBuffer(AdcGroup_1, buffer1);
	FanInit();
	PumpInit();
}
uint16_t ReadTemp(PartNum num){
	Adc_ReadAll();

	if(num == ONE)
		return buffer0[0];
	else
		return buffer0[1];
}
uint16_t ReadPressure(PartNum num){
	Adc_ReadAll();

	if(num == ONE)
		return buffer1[0];
	else
		return buffer0[2];
}
uint16_t ReadAcceleration(PartNum num){
	Adc_ReadAll();

	if(num == ONE)
		return buffer1[1];
	else
		return buffer0[3];
}

#ifdef __cplusplus
}
#endif



