
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
#define BRAKE_LIGHT_PIN 89U
#define DAC_WAKE_UP_PIN 139U
#define MAX_DUTY_CYCLE 32768U
#define THROTTLE1_CHANNEL 1U
#define THROTTLE2_CHANNEL 3U
#define BRAKE1_CHANNEL 0U
#define BRAKE2_CHANNEL 2U
#define FAN1_CHANNEL 4U
#define FAN2_CHANNEL 5U
#define PUMP1_CHANNEL 6U
#define PUMP2_CHANNEL 7U
#define RELAY_PIN 90U
#define FWD_LEFT 145U
#define FWD_RIGHT 137U
#define ECO_LEFT 123U
#define ECO_RIGHT 80U
#define ACC_LEFT 58U
#define ACC_RIGHT 102U
#define REV_LEFT 149U
#define REV_RIGHT 109U
#define BRK_LEFT 135U
#define BRK_RIGHT 64U
#define MAX_VOLTAGE 16383U //5 V pt adc de 14 biti
#define ADC_TEMP_1 AdcGroup_0
#define ADC_TEMP_2 AdcGroup_2
#define ADC_PRESSURE_1 AdcGroup_1
#define ADC_PRESSURE_2 AdcGroup_3
#define ADC_ACCELERATION_LEFT AdcGroup_5
#define ADC_ACCELERATION_RIGHT AdcGroup_4
/*==================================================================================================
*                                      LOCAL CONSTANTS
==================================================================================================*/


/*==================================================================================================
*                                      LOCAL VARIABLES
==================================================================================================*/
static Adc_ValueGroupType temp1;
static Adc_ValueGroupType temp2;
static Adc_ValueGroupType pres1;
static Adc_ValueGroupType pres2;
static Adc_ValueGroupType accLeft;
static Adc_ValueGroupType accRight;

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
void BrakeLight_SetState(bool value){
	if(value == STD_ON){
		Dio_WriteChannel(BRAKE_LIGHT_PIN, STD_HIGH);
	}
	else{
		Dio_WriteChannel(BRAKE_LIGHT_PIN, STD_LOW);
	}
}
void Inverters_Init(void){
	Pwm_SetDutyCycle(THROTTLE1_CHANNEL, 0U);
	Pwm_SetDutyCycle(THROTTLE2_CHANNEL, 0U);
	Pwm_SetDutyCycle(BRAKE1_CHANNEL, 0U);
	Pwm_SetDutyCycle(BRAKE2_CHANNEL, 0U);
	Adc_SetupResultBuffer(ADC_ACCELERATION_LEFT, &accLeft);
	Adc_SetupResultBuffer(ADC_ACCELERATION_RIGHT, &accRight);
	Dio_WriteChannel(DAC_WAKE_UP_PIN, STD_HIGH);
}
void Inverters_SetThrottle(Inverter inverter, uint8_t percentage){
	if(percentage > 100U){
		percentage = 100U;
	}
	uint32_t value = ((uint32_t)percentage * MAX_DUTY_CYCLE) / 100U;
	switch(inverter){
		case LEFT_INVERTER:
			Pwm_SetDutyCycle(THROTTLE1_CHANNEL, value);
			break;
		case RIGHT_INVERTER:
			Pwm_SetDutyCycle(THROTTLE2_CHANNEL, value);
			break;
	}
}
void Inverters_SetBrake(Inverter inverter, uint8_t percentage){
	if(percentage > 100U){
			percentage = 100U;
	}
	uint32_t value = ((uint32_t)percentage * 32768U) / 100U;
	switch(inverter){
		case LEFT_INVERTER:
			Pwm_SetDutyCycle(BRAKE1_CHANNEL, value);
			break;
		case RIGHT_INVERTER:
			Pwm_SetDutyCycle(BRAKE2_CHANNEL, value);
			break;
	}
}
void Cooling_SetFanSpeed(PartNum num, uint8_t percentage){
	if(percentage > 100U){
		percentage = 100U;
	}
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
void Cooling_SetPumpSpeed(PartNum num, uint8_t percentage){
	if(percentage > 100U){
		percentage = 100U;
	}
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
void Inverters_SetPower(bool value){
	if(value == STD_ON){
		Dio_WriteChannel(RELAY_PIN, STD_HIGH);
	}
	else{
		Dio_WriteChannel(RELAY_PIN, STD_LOW);
	}
}

void Inverters_SetFunction(Function name, Inverter inverter, bool value){
	Dio_LevelType pinValue = STD_HIGH;
	if(value == STD_ON){
		pinValue = STD_LOW;
	}
	switch(name){
		case FORWARD:
			if(inverter == LEFT_INVERTER)
				Dio_WriteChannel(FWD_LEFT, pinValue);
			else
				Dio_WriteChannel(FWD_RIGHT, pinValue);
			break;
		case REVERSE:
			if(inverter == LEFT_INVERTER)
				Dio_WriteChannel(REV_LEFT, pinValue);
			else
				Dio_WriteChannel(REV_RIGHT, pinValue);
			break;
		case ECO:
			if(inverter == LEFT_INVERTER)
				Dio_WriteChannel(ECO_LEFT, pinValue);
			else
				Dio_WriteChannel(ECO_RIGHT, pinValue);
			break;
		case ACCELERATE:
			if(inverter == LEFT_INVERTER)
				Dio_WriteChannel(ACC_LEFT, pinValue);
			else
				Dio_WriteChannel(ACC_RIGHT, pinValue);
			break;
		case BRAKE:
			if(inverter == LEFT_INVERTER)
				Dio_WriteChannel(BRK_LEFT, pinValue);
			else
				Dio_WriteChannel(BRK_RIGHT, pinValue);
			break;
	}
}
void Cooling_Init(){
	Adc_SetupResultBuffer(ADC_TEMP_1, &temp1);
	Adc_SetupResultBuffer(ADC_TEMP_2, &temp2);
	Adc_SetupResultBuffer(ADC_PRESSURE_1, &pres1);
	Adc_SetupResultBuffer(ADC_PRESSURE_2, &pres2);
	Pwm_SetDutyCycle(FAN1_CHANNEL, 0U);
	Pwm_SetDutyCycle(FAN2_CHANNEL, 0U);
	Pwm_SetDutyCycle(PUMP1_CHANNEL, 0U);
	Pwm_SetDutyCycle(PUMP2_CHANNEL, 0U);
}
uint16_t Cooling_ReadTemp(PartNum num){
	if(num == ONE){
		Adc_StartGroupConversion(ADC_TEMP_1);
		while(Adc_GetGroupStatus(ADC_TEMP_1) == ADC_BUSY);
		Adc_ReadGroup(ADC_TEMP_1, &temp1);
		return temp1;
	}
	else{
		Adc_StartGroupConversion(ADC_TEMP_2);
		while(Adc_GetGroupStatus(ADC_TEMP_2) == ADC_BUSY);
		Adc_ReadGroup(ADC_TEMP_2, &temp2);
		return temp2;
	}
}
uint16_t Cooling_ReadPressure(PartNum num){
	if(num == ONE){
		Adc_StartGroupConversion(ADC_PRESSURE_1);
		while(Adc_GetGroupStatus(ADC_PRESSURE_1) == ADC_BUSY);
		Adc_ReadGroup(ADC_PRESSURE_1, &pres1);
		return pres1;
	}
	else{
		Adc_StartGroupConversion(ADC_PRESSURE_2);
		while(Adc_GetGroupStatus(ADC_PRESSURE_2) == ADC_BUSY);
		Adc_ReadGroup(ADC_PRESSURE_2, &pres2);
		return pres2;
	}
}
uint16_t Inverters_ReadAcceleration(Inverter inverter){
	if(inverter == LEFT_INVERTER){
		Adc_StartGroupConversion(ADC_ACCELERATION_LEFT);
		while(Adc_GetGroupStatus(ADC_ACCELERATION_LEFT) == ADC_BUSY);
		Adc_ReadGroup(ADC_ACCELERATION_LEFT, &accLeft);
		return accLeft;
	}
	else{
		Adc_StartGroupConversion(ADC_ACCELERATION_RIGHT);
		while(Adc_GetGroupStatus(ADC_ACCELERATION_RIGHT) == ADC_BUSY);
		Adc_ReadGroup(ADC_ACCELERATION_RIGHT, &accRight);
		return accRight;
	}
}
void Cooling_Test(void){
	while(1){
		volatile uint64_t delay;
		for(uint64_t i=0;i<=MAX_DUTY_CYCLE;i++){
			Pwm_SetDutyCycle(FAN1_CHANNEL, i);
			Pwm_SetDutyCycle(FAN2_CHANNEL, i);
			Pwm_SetDutyCycle(PUMP1_CHANNEL, i);
			Pwm_SetDutyCycle(PUMP2_CHANNEL, i);
			delay=1000;
			while(delay--);
		}
		for(uint64_t i=MAX_DUTY_CYCLE;i>0;i--){
			Pwm_SetDutyCycle(FAN1_CHANNEL, i);
			Pwm_SetDutyCycle(FAN2_CHANNEL, i);
			Pwm_SetDutyCycle(PUMP1_CHANNEL, i);
			Pwm_SetDutyCycle(PUMP2_CHANNEL, i);
			delay=1000;
			while(delay--);
		}
	}
}

void Inverters_Test(void){
	while(1){
		volatile uint64_t delay;
		for(uint64_t i=0;i<=MAX_DUTY_CYCLE;i++){
			Pwm_SetDutyCycle(THROTTLE1_CHANNEL, i);
			Pwm_SetDutyCycle(THROTTLE2_CHANNEL, i);
			Pwm_SetDutyCycle(BRAKE1_CHANNEL, i);
			Pwm_SetDutyCycle(BRAKE2_CHANNEL, i);
			delay=1000;
			while(delay--);
		}
		for(uint64_t i=MAX_DUTY_CYCLE;i>0;i--){
			Pwm_SetDutyCycle(THROTTLE1_CHANNEL, i);
			Pwm_SetDutyCycle(THROTTLE2_CHANNEL, i);
			Pwm_SetDutyCycle(BRAKE1_CHANNEL, i);
			Pwm_SetDutyCycle(BRAKE2_CHANNEL, i);
			delay=1000;
			while(delay--);
		}
		Inverters_SetPower(1);

		Inverters_SetFunction(FORWARD, LEFT_INVERTER, 0);
		Inverters_SetFunction(FORWARD, RIGHT_INVERTER, 0);

		Inverters_SetFunction(REVERSE, LEFT_INVERTER, 0);
		Inverters_SetFunction(REVERSE, RIGHT_INVERTER, 0);

		Inverters_SetFunction(ECO, LEFT_INVERTER, 0);
		Inverters_SetFunction(ECO, RIGHT_INVERTER, 0);

		Inverters_SetFunction(ACCELERATE, LEFT_INVERTER, 0);
		Inverters_SetFunction(ACCELERATE, RIGHT_INVERTER, 0);

		Inverters_SetFunction(BRAKE, LEFT_INVERTER, 0);
		Inverters_SetFunction(BRAKE, RIGHT_INVERTER, 0);

		delay=50000000;
		while(delay--);
		Inverters_SetPower(0);
	}
}

#ifdef __cplusplus
}
#endif



