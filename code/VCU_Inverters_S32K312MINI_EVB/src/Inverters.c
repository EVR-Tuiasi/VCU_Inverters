
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
#include "Gpt.h"
#include "Inverters.h"
#include "Messaging.h"

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
#define GPT_200MS_TIMER 12000000U
#define TRESHOLD_LOW_VOLTAGE 800U
#define TRESHOLD_HIGH_VOLTAGE 900U
#define GPT_INVERTER_STATE_TIMER_CHANNEL 0U
#define GPT_INVERTER_CAN_TIMER_CHANNEL 1U
#define MEASUREMENTS_POINTS_NUM 12U
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

volatile bool inverters_state_timer_timeout = 0;
volatile bool inverters_can_timer_timeout = 0;
static InvertersState_t currentState = INVERTERS_OFF;
volatile bool init_flag = 1;

/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/
int32_t adcValues[MEASUREMENTS_POINTS_NUM] = {8192, 7347, 6858, 6502, 6430, 6375, 6144, 5745, 5675, 5388, 4926, 4337};
int32_t tempValues[MEASUREMENTS_POINTS_NUM] = {250, 308, 344, 364, 371, 376, 386, 415, 421, 444, 494, 560};

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
void BrakeLight_SetState(bool value){
	if(value == STD_ON){
		Dio_WriteChannel(BRAKE_LIGHT_PIN, STD_HIGH);
	}
	else{
		Dio_WriteChannel(BRAKE_LIGHT_PIN, STD_LOW);
	}
}
void BrakeLight_Test(void){
	volatile uint64_t i;
	while(1){
		BrakeLight_SetState(1);
		i=1000000;
		while(i--);
		BrakeLight_SetState(0);
		i=1000000;
		while(i--);
	}
}
void Inverters_Init(void){
	volatile uint64_t i;
	Inverters_SetPower(0);
	i=10000000;
	while(i--);
	Pwm_SetDutyCycle(THROTTLE1_CHANNEL, 0U);
	Pwm_SetDutyCycle(THROTTLE2_CHANNEL, 0U);
	Pwm_SetDutyCycle(BRAKE1_CHANNEL, 0U);
	Pwm_SetDutyCycle(BRAKE2_CHANNEL, 0U);
	Adc_SetupResultBuffer(ADC_ACCELERATION_LEFT, &accLeft);
	Adc_SetupResultBuffer(ADC_ACCELERATION_RIGHT, &accRight);
	Dio_WriteChannel(DAC_WAKE_UP_PIN, STD_HIGH);
	Inverters_Forward();
	Inverters_SetFunction(ECO, LEFT_INVERTER, 0);
	Inverters_SetFunction(ECO, RIGHT_INVERTER, 0);
	Inverters_SetFunction(ACCELERATE, LEFT_INVERTER, 1);
	Inverters_SetFunction(ACCELERATE, RIGHT_INVERTER, 1);
	Inverters_SetFunction(BRAKE, LEFT_INVERTER, 0);
	Inverters_SetFunction(BRAKE, RIGHT_INVERTER, 0);
	Gpt_EnableNotification(GPT_INVERTER_STATE_TIMER_CHANNEL);
	Gpt_EnableNotification(GPT_INVERTER_CAN_TIMER_CHANNEL);
	Gpt_StartTimer(GPT_INVERTER_STATE_TIMER_CHANNEL, GPT_200MS_TIMER);
	Gpt_StartTimer(GPT_INVERTER_CAN_TIMER_CHANNEL, GPT_200MS_TIMER);
}
void Inverters_SetThrottle(Inverter_t inverter, uint8_t percentage){
	if(percentage > 100U){
		percentage = 100U;
	}
	if(Inverters_GetState() == INVERTERS_ON){
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
	else{
		Pwm_SetDutyCycle(THROTTLE1_CHANNEL, 0);
		Pwm_SetDutyCycle(THROTTLE2_CHANNEL, 0);
	}
}
void Inverters_SetBrake(Inverter_t inverter, uint8_t percentage){
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
void Cooling_SetFanSpeed(PartNum_t num, uint8_t percentage){
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
void Cooling_SetPumpSpeed(PartNum_t num, uint8_t percentage){
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

void Inverters_SetFunction(Function_t name, Inverter_t inverter, bool value){
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

static int8_t getTempFromAdc(uint32_t adc, int32_t adc_low, int32_t adc_high, int32_t temp_low, int32_t temp_high){
	return ((((((int32_t)adc) - adc_low) * (temp_high - temp_low)) / (adc_high - adc_low)) + temp_low) / 10;
}

int8_t Cooling_ReadTemp(PartNum_t num){
	uint32_t result;
	if(num == ONE){
		Adc_StartGroupConversion(ADC_TEMP_1);
		while(Adc_GetGroupStatus(ADC_TEMP_1) == ADC_BUSY);
		Adc_ReadGroup(ADC_TEMP_1, &temp1);
		result =  temp1;
	}
	else{
		Adc_StartGroupConversion(ADC_TEMP_2);
		while(Adc_GetGroupStatus(ADC_TEMP_2) == ADC_BUSY);
		Adc_ReadGroup(ADC_TEMP_2, &temp2);
		result = temp2;
	}
	if(result > 11703U){
		return 0U;
	}
	for(uint8_t i=1; i < MEASUREMENTS_POINTS_NUM; i++){
		if(result > adcValues[i]){
			return getTempFromAdc(result, adcValues[i-1], adcValues[i], tempValues[i-1], tempValues[i]);
		}
	}
	return getTempFromAdc(result, adcValues[MEASUREMENTS_POINTS_NUM-2], adcValues[MEASUREMENTS_POINTS_NUM-1], tempValues[MEASUREMENTS_POINTS_NUM-2], tempValues[MEASUREMENTS_POINTS_NUM-1]);
}
uint32_t Cooling_ReadPressure(PartNum_t num){
	//0.5V => 0MPa; 4.5V => 1.2MPa
	//functia returneaza in Pa
	uint16_t result;
	if(num == ONE){
		Adc_StartGroupConversion(ADC_PRESSURE_1);
		while(Adc_GetGroupStatus(ADC_PRESSURE_1) == ADC_BUSY);
		Adc_ReadGroup(ADC_PRESSURE_1, &pres1);
		result = pres1;
	}
	else{
		Adc_StartGroupConversion(ADC_PRESSURE_2);
		while(Adc_GetGroupStatus(ADC_PRESSURE_2) == ADC_BUSY);
		Adc_ReadGroup(ADC_PRESSURE_2, &pres2);
		result = pres2;
	}
	if(result < 1683){
		result = 1638;
	}
	if(result > 14744){
		result = 14744;
	}
	return (uint32_t)((((uint64_t)1200000U) * (((uint64_t)result) - ((uint64_t)1638U))) / ((uint64_t)13106U));
}
uint8_t Inverters_ReadAcceleration(Inverter_t inverter){
	uint16_t result;
	if(inverter == LEFT_INVERTER){
		Adc_StartGroupConversion(ADC_ACCELERATION_LEFT);
		while(Adc_GetGroupStatus(ADC_ACCELERATION_LEFT) == ADC_BUSY);
		Adc_ReadGroup(ADC_ACCELERATION_LEFT, &accLeft);
		result = accLeft;
	}
	else{
		Adc_StartGroupConversion(ADC_ACCELERATION_RIGHT);
		while(Adc_GetGroupStatus(ADC_ACCELERATION_RIGHT) == ADC_BUSY);
		Adc_ReadGroup(ADC_ACCELERATION_RIGHT, &accRight);
		result = accRight;
	}
	return (uint8_t)(((uint32_t)result) * ((uint32_t)250U)) / ((uint32_t)16383U);
}
void Cooling_Test(void){
	while(1){
		volatile uint64_t delay;
		for(uint64_t i=0;i<=MAX_DUTY_CYCLE;i++){
			Pwm_SetDutyCycle(FAN1_CHANNEL, i);
			Pwm_SetDutyCycle(FAN2_CHANNEL, i);
			//Pwm_SetDutyCycle(PUMP1_CHANNEL, i);
			//Pwm_SetDutyCycle(PUMP2_CHANNEL, i);
			delay=1000;
			while(delay--);
		}
		for(uint64_t i=MAX_DUTY_CYCLE;i>0;i--){
			Pwm_SetDutyCycle(FAN1_CHANNEL, i);
			Pwm_SetDutyCycle(FAN2_CHANNEL, i);
			//Pwm_SetDutyCycle(PUMP1_CHANNEL, i);
			//Pwm_SetDutyCycle(PUMP2_CHANNEL, i);
			delay=1000;
			while(delay--);
		}
	}
}

void Inverters_Test(void){
	volatile uint64_t delay;
	while(1){
		for(uint64_t i=0;i<=100/2;i++){
			Inverters_Update();
			Inverters_SetThrottle(LEFT_INVERTER, i);
			Inverters_SetThrottle(RIGHT_INVERTER, i);
			//Pwm_SetDutyCycle(BRAKE1_CHANNEL, i);
			//Pwm_SetDutyCycle(BRAKE2_CHANNEL, i);
			delay=1000000;
			while(delay--);
		}
		for(uint64_t i=100/2;i>0;i--){
			Inverters_Update();
			Inverters_SetThrottle(LEFT_INVERTER, i);
			Inverters_SetThrottle(RIGHT_INVERTER, i);
			//Pwm_SetDutyCycle(BRAKE1_CHANNEL, i);
			//Pwm_SetDutyCycle(BRAKE2_CHANNEL, i);
			delay=1000000;
			while(delay--);
		}
	}
}

void Inverters_Update(void){
	switch(currentState){
		case INVERTERS_OFF:
			if(inverters_state_timer_timeout == 1){
				Inverters_SetPower(1);
				Inverters_ResetTimer();
				Inverters_ResetCanTimer();
				currentState = INVERTERS_WAITING_FOR_VOLTAGE;
			}
			break;
		case INVERTERS_WAITING_FOR_VOLTAGE:
			if(inverters_can_timer_timeout == 1){
				currentState = INVERTERS_OFF;
			}
			else if(MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage.valueCan > TRESHOLD_HIGH_VOLTAGE){
				Inverters_SetPower(0);
				Inverters_ResetTimer();
				currentState = INVERTERS_STARTING;
			}
			break;
		case INVERTERS_STARTING:
			if(inverters_state_timer_timeout == 1){
				Inverters_SetPower(1);
				Inverters_ResetTimer();
				Inverters_ResetCanTimer();
				currentState = INVERTERS_DELAY;
			}
			break;
		case INVERTERS_DELAY:
			if(inverters_state_timer_timeout == 1){
				Inverters_ResetTimer();
				Inverters_ResetCanTimer();
				currentState = INVERTERS_ON;
			}
			break;
		case INVERTERS_ON:
			if((MonitoredValues.InvertersMonitoredValues.LeftInverterInputVoltage.valueCan < TRESHOLD_LOW_VOLTAGE) || (inverters_can_timer_timeout == 1)){
				Inverters_SetPower(0);
				Inverters_ResetTimer();
				currentState = INVERTERS_OFF;
			}
			break;
	}
}

InvertersState_t Inverters_GetState(void){
	return currentState;
}

void Inverters_ResetTimer(void){
	inverters_state_timer_timeout = 0;
	Gpt_StopTimer(GPT_INVERTER_STATE_TIMER_CHANNEL);
	Gpt_StartTimer(GPT_INVERTER_STATE_TIMER_CHANNEL, GPT_200MS_TIMER);
}

void Inverters_ResetCanTimer(void){
	inverters_can_timer_timeout = 0;
	Gpt_StopTimer(GPT_INVERTER_CAN_TIMER_CHANNEL);
	Gpt_StartTimer(GPT_INVERTER_CAN_TIMER_CHANNEL, GPT_200MS_TIMER);
}

void Inverters_Timer_Timeout(void){
	inverters_state_timer_timeout = 1;
}

void CanMessaging_Inverters_Timeout(void){
	inverters_can_timer_timeout = 1;
}

void Inverters_Shutdown(void){
	Dio_WriteChannel(DAC_WAKE_UP_PIN, STD_LOW);

	Inverters_SetThrottle(LEFT_INVERTER, 0U);
	Inverters_SetThrottle(RIGHT_INVERTER, 0U);
	Inverters_SetBrake(LEFT_INVERTER, 0U);
	Inverters_SetBrake(RIGHT_INVERTER, 0U);

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
}

void Inverters_SetDirection(InvertersDirection_t direction){
	Dio_WriteChannel(DAC_WAKE_UP_PIN, STD_HIGH);

	if(direction == INVERTERS_DIRECTION_REVERSE){
		Inverters_Reverse();
	}
	else{
		Inverters_Forward();
	}
	Inverters_SetFunction(ECO, LEFT_INVERTER, 0);
	Inverters_SetFunction(ECO, RIGHT_INVERTER, 0);
	Inverters_SetFunction(ACCELERATE, LEFT_INVERTER, 1);
	Inverters_SetFunction(ACCELERATE, RIGHT_INVERTER, 1);
	Inverters_SetFunction(BRAKE, LEFT_INVERTER, 0);
	Inverters_SetFunction(BRAKE, RIGHT_INVERTER, 0);
}

void Inverters_Forward(void){
	Inverters_SetFunction(FORWARD, LEFT_INVERTER, 1);
	Inverters_SetFunction(FORWARD, RIGHT_INVERTER, 0);
	Inverters_SetFunction(REVERSE, LEFT_INVERTER, 0);
	Inverters_SetFunction(REVERSE, RIGHT_INVERTER, 1);
}
void Inverters_Reverse(void){
	Inverters_SetFunction(FORWARD, LEFT_INVERTER, 0);
	Inverters_SetFunction(FORWARD, RIGHT_INVERTER, 1);
	Inverters_SetFunction(REVERSE, LEFT_INVERTER, 1);
	Inverters_SetFunction(REVERSE, RIGHT_INVERTER, 0);
}

#ifdef __cplusplus
}
#endif



