#include <Drivers/HCSR04.h>
#include <main.h>
#include <Lib/printf.h>
#include <Lib/cmath.h>

/****************************************************************************************************************************
 * HCRS04 Sensor Driver
 ***************************************************************************************************************************/
#define MEDIAN_BUF_SIZE 1

int32_t Differences[4][MEDIAN_BUF_SIZE];
uint8_t Diff_i[] = {0, 0, 0, 0}; // index of the Differences buffer
int32_t Is_First_Captured[] = {0, 0, 0, 0}; // is the first value captured ?

// Global variables for front sensor
uint32_t IC_Val1_FRONT = 0;
uint32_t IC_Val2_FRONT = 0;

// Global variables for right sensor
uint32_t IC_Val1_RIGHT = 0;
uint32_t IC_Val2_RIGHT = 0;

// Global variables for left sensor
uint32_t IC_Val1_LEFT = 0;
uint32_t IC_Val2_LEFT = 0;

// Global variables for back sensor
uint32_t IC_Val1_BACK = 0;
uint32_t IC_Val2_BACK = 0;

TIM_HandleTypeDef* htim;

void delay (uint16_t time)
{
	uint32_t start = __HAL_TIM_GET_COUNTER(htim);
	while ((__HAL_TIM_GET_COUNTER(htim) - start) < time);
}

void HCSR04_Measure_direction (uint8_t direction)
{
	Is_First_Captured[direction] = 0;

	__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4);

	uint32_t trig_pin[] = {TRIG_FRONT_Pin, 0, TRIG_LEFT_Pin, TRIG_RIGHT_Pin};
	GPIO_TypeDef *trig_port[] = {TRIG_FRONT_GPIO_Port, 0, TRIG_LEFT_GPIO_Port, TRIG_RIGHT_GPIO_Port};
	uint32_t echo_chan[] = {TIM_IT_CC3, TIM_IT_CC2, TIM_IT_CC1, TIM_IT_CC4};

	__HAL_TIM_SET_CAPTUREPOLARITY(htim, echo_chan[direction], TIM_INPUTCHANNELPOLARITY_RISING);
	__HAL_TIM_ENABLE_IT(htim, echo_chan[direction]);

	HAL_GPIO_WritePin(trig_port[direction], trig_pin[direction], GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(trig_port[direction], trig_pin[direction], GPIO_PIN_RESET);  // pull the TRIG pin low
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *hcsr04_timer)

{
	//Sensor_LEFT
	if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured[DIST_LEFT]==0) // if the first value is not captured
		{
			IC_Val1_LEFT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured[DIST_LEFT] = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else   // if the first is already captured
		{
			IC_Val2_LEFT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

			if (IC_Val2_LEFT > IC_Val1_LEFT) {
				Differences[DIST_LEFT][Diff_i[DIST_LEFT]] = IC_Val2_LEFT-IC_Val1_LEFT;
				Diff_i[DIST_LEFT] = (Diff_i[DIST_LEFT]+1) % MEDIAN_BUF_SIZE;
			}
			
			Is_First_Captured[DIST_LEFT] = 0; // set it back to false
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
			HCSR04_Measure_direction(DIST_FRONT);
		}
	}
	//Sensor_BACK
	else if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel2
	{
		if (Is_First_Captured[DIST_BACK]==0) // if the first value is not captured
		{
			IC_Val1_BACK = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
			Is_First_Captured[DIST_BACK] = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else  // if the first is already captured
		{
			IC_Val2_BACK = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value  

			if (IC_Val2_BACK > IC_Val1_BACK) {
				Differences[DIST_BACK][Diff_i[DIST_BACK]] = IC_Val2_BACK-IC_Val1_BACK;
				Diff_i[DIST_BACK] = (Diff_i[DIST_BACK]+1) % MEDIAN_BUF_SIZE;
			}

			Is_First_Captured[DIST_BACK] = 0; // set it back to false
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
		}
	}
	//Sensor_Front
	else if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_3)  // if the interrupt source is channel3
	{
		if (Is_First_Captured[DIST_FRONT]==0) // if the first value is not captured
		{
			IC_Val1_FRONT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
			Is_First_Captured[DIST_FRONT] = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else  // if the first is already captured
		{
			IC_Val2_FRONT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value 

			if (IC_Val2_FRONT > IC_Val1_FRONT) {
				Differences[DIST_FRONT][Diff_i[DIST_FRONT]] = IC_Val2_FRONT-IC_Val1_FRONT;
				Diff_i[DIST_FRONT] = (Diff_i[DIST_FRONT]+1) % MEDIAN_BUF_SIZE;
			}

			Is_First_Captured[DIST_FRONT] = 0; // set it back to false
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
			HCSR04_Measure_direction(DIST_RIGHT);
		}
	}
	//Sensor_RIGHT
	else if (htim->Channel & HAL_TIM_ACTIVE_CHANNEL_4)  // if the interrupt source is channel2
	{
		if (Is_First_Captured[DIST_RIGHT]==0) // if the first value is not captured
		{
			IC_Val1_RIGHT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
			Is_First_Captured[DIST_RIGHT] = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else   // if the first is already captured
		{
			IC_Val2_RIGHT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value

			if (IC_Val2_RIGHT > IC_Val1_RIGHT) {
				Differences[DIST_RIGHT][Diff_i[DIST_RIGHT]] = IC_Val2_RIGHT-IC_Val1_RIGHT;
				Diff_i[DIST_RIGHT] = (Diff_i[DIST_RIGHT]+1) % MEDIAN_BUF_SIZE;
			}

			Is_First_Captured[DIST_RIGHT] = 0; // set it back to false
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);
		}
	}

}

void HCSR04_Init (TIM_HandleTypeDef *hcsr04_timer)
{
	htim = hcsr04_timer;
}

void HCSR04_Measure()
{
	HCSR04_Measure_direction(DIST_LEFT);
}

void HCSR04_Read (int32_t distances[])
{
	for (uint8_t i=0; i<4; i++) {
		int32_t travel_time = Differences[i][0];
		distances[i] = travel_time * 343/2; // Distance in µm
	}
}

