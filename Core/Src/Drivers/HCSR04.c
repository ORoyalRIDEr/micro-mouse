#include <Drivers/HCSR04.h>
#include <main.h>

/****************************************************************************************************************************
 * HCRS04 Sensor Driver
 ***************************************************************************************************************************/
// Global variables for front sensor
uint32_t IC_Val1_FRONT = 0;
uint32_t IC_Val2_FRONT = 0;
uint32_t Difference_FRONT = 0;
uint8_t Is_First_Captured_FRONT = 0;  // is the first value captured ?
int32_t Distance_FRONT  = 0;

// Global variables for right sensor
uint32_t IC_Val1_RIGHT = 0;
uint32_t IC_Val2_RIGHT = 0;
uint32_t Difference_RIGHT = 0;
uint8_t Is_First_Captured_RIGHT = 0;  // is the first value captured ?
int32_t Distance_RIGHT  = 0;

// Global variables for left sensor
uint32_t IC_Val1_LEFT = 0;
uint32_t IC_Val2_LEFT = 0;
uint32_t Difference_LEFT = 0;
uint8_t Is_First_Captured_LEFT = 0;  // is the first value captured ?
int32_t Distance_LEFT  = 0;

// Global variables for back sensor
uint32_t IC_Val1_BACK = 0;
uint32_t IC_Val2_BACK = 0;
uint32_t Difference_BACK = 0;
uint8_t Is_First_Captured_BACK = 0;  // is the first value captured ?
int32_t Distance_BACK  = 0;

TIM_HandleTypeDef* htim;
uint32_t n_measurements[4]; 

// Let's write the callback function

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *hcsr04_timer)
{
	//Sensor_LEFT
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured_LEFT==0) // if the first value is not captured
		{
			IC_Val1_LEFT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured_LEFT = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured_LEFT==1)   // if the first is already captured
		{
			IC_Val2_LEFT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2_LEFT > IC_Val1_LEFT)
			{
				Difference_LEFT = IC_Val2_LEFT-IC_Val1_LEFT;
			}

			else if (IC_Val1_LEFT > IC_Val2_LEFT)
			{
				Difference_LEFT = (0xffff - IC_Val1_LEFT) + IC_Val2_LEFT;
			}

			Distance_LEFT = Difference_LEFT * 343.2/2; // Distance in µm
			Is_First_Captured_LEFT = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
			n_measurements[DIST_LEFT] += 1;
		}
	}
	//Sensor_BACK
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel2
	{
		if (Is_First_Captured_BACK==0) // if the first value is not captured
		{
			IC_Val1_BACK = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
			Is_First_Captured_BACK = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured_BACK==1)   // if the first is already captured
		{
			IC_Val2_BACK = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2_BACK > IC_Val1_BACK)
			{
				Difference_BACK = IC_Val2_BACK-IC_Val1_BACK;
			}

			else if (IC_Val1_BACK > IC_Val2_BACK)
			{
				Difference_BACK = (0xffff - IC_Val1_BACK) + IC_Val2_BACK;
			}

			Distance_BACK = Difference_BACK * 343.2/2;
			Is_First_Captured_BACK = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC2);
			n_measurements[DIST_BACK] += 1;
		}
	}
	//Sensor_RIGHT
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  // if the interrupt source is channel3
	{
		if (Is_First_Captured_FRONT==0) // if the first value is not captured
		{
			IC_Val1_FRONT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
			Is_First_Captured_FRONT = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured_FRONT==1)   // if the first is already captured
		{
			IC_Val2_FRONT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2_FRONT > IC_Val1_FRONT)
			{
				Difference_FRONT = IC_Val2_FRONT-IC_Val1_FRONT;
			}

			else if (IC_Val1_FRONT > IC_Val2_FRONT)
			{
				Difference_FRONT = (0xffff - IC_Val1_FRONT) + IC_Val2_FRONT;
			}

			Distance_FRONT = Difference_FRONT * 343.2/2;
			Is_First_Captured_FRONT = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC3);
			n_measurements[DIST_FRONT] += 1;
		}
	}
	//Sensor_RIGHT
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)  // if the interrupt source is channel2
	{
		if (Is_First_Captured_RIGHT==0) // if the first value is not captured
		{
			IC_Val1_RIGHT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
			Is_First_Captured_RIGHT = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured_RIGHT==1)   // if the first is already captured
		{
			IC_Val2_RIGHT = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2_RIGHT > IC_Val1_RIGHT)
			{
				Difference_RIGHT = IC_Val2_RIGHT-IC_Val1_RIGHT;
			}

			else if (IC_Val1_RIGHT > IC_Val2_RIGHT)
			{
				Difference_RIGHT = (0xffff - IC_Val1_RIGHT) + IC_Val2_RIGHT;
			}

			Distance_RIGHT = Difference_RIGHT * 343.2/2;
			Is_First_Captured_RIGHT = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC4);
			n_measurements[DIST_RIGHT] += 1;
		}
	}

}

void HCSR04_Init (TIM_HandleTypeDef *hcsr04_timer)
{
	htim = hcsr04_timer;
}

void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(htim, 0);
	while (__HAL_TIM_GET_COUNTER (htim) < time);
}

void HCSR04_Measure (void)
{
	Distance_FRONT = 0;
	Distance_RIGHT = 0;
	Distance_BACK = 0;
	Distance_LEFT = 0;

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	// activate interrupts for all 4 channels of timer 2
	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC2);
	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC3);
	__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC4);
}

void HCSR04_Read (int32_t distances[])
{
    distances[DIST_FRONT] = (Distance_FRONT <= MAX_DIST) ? Distance_FRONT : 0;
    distances[DIST_RIGHT] = (Distance_RIGHT <= MAX_DIST) ? Distance_RIGHT : 0;
    distances[DIST_BACK] = (Distance_BACK <= MAX_DIST) ? Distance_BACK : 0;
    distances[DIST_LEFT] = (Distance_LEFT <= MAX_DIST) ? Distance_LEFT : 0;
}

