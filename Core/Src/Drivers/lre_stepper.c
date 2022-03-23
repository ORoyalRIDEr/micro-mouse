#include "Drivers/lre_stepper.h"
#include "main.h"
#include <Lib/printf.h>

#define ENGINE_MAX   100
#define ENGINE_SLOPE 10

enum stepper_side {STEP_LEFT, STEP_RIGHT};

volatile int16_t engine_speed[] = {0,0};     // -1000 ... 1000
volatile int16_t engine_speed_cmd[] = {0,0}; // -1000 ... 1000
volatile int16_t odom_steps[] = {0,0};


void lre_stepper_setStep_side(uint8_t step, enum stepper_side side){
   switch (step)
   {
   case 0:
      if(side == STEP_LEFT) {
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL2_Pin | EngineL3_Pin | EngineL4_Pin, GPIO_PIN_RESET);
      }
      else {
        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR2_Pin | EngineR3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_RESET);
        break;
      }

   case 1:
      if(side == STEP_LEFT) {
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL3_Pin | EngineL4_Pin, GPIO_PIN_RESET);
      }
      else {
        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin | EngineR2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_RESET);
      }
      break; 


   case 2:
      if(side == STEP_LEFT) {
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL3_Pin | EngineL4_Pin, GPIO_PIN_RESET);
      }
      else {
        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin | EngineR3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_RESET);
      }
      break; 

   case 3:
      if(side == STEP_LEFT) {
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL2_Pin | EngineL3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL4_Pin, GPIO_PIN_RESET);
      }
      else {
        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR2_Pin | EngineR3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_RESET);
      }
      break; 

   case 4:
      if(side == STEP_LEFT) {
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL2_Pin | EngineL4_Pin, GPIO_PIN_RESET);
      }
      else {
        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin | EngineR2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_RESET);
      }
      break; 

   case 5:
      if(side == STEP_LEFT) {
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL3_Pin | EngineL4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL2_Pin, GPIO_PIN_RESET);
      }
      else {   
        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin | EngineR2_Pin, GPIO_PIN_RESET);
      }
      break; 

   case 6:
      if(side == STEP_LEFT) {
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL2_Pin | EngineL3_Pin, GPIO_PIN_RESET);
      }
      else {   
        //engine right
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin | EngineR2_Pin | EngineR3_Pin, GPIO_PIN_RESET);
      }
      break; 

   case 7:
      if(side == STEP_LEFT) {
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL2_Pin | EngineL3_Pin, GPIO_PIN_RESET);
      }
      else {   
        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR2_Pin | EngineR3_Pin, GPIO_PIN_RESET);
      }
      break; 

    default:
        break;
    }
}

void rotate(int16_t speed)
{
   int16_t fwd_speed = (engine_speed_cmd[0] + engine_speed_cmd[1]) / 2 / ENGINE_SLOPE;
   int16_t spd0 = fwd_speed+speed;
   int16_t spd1 = fwd_speed-speed;
   engine_speed_cmd[0] = spd0 > 100 ? 100 : (spd0 < -100 ? -100 : spd0);
   engine_speed_cmd[1] = spd1 > 100 ? 100 : (spd1 < -100 ? -100 : spd1);

   engine_speed_cmd[0] *= ENGINE_SLOPE;
   engine_speed_cmd[1] *= ENGINE_SLOPE;

   //cprintf("turn %i\n\r", speed);
} 

void forward(int16_t speed)
{
   engine_speed_cmd[0] = speed * ENGINE_SLOPE;
   engine_speed_cmd[1] = speed * ENGINE_SLOPE;
   //cprintf("fwd %i\n\r", speed);
}

void engine_timer_callback()
{
   static uint16_t step[] = {0, 0};
   static int16_t ticks[] = {0, 0};

   for (uint8_t i=0; i<2; i++) {
      // ramp engine_speed -> engine_speed_cmd
      if (engine_speed[i] < engine_speed_cmd[i])
         engine_speed[i]++;
      else if (engine_speed[i] > engine_speed_cmd[i])
         engine_speed[i] = engine_speed_cmd[i];

      ticks[i] += engine_speed[i] / ENGINE_SLOPE;

      if (ticks[i] >= ENGINE_MAX) {
         ticks[i] -= ENGINE_MAX;

         step[i] += 1;
         odom_steps[i] += 1;
         step[i] = step[i] & 0b111;
         lre_stepper_setStep_side(step[i], i);
      }
      else if (ticks[i] <= -ENGINE_MAX) {
         ticks[i] += ENGINE_MAX;

         step[i] -= 1;
         odom_steps[i] -= 1;
         step[i] = step[i] & 0b111;
         lre_stepper_setStep_side(step[i], i);
      }
   }
}

void get_engine_odometry(int16_t ret_steps[])
{
  ret_steps[0] = odom_steps[0];
  ret_steps[1] = odom_steps[1];

  odom_steps[0] = 0;
  odom_steps[1] = 0;
} 