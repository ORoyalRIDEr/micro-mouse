#include "Drivers/lre_stepper.h"
#include "main.h"

#define ENGINE_MAX   100

enum stepper_side {STEP_LEFT, STEP_RIGHT};
enum direction_t {DIR_BW=-1, DIR_STOP=0, DIR_FW=1};

int16_t engine_speed[] = {0,0}; 


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
   engine_speed[0] = speed;
   engine_speed[1] = -speed;
} 

void forward(int16_t speed)
{
   engine_speed[0] = speed;
   engine_speed[1] = speed;
}

void engine_timer_callback()
{
   static uint16_t step[] = {0, 0};
   static int16_t ticks[] = {0, 0};

   for (uint8_t i=0; i<2; i++) {
      ticks[i] += engine_speed[i];
      if (ticks[i] >= ENGINE_MAX) {
         ticks[i] -= ENGINE_MAX;

         step[i] += 1;
         step[i] = step[i] & 0xF;
         lre_stepper_setStep_side(step[i], i);
      }
      else if (ticks[i] <= -ENGINE_MAX) {
         ticks[i] += ENGINE_MAX;

         step[i] -= 1;
         step[i] = step[i] & 0xF;
         lre_stepper_setStep_side(step[i], i);
      }
   }
}