#include "lre_stepper.h"
#include "main.h"

void lre_stepper_setStep(uint8_t step){
    switch (step)
    {
    case 0:
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL2_Pin | EngineL3_Pin | EngineL4_Pin, GPIO_PIN_RESET);

        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR2_Pin | EngineR3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_RESET);
        break;

     case 1:
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL3_Pin | EngineL4_Pin, GPIO_PIN_RESET);

        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin | EngineR2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_RESET);
        break; 

     case 2:
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL3_Pin | EngineL4_Pin, GPIO_PIN_RESET);

        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin | EngineR3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_RESET);
        break;

     case 3:
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL2_Pin | EngineL3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL4_Pin, GPIO_PIN_RESET);

        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR2_Pin | EngineR3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_RESET);
        break; 

    case 4:
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL2_Pin | EngineL4_Pin, GPIO_PIN_RESET);

        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin | EngineR2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_RESET);
        break;

     case 5:
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL3_Pin | EngineL4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL2_Pin, GPIO_PIN_RESET);

        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin | EngineR2_Pin, GPIO_PIN_RESET);
        
        break; 

     case 6:
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL2_Pin | EngineL3_Pin, GPIO_PIN_RESET);

        //engine right
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin | EngineR2_Pin | EngineR3_Pin, GPIO_PIN_RESET);
        break;

     case 7:
        //engine left
        HAL_GPIO_WritePin(GPIOB, EngineL1_Pin | EngineL4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, EngineL2_Pin | EngineL3_Pin, GPIO_PIN_RESET);

        //engine right
        HAL_GPIO_WritePin(GPIOC, EngineR1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, EngineR4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, EngineR2_Pin | EngineR3_Pin, GPIO_PIN_RESET);
        break;

    default:
        break;
    }
}

void rotate(uint32_t times){
   int i = 0;
   int rotations = 0;
   while(rotations < times){
      lre_stepper_setStep_side(i, 0);
      lre_stepper_setStep_side(7-i, 1); 
      i++;
      if (i > 7) {
         i = 0;
         rotations++;
      }
      HAL_Delay(1);
   }
}  

void forward(uint32_t times){
   int i = 0;
   int rotations = 0;
   while(rotations < times){
      lre_stepper_setStep_side(i, 0);
      lre_stepper_setStep_side(i, 1); 
      i++;
      if (i > 7) {
         i = 0;
         rotations++;
      }
      HAL_Delay(1);
   }
}  


void lre_stepper_setStep_side(uint8_t step, uint8_t right){
   switch (step)
   {
   case 0:
      if(!right) {
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
      if(!right) {
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
      if(!right) {
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
      if(!right) {
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
      if(!right) {
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
      if(!right) {
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
      if(!right) {
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
      if(!right) {
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