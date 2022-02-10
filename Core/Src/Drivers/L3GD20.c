#include <Drivers/L3GD20.h>
#include <main.h>


// Iclude SPI handler
extern SPI_HandleTypeDef hspi2;



void L3GD20_Init (uint16_t Data)
{
    // Set Powermode
    
    // Set Bandwith
    
    // Set Scale
    
    // Set endianess
    
    // Set Highpassfilter
}
void L3GD20_IO (uint16_t Data)
{
    //Write Config
    // set chip select
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    // transmit register
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&DatA, 2, 100);
    //unset chip select
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

}
uint16_t L3GD20_Read (void)
{

uint16_t gyro_z =0;

    return gyro_z;
}
