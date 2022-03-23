#ifndef I3G4250D_gyro
#define I3G4250D_gyro

#include <stdint.h>
#include <stm32f0xx_hal.h>

void I3G4250D_gyro_Init (SPI_HandleTypeDef *spi_dev);
void I3G4250D_gyro_Measure (void);

/* get current yaw rate and integral of yaw rate 
@param yaw rate in tbd
@param integrated yaw rate in tbd
*/
void I3G4250D_gyro_GetGyrIntZ (int32_t* r, int32_t* int_r);

void I3G4250D_gyro_Calibrate(void);

void I3G4250D_gyro_SetHeading(int32_t hd);

#endif // I3G4250D_gyro