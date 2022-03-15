#include <Drivers/I3G4250D_gyro.h>
#include <Lib/cmath.h>
#include <ctrl_stack.h>
#include <main.h>

#define I3G4250D_READ_WRITE_OFF 7 // 0: write, 1:read

#define I3G4250D_ADDR_CTRL_REG1 0x20 // Output data rate, Bandwith, power and axis enabling
#define I3G4250D_OFF_CTRL_REG1_Xen 0 // 0: X disabled, 1: X enabled
#define I3G4250D_OFF_CTRL_REG1_Yen 1 // 0: Y disabled, 1: Y enabled
#define I3G4250D_OFF_CTRL_REG1_Zen 2 // 0: Z disabled, 1: Z enabled
#define I3G4250D_OFF_CTRL_REG1_PD 3  // 0: power down; 1: normal mode or sleep mode

#define I3G4250D_ADDR_TMP 0x26
#define I3G4250D_ADDR_X_L 0x28
#define I3G4250D_ADDR_X_H 0x29
#define I3G4250D_ADDR_Y_L 0x2A
#define I3G4250D_ADDR_Y_H 0x2B
#define I3G4250D_ADDR_Z_L 0x2C
#define I3G4250D_ADDR_Z_H 0x2D

#define I3G4250D_scale 250             // degree per second
#define I3G4250D_scale_calib 360 / 310 // scaling factor through calibration

#define I3G4250D_CALIBRATION_SAMPLES 100 // Number of samples used for calibration

SPI_HandleTypeDef *I3G4250D_spi_dev;
int32_t yaw_rate = 0;
int32_t int_yaw_rate = 0;
int16_t bias_raw = 0;

uint8_t I3G4250D_gyro_read(uint8_t addr)
{
    addr |= 1 << I3G4250D_READ_WRITE_OFF;
    uint16_t msg = ((uint16_t)addr) << 8;
    uint16_t tmp_ret;

    HAL_GPIO_WritePin(NCS_MEMS_SPI_GPIO_Port, NCS_MEMS_SPI_Pin, GPIO_PIN_RESET); // Slave select
    HAL_SPI_TransmitReceive(I3G4250D_spi_dev, (uint8_t *)&msg, (uint8_t *)&tmp_ret, 1, 100);
    HAL_GPIO_WritePin(NCS_MEMS_SPI_GPIO_Port, NCS_MEMS_SPI_Pin, GPIO_PIN_SET); // Slave select

    return (uint8_t)(tmp_ret & 0xFF);
}

void I3G4250D_gyro_write(uint8_t addr, uint8_t data)
{
    addr &= ~(1 << I3G4250D_READ_WRITE_OFF);
    uint16_t msg = addr;
    msg = msg << 8;
    msg |= data;

    HAL_GPIO_WritePin(NCS_MEMS_SPI_GPIO_Port, NCS_MEMS_SPI_Pin, GPIO_PIN_RESET); // Slave select
    HAL_SPI_Transmit(I3G4250D_spi_dev, (uint8_t *)&msg, 1, 100);
    HAL_GPIO_WritePin(NCS_MEMS_SPI_GPIO_Port, NCS_MEMS_SPI_Pin, GPIO_PIN_SET); // Slave select
}

void I3G4250D_gyro_Init(SPI_HandleTypeDef *spi_dev)
{
    I3G4250D_spi_dev = spi_dev;

    uint32_t ctrl_reg1 =
        (1 << I3G4250D_OFF_CTRL_REG1_PD) |
        (1 << I3G4250D_OFF_CTRL_REG1_Xen) |
        (1 << I3G4250D_OFF_CTRL_REG1_Yen) |
        (1 << I3G4250D_OFF_CTRL_REG1_Zen);
    I3G4250D_gyro_write(I3G4250D_ADDR_CTRL_REG1, (uint8_t)ctrl_reg1);
    HAL_GPIO_WritePin(NCS_MEMS_SPI_GPIO_Port, NCS_MEMS_SPI_Pin, GPIO_PIN_SET); // Slave select
}

int16_t I3G4250D_gyro_Measure_raw()
{
    uint8_t zl, zh; // z measurement low and high
    zl = I3G4250D_gyro_read(I3G4250D_ADDR_Z_L);
    zh = I3G4250D_gyro_read(I3G4250D_ADDR_Z_H);

    int16_t raw = (zh << 8) | zl;
    return raw;
}

void I3G4250D_gyro_Measure()
{
    int32_t raw = (int32_t) I3G4250D_gyro_Measure_raw();
    raw -= bias_raw;
    yaw_rate = -raw * deg2rad1000(I3G4250D_scale) / 0x7FFF * 1000 * I3G4250D_scale_calib;

    int_yaw_rate += yaw_rate / GYRO_FREQ;
}

void I3G4250D_gyro_GetGyrIntZ(int32_t *r, int32_t *int_r)
{
    *r = yaw_rate;
    *int_r = int_yaw_rate;
}

void I3G4250D_gyro_Calibrate()
{
    // bias estimation
    int32_t bias_sum = 0;

    for (uint32_t i=0; i<I3G4250D_CALIBRATION_SAMPLES; i++)
    {
        bias_sum += I3G4250D_gyro_Measure_raw();
        HAL_Delay(10);
    }

    bias_raw = bias_sum / I3G4250D_CALIBRATION_SAMPLES;
    int_yaw_rate = 0;
}