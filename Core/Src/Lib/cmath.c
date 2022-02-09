#include <Lib/cmath.h>

// lookup table for cosine from 0° ... 90°
const int32_t cos_lut[] = {1000,999,995,988,978,966,951,934,914,891,866,839,809,777,743,707,669,629,588,545,500,454,407,358,309,259,208,156,105,52,0};
const int32_t cos_lut_length = sizeof(cos_lut)/sizeof(cos_lut[0]) - 1;
//const int32_t cos_delta_x = 90 / cos_lut_length;

uint32_t PI1000 = 3142;

int32_t deg2rad1000(int32_t deg)
{
    return (deg*PI1000/180);
}

int32_t rad10002deg(int32_t rad1000)
{
    return (rad1000*180/PI1000);
}

int32_t cosd1000(int32_t deg)
{
    // map x to 0 ... -90°
    if (deg < 0)
        deg = -deg; // cos is symmetric to y-axis
    deg = deg % 360;

    if (deg == 90)
        return 0;

    uint8_t sec34 = (deg >= 180);
    if (sec34)
        return -cos1000(deg-180);
    uint8_t sec2 = (deg >= 90);
    if (sec2)
        return -cos1000(2*90-deg);

    // find argument
    uint8_t index = deg / (90 / cos_lut_length);
    uint8_t rest =  deg % (90 / cos_lut_length);
    int32_t lower = cos_lut[index];
    int32_t upper = cos_lut[index+1];
    int32_t arg = lower + ((upper-lower)*rest)/(90 / cos_lut_length); // linear interpolation between lower and upper boundary

    return arg;
}

int32_t sind1000(int32_t deg)
{
    return cosd1000(deg - 90);
}

int32_t cos1000(int32_t rad1000)
{
    return cosd1000(rad10002deg(rad1000));
}

int32_t sin1000(int32_t rad1000)
{
    return sind1000(rad10002deg(rad1000));
}
