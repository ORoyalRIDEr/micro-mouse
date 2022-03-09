#include <Lib/cmath.h>

// lookup table for cosine from 0° ... 90°
const int32_t cos_lut[] = {1000,999,995,988,978,966,951,934,914,891,866,839,809,777,743,707,669,629,588,545,500,454,407,358,309,259,208,156,105,52,0};
const int32_t cos_lut_length = sizeof(cos_lut)/sizeof(cos_lut[0]) - 1;

int32_t PI1000 = 3142;

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
    const int32_t cos_delta_x = 90 / cos_lut_length;
    
    // map x to 0 ... -90°
    if (deg < 0)
        deg = -deg; // cos is symmetric to y-axis
    deg = deg % 360;

    if (deg == 90)
        return 0;

    uint8_t sec34 = (deg >= 180);
    if (sec34)
        return -cosd1000(deg-180);
    uint8_t sec2 = (deg >= 90);
    if (sec2)
        return -cosd1000(2*90-deg);

    // find argument
    uint8_t index = deg / cos_delta_x;
    uint8_t rest =  deg % cos_delta_x;
    int32_t lower = cos_lut[index];
    int32_t upper = cos_lut[index+1];
    int32_t arg = lower + ((upper-lower)*rest)/cos_delta_x; // linear interpolation between lower and upper boundary

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

int32_t absolute(int32_t value){
    if (value >= 0){
        return value;
    }
    else{
        return - value;
    }
} 


void mat_vec_mult(int32_t* res, int32_t* mat, int32_t* vec, uint8_t n_cols, uint8_t n_rows)
{
    for (uint8_t row=0; row<n_rows; row++) {
        res[row] = 0;
        for (uint8_t col=0; col<n_cols; col++)
            res[row] += vec[col] * mat[row*n_cols+col];
    }
}

int32_t median(int32_t vals[], uint8_t n_vals)
{
    int32_t buf[n_vals], temp;
    int8_t i, j;
    
    for (i=0; i<n_vals; i++)
        buf[i] = vals[i];

    for (i=0; i<n_vals-1; i++) {
        for(j=0; j<n_vals-i-1; j++) {
            if(buf[j]<buf[j+1]) {                                                                           
                temp = buf[j];
                buf[j] = buf[j+1];
                buf[j+1] = temp;
            }
        }
    }

    return buf[n_vals/2];
}
