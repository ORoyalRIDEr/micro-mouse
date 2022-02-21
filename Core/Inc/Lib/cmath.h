#ifndef CMATH_H
#define CMATH_H

#include <stdint.h>

/* Converts value in radian*1000 to degree
(e.g. deg2rad1000(45)  =  785 = 1000*pi/4) */
int32_t deg2rad1000(int32_t deg);
/* Converts value in degree to value in radian*1000
(e.g. rad10002deg(1000*pi/4 = 785)  =  45) */
int32_t rad10002deg(int32_t rad1000);

/* Returns cosine*1000 for angle in degree
(e.g. cosd1000(180) = -1000) */
int32_t cosd1000(int32_t deg);
/* Returns sine*1000 for angle in degree
(e.g. sind1000(90) = 1000) */
int32_t sind1000(int32_t deg);
/* Returns cosine*1000 for angle in radian*1000
(e.g. cos1000(1000*pi) = -1000) */
int32_t cos1000(int32_t rad1000);
/* Returns sine*1000 for angle in radian*1000
(e.g. sin1000(1000*pi/2) = 1000) */
int32_t sin1000(int32_t rad1000);
/* Returns absolute value of the value */ 
int32_t absolute(int32_t value);
// PI * 1000
extern uint32_t PI1000;

#endif // CMATH_H