#ifndef CMATH_H
#define CMATH_H

#include <stdint.h>

/* Converts value in degree to value in radian*1000
(e.g. deg2rad1000(45)  =  785 = 1000*pi/4) */
int32_t deg2rad1000(int32_t deg);
/* Converts value in radian*1000 to degree
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
/* Returns arctan*1000 for angles in radian*1000
CAUTION: x and y must be smaller than 100 to avoid overflow errors
(e.g. atan21000(-78, -95) = -2455 = -140°)) */
int32_t atan21000(int32_t y, int32_t x);


/* Returns absolute value of the value */ 
int32_t absolute(int32_t value);
// PI * 1000
extern int32_t PI1000;

/* executes matrix multiplication with mat and vec;
n_cols and n_rows must match dimension of mat and vec; 
res contains resulting vector */
void mat_vec_mult(int32_t* res, int32_t* mat, int32_t* vec, uint8_t n_cols, uint8_t n_rows);

/* adds two vectors together */
void vec_add(int32_t* res, int32_t* v1, int32_t* v2, uint8_t n_rows);

int32_t median(int32_t vals[], uint8_t n_vals);

uint32_t int_sqrt(uint32_t x);

#endif // CMATH_H