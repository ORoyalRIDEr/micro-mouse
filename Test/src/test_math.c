#include <stdio.h>
#include "../../Core/Inc/Lib/cmath.h"

void print_matrix(int32_t* mat, uint8_t n_cols, uint8_t n_rows)
{
    printf("{");
    for (uint8_t row=0; row<n_rows; row++) {
        printf("{ ");
            for (uint8_t col=0; col<n_cols; col++)
                printf("%i ", mat[row*n_cols+col]);
        printf("}");
    }
    printf("}");
}

void test_math_vec_mult_scalar()
{
    int32_t res;
    int32_t exp = 15;
    int32_t mat[][1] = {{3}};
    int32_t v[] = {5};
    mat_vec_mult(&res, mat, v, 1, 1);
    printf("Expected %i, got %i\n", exp, res);
}

void test_math_vec_mult_1d()
{
    int32_t res;
    int32_t exp = -9;
    int32_t mat[1][2] = {{3, -4}};
    int32_t v[2][1] = {{5}, {6}};
    mat_vec_mult(&res, mat, v, 2, 1);
    printf("Expected %d, got %d\n", exp, res);
}

void test_math_vec_mult_2d()
{
    int32_t res[3][1];
    int32_t exp[3][1] = {{-3324}, {1466}, {1012}};
    int32_t mat[3][2] = {{56, -78}, {-91, 11}, {12, 34}};
    int32_t v[2][1] = {{-12}, {34}};
    mat_vec_mult(&res, mat, v, 2, 3);
    
    printf("Expected ");
    print_matrix(exp, 1, 3);
    printf(", got ");
    print_matrix(res, 1, 3);
    printf("\n");
}

void test_math()
{
    test_math_vec_mult_scalar();
    test_math_vec_mult_1d();
    test_math_vec_mult_2d();
}