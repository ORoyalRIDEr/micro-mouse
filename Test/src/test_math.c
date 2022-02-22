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

void test_cos()
{
    int32_t threshold = 20;

    int32_t x[] = {-17453,-17279,-17104,-16930,-16755,-16581,-16406,-16232,-16057,-15882,-15708,-15533,-15359,-15184,-15010,-14835,-14661,-14486,-14312,-14137,-13963,-13788,-13614,-13439,-13265,-13090,-12915,-12741,-12566,-12392,-12217,-12043,-11868,-11694,-11519,-11345,-11170,-10996,-10821,-10647,-10472,-10297,-10123,-9948,-9774,-9599,-9425,-9250,-9076,-8901,-8727,-8552,-8378,-8203,-8029,-7854,-7679,-7505,-7330,-7156,-6981,-6807,-6632,-6458,-6283,-6109,-5934,-5760,-5585,-5411,-5236,-5061,-4887,-4712,-4538,-4363,-4189,-4014,-3840,-3665,-3491,-3316,-3142,-2967,-2793,-2618,-2443,-2269,-2094,-1920,-1745,-1571,-1396,-1222,-1047,-873,-698,-524,-349,-175,0,175,349,524,698,873,1047,1222,1396,1571,1745,1920,2094,2269,2443,2618,2793,2967,3142,3316,3491,3665,3840,4014,4189,4363,4538,4712,4887,5061,5236,5411,5585,5760,5934,6109,6283,6458,6632,6807,6981,7156,7330,7505,7679,7854,8029,8203,8378,8552,8727,8901,9076,9250,9425,9599,9774,9948,10123,10297,10472,10647,10821,10996,11170,11345,11519,11694,11868,12043,12217,12392,12566,12741,12915,13090,13265,13439,13614,13788,13963,14137,14312,14486,14661,14835,15010,15184,15359,15533,15708,15882,16057,16232,16406,16581,16755,16930,17104,17279,17453};
    int32_t y[] = {173,0,-174,-342,-500,-643,-766,-866,-940,-985,-1000,-985,-940,-866,-766,-643,-500,-342,-174,0,173,342,500,643,766,866,940,985,1000,985,940,866,766,643,500,342,174,0,-174,-342,-500,-643,-766,-866,-940,-985,-1000,-985,-940,-866,-766,-643,-500,-342,-174,0,174,342,500,643,766,866,940,985,1000,985,940,866,766,643,500,342,174,0,-174,-342,-500,-643,-766,-866,-940,-985,-1000,-985,-940,-866,-766,-643,-500,-342,-173,0,174,342,500,643,766,866,940,985,1000,985,940,866,766,643,500,342,174,0,-173,-342,-500,-643,-766,-866,-940,-985,-1000,-985,-940,-866,-766,-643,-500,-342,-174,0,174,342,500,643,766,866,940,985,1000,985,940,866,766,643,500,342,174,0,-174,-342,-500,-643,-766,-866,-940,-985,-1000,-985,-940,-866,-766,-643,-500,-342,-174,0,174,342,500,643,766,866,940,985,1000,985,940,866,766,643,500,342,173,0,-174,-342,-500,-643,-766,-866,-940,-985,-1000,-985,-940,-866,-766,-643,-500,-342,-174,0,173};
    int32_t n_vals = sizeof(x)/sizeof(x[0]);
    printf("Test cosine\n");
    for (int32_t i=0; i<n_vals; i++) {
        int32_t act = cos1000(x[i]);
        int32_t diff = act - y[i];
        if (diff > threshold || diff < -threshold)
            printf("%i/%i: in: %i, out: %i, exp: %i\n", i, n_vals, x[i], act, y[i]);
    }
    printf("done\n");
}

void test_math()
{
    /*test_math_vec_mult_scalar();
    test_math_vec_mult_1d();
    test_math_vec_mult_2d();*/
    test_cos();
}