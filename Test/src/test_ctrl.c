#include <stdio.h>
#include "../../Core/Inc/Lib/cmath.h"
#include "../../Core/Inc/Ecl/state_estimator.h"
#include "../../Core/Inc/Ecl/orientation_ctrl.h"

extern int32_t orient_setpoint;

void test_orientation_ctrl()
{
    int32_t threshold = 0;

    int32_t x[] = {-180,-175,-170,-165,-160,-155,-150,-145,-140,-135,-130,-125,-120,-115,-110,-105,-100,-95,-90,-85,-80,-75,-70,-65,-60,-55,-50,-45,-40,-35,-30,-25,-20,-15,-10,-5,0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105,110,115,120,125,130,135,140,145,150,155,160,165,170,175,180};
    //int32_t exp[] = {2,4,10,67,11111};
    int32_t n_vals = sizeof(x)/sizeof(x[0]);
    printf("Test sqrt\n");
    for (int32_t i=0; i<n_vals; i++) {
        orientation_ctrl_setpoint(deg2rad1000(x[i]), REL);
        printf("%i -> %i\n\r", x[i], rad10002deg(orient_setpoint));
    }
    printf("done\n");
}

void test_ctrl()
{
    test_orientation_ctrl();
}