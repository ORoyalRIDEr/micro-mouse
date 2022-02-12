#include <programs.h>

void parking()
{
    uint32_t distances[4];
    for (uint32_t i=0; i<10; i++) {
        HCSR04_Measure();
        HAL_Delay(100);
        HCSR04_Read(distances);
        cprintf("Front: %u\t Left: %u\t Right: %u\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000); 
    }        
    while ((distances[DIST_FRONT]/1000 > 170))
    {
        /*forward(10);
        HAL_Delay(10);
        forward(20);
        HAL_Delay(10);
        forward(30);
        HAL_Delay(10);
        forward(40);
        HAL_Delay(10); */
        forward(50);
        HAL_Delay(1000);
        //forward(0);
        for (uint32_t i=0; i<10; i++) {
        HCSR04_Measure();
        HAL_Delay(100);
        HCSR04_Read(distances);
        }
    }
    if (distances[DIST_FRONT]/1000 < 170) //distances in mm
    { 
        forward(0);
        rotate(62);
        HAL_Delay(2000);
        rotate(0);
        forward(0);
        
        forward(-10);
        HAL_Delay(10);
        forward(-20);
        HAL_Delay(10);
        forward(-30);
        HAL_Delay(10);
        forward(-40);
        HAL_Delay(10);
        forward(-50);
        HAL_Delay(2500);
        forward(0);
    }
}


void follow_left_wall() 
{
    uint32_t distances[4]; 
    uint32_t i;
    uint32_t n = 10;
    uint32_t front[n];     //interim_results_front
    uint32_t left[n];     //interim_results_left
    uint32_t right[n];     //interim_results_right
    uint32_t average_front;
    uint32_t average_left; 
    uint32_t average_right;       

    do {
        uint32_t sum_front = 0;         //reset sum of interim results front 
        uint32_t sum_left = 0;         //reset sum of interim results left
        uint32_t sum_right = 0;         //reset sum of interim results right
        for (i=0; i<n; i++) {
            HCSR04_Measure();
            HAL_Delay(100);
            HCSR04_Read(distances);
            cprintf("Front: %u\t Left: %u\t Right: %u\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);
            front[i] = distances[DIST_FRONT]/1000;
            left[i] = distances[DIST_LEFT]/1000;
            right[i] = distances[DIST_RIGHT]/1000;
            sum_front += front[i];
            sum_left += left[i];
            sum_right += right[i];
        }                       
            average_front = sum_front/n;
            average_left = sum_left/n;
            average_right = sum_right/n;

            cprintf("average_front: %u\n\r", average_front);
            cprintf("average_left: %u\n\r", average_left);
            cprintf("average_right: %u\n\r", average_right);
        
        if (average_left > 100 && average_left < 200)
        {
            cprintf("entering while loop\n\r");
            cprintf("Front: %u\t Left: %u\t Right: %u\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);
            /*forward(10);
            HAL_Delay(10);
            forward(20);
            HAL_Delay(10);
            forward(30);
            HAL_Delay(10);
            forward(40);
            HAL_Delay(10); */
            forward(50);
            HAL_Delay(1000);
            //forward(0);                       //Messung bei Stillstand i.d.R. präziser
        }
        if (average_left < 100) //distances in mm
        { 
            cprintf("distance left smaller than allowed\n\r");
            cprintf("Front: %u\t Left: %u\t Right: %u\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);    
            /*forward(0);
            rotate(10);
            HAL_Delay(10);
            rotate(20);
            HAL_Delay(10);  */
            rotate(30);
            HAL_Delay(500);
            
            /*forward(0);
            HAL_Delay(10);
            forward(10);
            HAL_Delay(10);
            forward(20);
            HAL_Delay(10);  */
            forward(50);
            HAL_Delay(500);
        }
        if (average_left > 200) //distances in mm
        { 
            cprintf("distance left bigger than allowed\n\r");     
            //forward(0);
            //rotate(-10);
            //rotate(-20);
            rotate(-30);
            HAL_Delay(500);
            
            /*forward(0);
            forward(10);
            HAL_Delay(10);
            forward(20);
            HAL_Delay(10);  */
            forward(50);
            HAL_Delay(500);
        }
        if (average_front < 10) //distances in mm
        {
            cprintf("distance front smaller than allowed\n\r");
            cprintf("Front: %u\t Left: %u\t Right: %u\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);      
            forward(0);
            /*rotate(10);
            HAL_Delay(10);
            rotate(20);
            HAL_Delay(10);
            rotate(30);
            HAL_Delay(10);
            rotate(40);
            HAL_Delay(10);  */
            rotate(50);
            HAL_Delay(1300);
            forward(0);
        }
    } while (/*!*/(average_front > 10));
}
            