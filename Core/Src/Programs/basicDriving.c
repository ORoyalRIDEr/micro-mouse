#include <programs.h>

void speed_ramp(int32_t from, int32_t to, uint16_t steps)
{
    int32_t step_size = (to-from)/steps;
    for (uint8_t i=0; i<steps; i++) {
        forward(from + step_size*i);
        HAL_Delay(10);
    }
}

void parking()
{
    int32_t distances[4];
    for (uint32_t i=0; i<10; i++) {
        HCSR04_Measure();
        HAL_Delay(100);
        HCSR04_Read(distances);
        cprintf("Front: %i\t Left: %i\t Right: %i\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000); 
    }        
    while ((distances[DIST_FRONT]/1000 > 170))
    {
        //speed_ramp
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

        for (uint8_t i=0; i<5; i++) {
            forward(-10*i);
            HAL_Delay(10);
        }
        //speed_ramp_reverse 
        forward(-50);
        HAL_Delay(2500);
        forward(0);
    }
}

void follow_left_wall() 
{
    int32_t distances[4]; 
    uint32_t i;
    uint32_t n = 10;
    int32_t front[n-1];     //interim_results_front
    int32_t left[n-1];     //interim_results_left
    int32_t right[n-1];     //interim_results_right
    int32_t average_front;
    int32_t average_left; 
    int32_t average_right;
   
    do {
        uint32_t sum_front = 0;         //reset sum of interim results front 
        uint32_t sum_left = 0;         //reset sum of interim results left
        uint32_t sum_right = 0;         //reset sum of interim results right
        
        uint32_t f = n;         //Zählfaktor für Mittelwert front sensor
        uint32_t l = n;         //Zählfaktor für Mittelwert left sensor
        uint32_t r = n;       //Zählfaktor für Mittelwert right sensor

        for (i=0; i<n; i++) {
            HCSR04_Measure();
            HAL_Delay(100);
            HCSR04_Read(distances);
            cprintf("Front: %i\t Left: %i\t Right: %i\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);
            
            front[i] = distances[DIST_FRONT]/1000;
            left[i] = distances[DIST_LEFT]/1000;
            right[i] = distances[DIST_RIGHT]/1000;
            sum_front += front[i];
            sum_left += left[i];
            sum_right += right[i];
            
            //f = (distances[DIST_FRONT]/1000 == 0) ? f -- : f;
            //l = (distances[DIST_LEFT]/1000 == 0) ? l -- : l;
            //r = (distances[DIST_RIGHT]/1000 == 0) ? r -- : r; 

            if (distances[DIST_FRONT]/1000 == 0) {
                f --;
            }
            if (distances[DIST_LEFT]/1000 == 0) {
                l --;
            }
            if (distances[DIST_RIGHT]/1000 == 0) {
                r --;
            }
        }   
            cprintf("n: %u\n\r", n);
            cprintf("f: %u\n\r", f);
            cprintf("l: %u\n\r", l);
            cprintf("r: %u\n\r", r);

            average_front = sum_front/f;
            average_left = sum_left/l;
            average_right = sum_right/r;

            cprintf("average_front: %i\n\r", average_front);
            cprintf("average_left: %i\n\r", average_left);
            cprintf("average_right: %i\n\r", average_right);
        
        if (average_left > 100 && average_left < 200)
        {
            cprintf("entering while loop\n\r");
            //cprintf("Front: %i\t Left: %i\t Right: %i\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);
            //speed_ramp
            forward(50);
            HAL_Delay(1000);
            //forward(0);                       //Messung bei Stillstand i.d.R. präziser
        }
        if (average_left < 100) //distances in mm
        { 
            cprintf("distance left smaller than allowed\n\r");
            cprintf("Front: %i\t Left: %i\t Right: %i\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);    
            //speed_ramp
            rotate(15);
            HAL_Delay(500);
            //speed_ramp
            forward(50);
            HAL_Delay(500);
        }
        
        if (average_left > 200) //distances in mm
        { 
            cprintf("distance left bigger than allowed\n\r");     
            //speed_ramp
            rotate(-15);
            HAL_Delay(500);
            //speed_ramp
            forward(50);
            HAL_Delay(500);
        }
        if (average_left == 0) //distances in mm
        { 
            cprintf("distance left bigger than allowed\n\r");     
            //speed_ramp
            rotate(-15);
            HAL_Delay(500);
            //speed_ramp
            forward(50);
            HAL_Delay(500);
        }
        if (average_front < 100 && average_front > 0 && f > n -2 ) //distances in mm
        {
            cprintf("distance front smaller than allowed\n\r");
            cprintf("Front: %i\t Left: %i\t Right: %i\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);      
            forward(0);
            //speed_ramp
            rotate(50);
            HAL_Delay(1300);
            forward(50);
        }
    } while (/*!*/(average_front > 100 || average_front == 0));
}
            