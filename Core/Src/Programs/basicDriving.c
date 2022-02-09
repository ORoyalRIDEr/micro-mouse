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
        
    do {

        for (uint32_t i=0; i<1; i++) {
            HCSR04_Measure();
            HAL_Delay(100);
            HCSR04_Read(distances);
            cprintf("Front: %u\t Left: %u\t Right: %u\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000); 
    }

        if (distances[DIST_LEFT]/1000 > 100 && distances[DIST_LEFT]/1000 < 200)
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
        if (distances[DIST_LEFT]/1000 < 100) //distances in mm
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
        if (distances[DIST_LEFT]/1000 > 200) //distances in mm
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
        if (distances[DIST_FRONT]/1000 < 50) //distances in mm
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
    } while (/*!*/(distances[DIST_FRONT]/1000 > 50));
}
            