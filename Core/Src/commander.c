#include <commander.h>
#include <main.h>
#include <ctrl_stack.h>

#include <Lib/str.h>
#include <Lib/printf.h>

#include <Drivers/HCSR04.h>
#include <Drivers/lre_stepper.h>

#include <Ecl/state_estimator.h>
#include <Ecl/orientation_ctrl.h>

volatile enum program {NONE, GO, STOP, TURN, DIST, STATE, DRIVE, PARK, FOLLOW_L, ORIENT} program;
int8_t speed_cmd = 0;

void bt_callback(char* str)
{
    if (strcmp("light", str)) {
        HAL_GPIO_TogglePin(GPIOC, LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin);
    }
    else if (strncmp("go", str, sizeof("go")-1)) {
        program = GO;
        speed_cmd = atoi(str + sizeof("go"));
    }
    else if (strncmp("turn", str, sizeof("turn")-1)) {
        program = TURN;
        speed_cmd = atoi(str + sizeof("turn"));
    }
    else if (strcmp("stop", str))
        program = STOP;
    else if (strcmp("dist", str))
        program = DIST;
    else if (strcmp("state", str)) {
        cprintf("test\n\r");    
        program = STATE;
    } 
    else if (strcmp("mv ds", str)) {
        cprintf("driving 1m\n\r");
        program = DRIVE;    
    }   
    else if (strcmp("park", str)) {
        cprintf("parking\n\r");
        program = PARK;    
    }
    else if (strcmp("follow_l", str)) {
        cprintf("following left wall\n\r");     
        program = FOLLOW_L;    
    }
    else if (strncmp("orient", str, sizeof("orient")-1)) {
        program = ORIENT;
        speed_cmd = atoi(str + sizeof("orient"));
    }
}

void commander(void)
{
    uint32_t distances[4];

    cprintf("\n\rWall-E ready\n\r");

    while (1)
    {
        switch(program) {
        case GO:
            ctrl_set_mode(CTRL_BASE);
            forward(speed_cmd);
            program = NONE;
            break;
        case TURN:
            ctrl_set_mode(CTRL_BASE);
            rotate(speed_cmd);
            program = NONE;
            break;
        case STOP:
            ctrl_set_mode(CTRL_BASE);
            forward(0);
            program = NONE;
            break;
        case DIST:
            for (uint32_t i=0; i<10; i++) {
                HCSR04_Measure();
                HAL_Delay(100);
                HCSR04_Read(distances);
                cprintf("Front: %u\t Left: %u\t Right: %u\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);
            }
            program = NONE;
            break;
        case STATE:;
            int32_t pos[2], V, heading[2];
            get_state(pos, &V, heading);
            cprintf("Position: (%i,%i)\tVelocity: %i\t Heading: (%i,%i)\n\r",
                pos[0], pos[1], V, heading[0], heading[1]);
            program = NONE;
            break;
        case DRIVE:
            ctrl_set_mode(CTRL_BASE);
            forward(50);
            HAL_Delay(23000);
            forward(0);
            program = NONE;
            break;    
            
        case PARK:
            ctrl_set_mode(CTRL_BASE);
            for (uint32_t i=0; i<10; i++) {
                HCSR04_Measure();
                HAL_Delay(100);
                HCSR04_Read(distances);
                cprintf("Front: %u\t Left: %u\t Right: %u\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000); 
            }        
            while ((distances[DIST_FRONT]/1000 > 170))
            {
                forward(10);
                HAL_Delay(10);
                forward(20);
                HAL_Delay(10);
                forward(30);
                HAL_Delay(10);
                forward(40);
                HAL_Delay(10);
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
            program = NONE;
            break;    

        case FOLLOW_L:    //following left wall
            ctrl_set_mode(CTRL_BASE);
            for (uint32_t i=0; i<20; i++) {
                HCSR04_Measure();
                HAL_Delay(100);
                HCSR04_Read(distances);
                cprintf("Front: %u\t Left: %u\t Right: %u\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000); 
            }
            
            while (distances[DIST_LEFT]/1000 > 150 && distances[DIST_LEFT]/1000 < 250 && distances[DIST_FRONT]/1000 > 200 && distances[DIST_RIGHT]/1000 > 200)
            {
                cprintf("entering while loop\n\r");
                forward(10);
                HAL_Delay(10);
                forward(20);
                HAL_Delay(10);
                forward(30);
                HAL_Delay(10);
                forward(40);
                HAL_Delay(10);
                forward(50);
                HAL_Delay(1000);
                forward(0);                       //Messung bei Stillstand i.d.R. präziser
                program = FOLLOW_L;
                break;
            }
            if (distances[DIST_LEFT]/1000 < 150 && distances[DIST_FRONT]/1000 > 200 && distances[DIST_RIGHT]/1000 > 200) //distances in mm
            { 
                cprintf("distance left smaller than allowed\n\r");    
                forward(0);
                rotate(10);
                HAL_Delay(10);
                rotate(20);
                HAL_Delay(10);
                rotate(30);
                HAL_Delay(500);
                
                forward(0);
                forward(10);
                HAL_Delay(10);
                forward(20);
                HAL_Delay(10);
                forward(30);
                HAL_Delay(500);
                program = FOLLOW_L;
                break;
            }
            if (distances[DIST_LEFT]/1000 > 250 && distances[DIST_FRONT]/1000 > 200 && distances[DIST_RIGHT]/1000 > 200) //distances in mm
            { 
                cprintf("distance left bigger than allowed\n\r");     
                forward(0);
                rotate(-10);
                rotate(-20);
                rotate(-30);
                HAL_Delay(500);
                
                forward(0);
                forward(10);
                HAL_Delay(10);
                forward(20);
                HAL_Delay(10);
                forward(30);
                HAL_Delay(500);
                program = FOLLOW_L;
                break;
            }
            if (distances[DIST_FRONT]/1000 < 200) //distances in mm
            {
                cprintf("distance front smaller than allowed\n\r");      
                forward(0);
                rotate(10);
                HAL_Delay(10);
                rotate(20);
                HAL_Delay(10);
                rotate(30);
                HAL_Delay(10);
                rotate(40);
                HAL_Delay(10);
                rotate(50);
                HAL_Delay(1500);
                rotate(0);
                forward(0);
            
                program = FOLLOW_L;
                break;
            }
            if (distances[DIST_FRONT]/1000 < 200 && distances[DIST_RIGHT]/1000 < 200)   {
                program = PARK;
                break;
            }      

        case ORIENT:
            ctrl_set_mode(CTRL_ORIENTATION);
            orientation_ctrl_setpoint(speed_cmd);
            program = NONE;
            break;

        default:;
        }
    }
}