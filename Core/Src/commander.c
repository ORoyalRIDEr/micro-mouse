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
int32_t arg_number = 0;

void bt_callback(uint8_t argc, char* argv[])
{
    char* str = argv[0];
    
    if (strcmp("tm", str)) {
        
        if (strcmp("ds", argv[1]))
            program = DIST;
        else if (strcmp("od", argv[1]))
            program = STATE;
    }
    else if (strcmp("mv", str)) {
        if (strcmp("sp", argv[1])) {
            program = GO;
            arg_number = atoi(argv[2]);
        }
        else if (strcmp("rt", argv[1])) {
            program = TURN;
            arg_number = atoi(argv[2]);
        }
        else if (strcmp("ds", argv[1])) {
            program = DRIVE;
            arg_number = atoi(argv[2]);
        }
    }
    else if (strcmp("light", str)) {
        HAL_GPIO_TogglePin(GPIOC, LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin);
    }
    else if (strcmp("go", str)) {
        program = GO;
        arg_number = atoi(argv[1]);
    }
    else if (strcmp("turn", str)) {
        program = TURN;
        arg_number = atoi(argv[2]);
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
        arg_number = atoi(str + sizeof("orient"));
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
            forward(arg_number);
            program = NONE;
            break;
        case TURN:
            ctrl_set_mode(CTRL_BASE);
            rotate(arg_number);
            program = NONE;
            break;
        case STOP:
            ctrl_set_mode(CTRL_BASE);
            forward(0);
            program = NONE;
            break;
        case DIST:
            for (uint32_t i=0; i<20; i++) {
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
            cprintf("Position: (%i,%i)\n\r",
                pos[0], pos[1]);
            program = NONE;
            break;
        case DRIVE:
            ctrl_set_mode(CTRL_BASE);
            forward(50);
            HAL_Delay(19200/1000*arg_number);
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
            program = NONE;
            break;    

        case FOLLOW_L:    //following left wall
            ctrl_set_mode(CTRL_BASE);
            for (uint32_t i=0; i<10; i++) {
                HCSR04_Measure();
                HAL_Delay(100);
                HCSR04_Read(distances);
                cprintf("Front: %u\t Left: %u\t Right: %u\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000); 
            }
            
            while (distances[DIST_LEFT]/1000 > 100 && distances[DIST_LEFT]/1000 < 200 && distances[DIST_FRONT]/1000 > 200 && distances[DIST_RIGHT]/1000 > 200)
            {
                cprintf("entering while loop\n\r");
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
                program = FOLLOW_L;
                break;
            }
            if (distances[DIST_LEFT]/1000 < 100 && distances[DIST_FRONT]/1000 > 200 && distances[DIST_RIGHT]/1000 > 200) //distances in mm
            { 
                cprintf("distance left smaller than allowed\n\r");    
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
                program = FOLLOW_L;
                break;
            }
            if (distances[DIST_LEFT]/1000 > 200 && distances[DIST_FRONT]/1000 > 200 && distances[DIST_RIGHT]/1000 > 200) //distances in mm
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
                program = FOLLOW_L;
                break;
            }
            if (distances[DIST_FRONT]/1000 < 200) //distances in mm
            {
                cprintf("distance front smaller than allowed\n\r");      
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
            
                program = FOLLOW_L;
                break;
            }
            if (distances[DIST_FRONT]/1000 < 200 && distances[DIST_RIGHT]/1000 < 200)   {
                program = PARK;
                break;
            }      

        case ORIENT:
            ctrl_set_mode(CTRL_ORIENTATION);
            orientation_ctrl_setpoint(arg_number);
            program = NONE;
            break;

        default:;
        }
    }
}