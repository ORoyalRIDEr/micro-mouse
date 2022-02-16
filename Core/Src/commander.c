#include <commander.h>
#include <main.h>
#include <ctrl_stack.h>
#include <programs.h>

#include <Lib/str.h>
#include <Lib/printf.h>

#include <Drivers/HCSR04.h>
#include <Drivers/lre_stepper.h>

#include <Ecl/state_estimator.h>
#include <Ecl/orientation_ctrl.h>

volatile enum program {NONE, GO, STOP, TURN, DIST, STATE, DRIVE, PARK, FOLLOW_L, ORIENT, SAMPLE_MAP, SAMPLE_ROUTE, MAP, W_WRITE, W_READ, POSITION, HEADING} program;
int8_t speed_cmd = 0;
int32_t arg_number = 0;
int32_t arg_1 = 0;
int32_t arg_2 = 0;
int32_t arg_3 = 0;
int32_t arg_4 = 0;
int32_t arg_5 = 0;
int32_t arg_6 = 0;
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
        else if (strcmp("ds", argv[1])) {
            program = DRIVE;
            arg_number = atoi(argv[2]);
        }
        else if (strcmp("st", argv[1])) {
            program = STOP;
        }
        else if (strcmp("rt", argv[1])) {
            program = TURN;
            arg_number = atoi(argv[2]);
        }
    }
    else if (strcmp("light", str)) {
        HAL_GPIO_TogglePin(GPIOC, LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin);
    }
    else if (strcmp("stop", str))
        program = STOP;

    else if (strcmp("state", str)) {
        cprintf("test\n\r");    
        program = STATE;
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
    else if (strcmp("sample_map", str))
        program = SAMPLE_MAP;
    else if (strcmp("sample_route", str))
        program = SAMPLE_ROUTE;
    else if (strcmp("map", str))
        program = MAP;

    else if (strcmp("mz", str)) {
        if (strcmp("wr", argv[1])) {
            program = GO;
            arg_1 = atoi(argv[2]);
            arg_2 = atoi(argv[3]);
            arg_3 = atoi(argv[4]);
            arg_4 = atoi(argv[5]);
        }
        else if (strcmp("rd", argv[1])) {
            program = DRIVE;
            arg_1 = atoi(argv[2]);
            arg_2 = atoi(argv[3]);
        }
        else if (strcmp("ps", argv[1])) {
            program = POSITION;
        }
        else if (strcmp("hd", argv[1])) {
            program = HEADING;
        }
    }
    
}

void commander(void)
{
    int32_t distances[4];

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
            for (uint32_t i=0; i<10; i++) {
                HCSR04_Measure();
                HAL_Delay(100);
                HCSR04_Read(distances);
                cprintf("Front: %i\t Left: %i\t Right: %i\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);
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
            parking();
            program = NONE;                       
            break;

        case FOLLOW_L:    //following left wall
            ctrl_set_mode(CTRL_BASE);
            follow_left_wall();
            program = NONE;
            break;

         case ORIENT:
            ctrl_set_mode(CTRL_ORIENTATION);
            orientation_ctrl_setpoint(arg_number);
            program = NONE;
            break;

        case SAMPLE_MAP: ;
            sample_map();
            cprintf("Sample map initialised!");
            break;

        case SAMPLE_ROUTE: ;
            sample_route();
            cprintf("Sample route initialised!");
            break;

        case MAP: ;
            print_map();
            break;

        case W_READ: ;
            uint8_t w[4];
            read_wall(arg_1, arg_2, w);
            cprintf("%u %u %u %u %u %u", arg_1, arg_2, w[0], w[1], w[2],w[3]);
            break;

        case W_WRITE: ;
            write_wall(arg_1,arg_2,arg_3,arg_4,arg_5,arg_6);
            break;

        case POSITION: ;
            uint8_t* p = get_position();
            cprintf("%u %u", p[0], p[1]);
            break;

        case HEADING: ;
            uint8_t h = get_heading();
            cprintf("%u", h);
            break;

        default:;
        }
    }
}