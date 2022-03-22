#include <commander.h>
#include <main.h>
#include <ctrl_stack.h>
#include <programs.h>

#include <Lib/str.h>
#include <Lib/cmath.h>
#include <Lib/printf.h>

#include <Drivers/HCSR04.h>
#include <Drivers/lre_stepper.h>

#include <Ecl/driver.h>
#include <Ecl/state_estimator.h>
#include <Ecl/orientation_ctrl.h>
#include <Ecl/position_ctrl.h>

volatile enum program {NONE, CPU, GO, STOP, TURN, POSITION_CTRL, DIST, MV_CELL, MV_EXPL, FOLLOW_ROUTE, STATE, HEADING, DRIVE, PARK, FOLLOW_L, FOLLOW_CURVE, ORIENT, SAMPLE_MAP, SAMPLE_ROUTE, MAP, W_WRITE, W_READ, POSITION, HEADING_2} 
    program;
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
        else if (strcmp("hd", argv[1]))
            program = HEADING;
        else if (strcmp("cpu", argv[1]))
            program = CPU;
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
        else if (strcmp("pos", argv[1])) {
            program = POSITION_CTRL;
            arg_1 = atoi(argv[2]);
            arg_2 = atoi(argv[3]);
        }
        else if (strcmp("route", argv[1])) {
            program = FOLLOW_ROUTE;
            arg_1 = atoi(argv[2]);
        }
        else if (strcmp("cell", argv[1])) {
            program = MV_CELL;
            arg_1 = atoi(argv[2]);
            arg_2 = atoi(argv[3]);
            arg_3 = atoi(argv[4]);
        }
        else if (strcmp("expl", argv[1])) {
            program = MV_EXPL;
            arg_1 = atoi(argv[2]);
            arg_2 = atoi(argv[3]);
            arg_3 = atoi(argv[4]);
        }
    }
    else if (strcmp("ctrl", str)) {
        if (strcmp("slam", argv[1])) {
            ctrl_set_mode(EST_SLAM);
            cprintf("\tTurn on slam estimator\n\r");
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
    else if (strcmp("f_l", str)) {
        cprintf("following left wall\n\r");     
        program = FOLLOW_L;                  
    }
    else if (strcmp("curve", str)) {
        cprintf("following curve\n\r");     
        program = FOLLOW_CURVE;              
    }
    else if (strncmp("orient", str, sizeof("orient")-1)) {
        program = ORIENT;
        arg_number = atoi(str + sizeof("orient"));
        arg_number = deg2rad1000(arg_number);
    }
    else if (strcmp("sample_map", str))
        program = SAMPLE_MAP;
    else if (strcmp("sample_route", str))
        program = SAMPLE_ROUTE;
    else if (strcmp("map", str))
        program = MAP;

    else if (strcmp("mz", str)) {
        if (strcmp("wr", argv[1])) {
            program = W_WRITE;
            arg_1 = atoi(argv[2]);
            arg_2 = atoi(argv[3]);
            arg_3 = atoi(argv[4]);
            arg_4 = atoi(argv[5]);
            arg_5 = atoi(argv[6]);
            arg_6 = atoi(argv[7]);
        }
        else if (strcmp("rd", argv[1])) {
            program = W_READ;
            arg_1 = atoi(argv[2]);
            arg_2 = atoi(argv[3]);
        }
        else if (strcmp("ps", argv[1])) {
            program = POSITION;
        }
        else if (strcmp("hd", argv[1])) {
            program = HEADING_2;
        }
    }
    
}

void commander(void)
{
    int32_t distances[4];
    int32_t pos[2], V, heading;

    cprintf("\n\rWall-E ready\n\r");

    while (1)
    {
        switch(program) {
        case CPU:
            cprintf("  CPU usage: %i%%/1000\n\r", get_cpu_usage());
            program = NONE;
            break;

        case GO:
            forward(arg_number);
            program = NONE;
            break;

        case TURN:
            rotate(arg_number);   
            program = NONE;
            break;

        case STOP:
            ctrl_unset_mode(CTRL_EST_ALL);
            forward(0);
            program = NONE;
            break;

        case DIST:
            for (uint32_t i=0; i<1000; i++) {
                HAL_Delay(100);
                HCSR04_Read(distances);
                cprintf("Front: %i\t Left: %i\t Right: %i\n\r", distances[DIST_FRONT]/1000, distances[DIST_LEFT]/1000, distances[DIST_RIGHT]/1000);
            }
            program = NONE;
            break;

        case STATE:;
            get_state(pos, &V, &heading);
            cprintf("Position: (%i,%i)\n\r",
                pos[0]/1000, pos[1]/1000);
            program = NONE;
            break;

        case HEADING:;
            for (uint32_t i=0; i<1; i++) {
                get_state(pos, &V, &heading);
                cprintf("Heading: %i\n\r", heading*180/PI1000/1000);
                HAL_Delay(100);
            }
            program = NONE;
            break;

        case DRIVE:
            forward(50);
            HAL_Delay(19200/1000*arg_number);
            forward(0);
            program = NONE;
            break;   

        case PARK:
            parking();
            program = NONE;                       
            break;

        case FOLLOW_L:    //following left wall
            follow_left_wall();
            program = NONE;
            break;

        case FOLLOW_CURVE:    //following left wall and make curve
            follow_curve();
            program = NONE;
            break;

        case ORIENT:
            ctrl_set_mode(CTRL_ORIENTATION);
            orientation_ctrl_setpoint(arg_number, REL);
            program = NONE;
            break;

        case POSITION_CTRL:;
            int32_t args[] = {arg_1*1000, arg_2*1000};
            pos_ctrl_setpoint(args, 50);
            ctrl_set_mode(CTRL_ORIENTATION);
            ctrl_set_mode(CTRL_POS);
            program = NONE;
            break;

        case FOLLOW_ROUTE:;
            ctrl_set_mode(CTRL_ORIENTATION);
            ctrl_set_mode(CTRL_DRIVE);
            follow_route(3, arg_1);
            program = NONE;
            break;

        case MV_CELL:;
            ctrl_set_mode(CTRL_ORIENTATION);
            ctrl_set_mode(CTRL_DRIVE);
            uint8_t cell[] = {arg_1, arg_2};
            drive_to_cell(cell, arg_3, 0);
            program = NONE;
            break;

        case MV_EXPL:;
            ctrl_set_mode(CTRL_ORIENTATION);
            ctrl_set_mode(CTRL_DRIVE);
            uint8_t cell2[] = {arg_1, arg_2};
            drive_to_cell(cell2, arg_3, 1);
            program = NONE;
            break;

        case SAMPLE_MAP: ;
            sample_map();
            cprintf("Sample map initialised!\n\r");
            program = NONE;
            break;

        case SAMPLE_ROUTE: ;
            sample_route();
            cprintf("Sample route initialised!\n\r");
            program = NONE;
            break;

        case MAP: ;
            print_map();
            //print_maze();
            program = NONE;
            break;

        case W_READ: ;
            uint8_t w[4];
            read_wall(arg_1, arg_2, w);
            cprintf("%u %u %u %u %u %u\n\r", arg_1, arg_2, w[0], w[1], w[2],w[3]);
            program = NONE;
            break;

        case W_WRITE: ;
            write_wall(arg_1,arg_2,arg_3,arg_4,arg_5,arg_6);
            program = NONE;
            break;

        case POSITION: ;
            uint8_t p[2];
            get_position(p);
            cprintf("%u %u\n\r", p[0], p[1]);
            program = NONE;
            break;

        case HEADING_2: ;
            uint8_t h = get_heading();
            cprintf("%u\n\r", h);
            program = NONE;
            break;

        default:;
        }
    }
}