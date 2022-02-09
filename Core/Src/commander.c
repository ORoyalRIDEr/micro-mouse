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

volatile enum program {NONE, GO, STOP, TURN, DIST, STATE, DRIVE, PARK, FOLLOW_L, ORIENT, SAMPLE_MAP, SAMPLE_ROUTE ,MAP} program;
int8_t speed_cmd = 0;
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
}

void commander(void)
{
    uint32_t distances[4];

    cprintf("\n\rWall-E ready\n\r");

    int map[map_size*2][map_size];
    int route[map_size*map_size][2];

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
            sample_map(map);
            break;

        case SAMPLE_ROUTE: ;
            const int8_t cells[7][2] = {{1,1},{1,2},{1,3},{2,3},{3,3},{3,2},{3,1}};

            for (int8_t i = 0; i < 7; i++)
            {
                route[i][0] = cells[i][0];
                route[i][1] = cells[i][1];
            }
            
            cprintf("Sample route initialised!");
            break;

        case MAP: ;
            int8_t location_row = 1;
            int8_t location_column = 5;

            //i = row, j = column
            for (int8_t i = 0; i < map_size; i++) {
                for (int8_t j = 0; j < map_size; j++){                
                    if(i%2 == 0){
                        if(j%map_size != map_size-1){
                            cprintf("+");
                        }
                        else{
                            cprintf("\n");
                        }
                        cprintf("%u", map[i][j]);
                    }
                    else{
                        cprintf("%u", map[i]);
                        if(j%map_size != map_size-1){
                            if(i == location_row*2 && j == location_column){
                                cprintf("X");
                            }
                            else{
                                cprintf(" ");
                            }
                        }
                        else{
                            cprintf("\n");
                    }
                }
                }
            }
            break;

        default:;
        }
    }
}