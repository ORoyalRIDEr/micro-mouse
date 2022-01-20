#include <commander.h>
#include <main.h>

#include <Lib/str.h>
#include <Lib/printf.h>

#include <Drivers/HCSR04.h>
#include <Drivers/lre_stepper.h>

volatile enum program {NONE, GO, TURN, STOP, DIST} program;
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
    else if (strcmp("dist", str))
        program = DIST;
    else if (strcmp("stop", str))
        program = STOP;

}

void commander(void)
{
    uint32_t distances[4];

    cprintf("\n\rWall-E ready\n\r");

    while (1)
    {
        switch(program) {
        case GO:
            forward(speed_cmd);
            program = NONE;
            break;
        case TURN:
            rotate(speed_cmd);
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
        case STOP:
            forward(0);
            program = NONE;
            break;
        }
    }
}