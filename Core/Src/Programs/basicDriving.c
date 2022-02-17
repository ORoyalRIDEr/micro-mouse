#include <programs.h> 
#include <Ecl/state_estimator.h>


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
    uint8_t stop = 0;
    int32_t distances[4];
    const int32_t dcmd = 250000;
    const int32_t kinv = 30000;
    uint8_t has_turned = 0;
    uint8_t n_under_dcmd = 0;

    forward(30);

    while (!stop) {
        HCSR04_Measure();
        HAL_Delay(100);
        HCSR04_Read(distances);

        cprintf("d: %i\n\r", distances[DIST_FRONT]/1000);

        int32_t pos[2] = {0, 0};
        int32_t V;
        int32_t Psi;
        get_state(pos, &V, &Psi);
        
        if ((distances[DIST_FRONT] <= dcmd) && (distances[DIST_FRONT] != 0)) {
            n_under_dcmd++;
        }
        else
            n_under_dcmd = 0;

        if ((n_under_dcmd >= 10) && (!has_turned)) {
            forward(0);
            stop = 1;
        }
    }

    forward(0);

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
    if (distances[DIST_FRONT]/1000 < 190) //distances in mm
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
        HAL_Delay(1800);
        forward(0);
    }
}

void follow_left_wall() 
{
    uint8_t stop = 0;
    int32_t distances[4];
    const int32_t dcmd = 200000;
    const int32_t kinv = 30000;

    forward(50);

    while (!stop) {
        HCSR04_Measure();
        HAL_Delay(10);
        HCSR04_Read(distances);

        int32_t d = distances[DIST_LEFT];
        if (d == 0)
            continue;

        int32_t error = dcmd - d;

        int32_t rot_cmd = error / kinv;
        rotate(rot_cmd);

        //cprintf("d: %i, rot_cmd: %i\n\r", distances[DIST_FRONT]/1000, rot_cmd);

        int32_t pos[2] = {0, 0};
        int32_t V;
        int32_t Psi;
        get_state(pos, &V, &Psi);

        //cprintf("d: %i\n\r", pos[0]);
        if (pos[0] >= 2000)
            stop = 1;
    }

    forward(0);
}
            
void follow_curve() 
{
    uint8_t stop = 0;
    int32_t distances[4];
    const int32_t dcmd = 200000;
    const int32_t kinv = 30000;
    uint8_t has_turned = 0;
    uint8_t n_under_dcmd = 0;

    forward(30);

    while (!stop) {
        HCSR04_Measure();
        HAL_Delay(100);
        HCSR04_Read(distances);

        int32_t d = distances[DIST_LEFT];
        cprintf("d: %i, n: %i\n\r", distances[DIST_FRONT]/1000, d/1000);

        if (d != 0) {
            int32_t error = dcmd - d;
            int32_t rot_cmd = error / kinv;
            rotate(rot_cmd);
        }

        //cprintf("d: %i, rot_cmd: %i\n\r", distances[DIST_FRONT]/1000, rot_cmd);

        int32_t pos[2] = {0, 0};
        int32_t V;
        int32_t Psi;
        get_state(pos, &V, &Psi);

        //cprintf("d: %i\n\r", pos[0]);
        if ((pos[0] >= 600) && has_turned)
            stop = 1;
        
        if ((distances[DIST_FRONT] <= dcmd) && (distances[DIST_FRONT] != 0)) {
            n_under_dcmd++;
        }
        else
            n_under_dcmd = 0;
        if ((n_under_dcmd >= 10) && (!has_turned)) {
            forward(0);
            uint8_t this_is_really_the_corner = 1;
            for (uint8_t i=0; i<5; i++) {
                HCSR04_Measure();
                HAL_Delay(10);
                HCSR04_Read(distances);

                if (distances[DIST_FRONT] == 0) {
                    this_is_really_the_corner = 0;
                    break;
                }
            }

            // TURN
            if (this_is_really_the_corner) {
                rotate(50);
                HAL_Delay(1300);
                forward(50);
                has_turned = 1;

                extern int32_t est_pos[];
                est_pos[0] = 0;
            }
            else {
                n_under_dcmd = 0;
                forward(50);
                cprintf("no corner \n\r");
                HAL_Delay(3000);
            }
        }
    }

    forward(0);
}