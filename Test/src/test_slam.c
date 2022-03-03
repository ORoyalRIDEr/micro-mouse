#include <stdio.h>
#include "../../Core/Inc/Ecl/state_estimator.h"

void slam(int32_t dist[], uint32_t maze_pos[]);

extern int32_t est_pos[]; // um
extern int32_t est_V;          // um/s
extern int32_t est_Psi; // 1000 rad, -pi ... pi

void test_slam()
{
    init_maze();

    est_Psi = -1571;
    est_pos[0] =  50 * 1000 + 5*200*1000;
    est_pos[1] = 100 * 1000 + 5*200*1000;

    print_maze();

    //uint32_t dist[] = {40000+1000, 90000-1000, 40000+3000, 140000+200000-3000};
    uint32_t dist[] = {40000, 0, 55000, 0x0FFFFFFF};
    uint32_t maze_pos[2];

    for (int i=0; i<20; i++)
        slam(dist, maze_pos);
        print_maze();
}
