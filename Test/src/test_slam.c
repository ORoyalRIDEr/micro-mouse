#include <stdio.h>
#include "../../Core/Inc/Ecl/state_estimator.h"

void slam(int32_t dist[], uint32_t maze_pos[]);

extern int32_t est_pos[]; // um
extern int32_t est_V;          // um/s
extern int32_t est_Psi; // 1000 rad, -pi ... pi

void test_slam()
{
    est_Psi = -1570;

    uint32_t dist[] = {0, 0, 0, 0};
    uint32_t maze_pos[2];

    slam(dist, maze_pos);
}
