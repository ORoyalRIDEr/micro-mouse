#include <stdio.h>
#include "../../Core/Inc/Lib/cmath.h"
#include "../../Core/Inc/Ecl/state_estimator.h"
#include "../../Core/Inc/Ecl/orientation_ctrl.h"

extern int32_t est_pos[]; // um
extern int32_t est_V;          // um/s
extern int32_t est_Psi; // 1000 rad, -pi ... pi

int32_t map_2[][8] = {
      {1, 1, 1, 1, 1, 1, 1, 0},
    {1, 0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 1},
      {0, 0, 0, 0, 0, 0, 0, 0},
    {1, 0, 0, 0, 0, 0, 0, 1},
      {1, 1, 1, 1, 1, 1, 1, 0}
};

void test_slam()
{
    init_maze();
    est_Psi = 1571*1000;

    est_pos[0] = (200+10-45)*1000;
    est_pos[1] = (200+10-60)*1000;
    //est_pos[1] = 100 * 1000 + 5*200*1000;

    print_maze();

    //uint32_t dist[] = {40000+1000, 90000-1000, 40000+3000, 140000+200000-3000};
    //uint32_t dist[] = {55000, 0, 400000, 55000};
    uint32_t dist[] = {400000, 0, 180000, 55000};

    for (int i=0; i<20; i++) {
      slam(dist);
      //printf("Pos: (%i, %i)\n", est_pos[0], est_pos[1]);
    }
    print_maze();
}
