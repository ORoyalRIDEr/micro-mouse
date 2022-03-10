#include <stdio.h>
#include "../../Core/Inc/Lib/cmath.h"
#include "../../Core/Inc/Ecl/state_estimator.h"
#include "../../Core/Inc/Ecl/orientation_ctrl.h"

extern int32_t est_pos[]; // um
extern int32_t est_V;     // um/s
extern int32_t est_Psi; // 1000 rad, -pi ... pi

void test_slam()
{
    int32_t Vtrue[] = {10000, 60000};

    init_maze();
    est_Psi = 1571*1000;

    est_pos[0] = (100)*1000;
    est_pos[1] = (30)*1000;
    //est_pos[0] = (200+10-45)*1000;
    //est_pos[1] = (200+10-60)*1000;
    //est_pos[1] = 100 * 1000 + 5*200*1000;

    print_maze();

    //uint32_t dist[] = {40000+1000, 90000-1000, 40000+3000, 140000+200000-3000};
    //uint32_t dist[] = {55000, 0, 400000, 55000};
    uint32_t dist[] = {400000, 0, 50000, 60000};

    for (int i=0; i<20; i++) {
      dist[2] -= Vtrue[0]/10;
      dist[3] += Vtrue[0]/10;
      est_pos[1] += Vtrue[1]/10;

      slam(dist);
      //printf("Pos: (%i, %i), Heading: %i\n", est_pos[0]/1000, est_pos[1]/1000, est_Psi*180/PI1000/1000);
    }
    print_maze();
}
