#include <stdio.h>
#include "../../Core/Inc/programs.h"

void test_pathfinding()
{
  sample_map(); //{5,6} -> (4,3)
//  sample_route();
  path_to_cell(4, 3);
  print_map();
}