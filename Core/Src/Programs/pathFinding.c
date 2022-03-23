#include <programs.h>
#include <main.h>
#include <Lib/cmath.h>
#include <Ecl/driver.h>

uint8_t map[MAP_SIDE*2+1][MAP_SIDE+1] = {{2}};
uint8_t route[MAP_SIDE*MAP_SIDE][2];
enum cell_state_t {NONE, OPEN, CLOSED};

/* Returns the 1d index of the first instance of the smallest element in the array */
int16_t smallest_element_map(uint8_t a[MAP_SIDE][MAP_SIDE], enum cell_state_t cell_states[MAP_SIDE][MAP_SIDE]);

void read_wall(uint8_t x, uint8_t y, uint8_t* walls)
{
    uint8_t cell_y = y*2;
    int8_t cell_i[][2] = {
        {cell_y, x},
        {cell_y+1, x+1},
        {cell_y+2, x},
        {cell_y+1, x},
    };

    for (uint8_t i=0; i<4; i++) {
        int8_t* ind = cell_i[i];
        if ((ind[0] > MAP_SIDE*2+1) || (ind[1] > MAP_SIDE+1))
            walls[i] = -1;
        else
            walls[i] = map[ind[0]][ind[1]];
    }

    walls[0] = map[cell_y][x];
    walls[1] = map[cell_y+1][x+1];
    walls[2] = map[cell_y+2][x];
    walls[3] = map[cell_y+1][x];
}

void write_wall(uint8_t x, uint8_t y, uint8_t wall_n, uint8_t wall_e, uint8_t wall_s, uint8_t wall_w)
{
    uint8_t cell_y = 2*y; 
    
    map[cell_y][x] = wall_n;
    map[cell_y+1][x+1] = wall_e;
    map[cell_y+2][x] = wall_s;
    map[cell_y+1][x] = wall_w;
}

void sample_map(void){
    uint8_t model[MAP_SIDE*2][MAP_SIDE+1] = {
      {1,  1,  1,  1,  1,  1,  1,  1},
    {1,  0,  1,  0,  1,  0,  0,  1},
      {0,  0,  0,  0,  1,  0,  0,  0},
    {1,  1,  0,  1,  0,  1,  1,  1},
      {1,  0,  1,  1,  0,  0,  0,  0},
    {1,  0,  0,  0,  1,  1,  1,  1},
      {1,  1,  0,  1,  0,  1,  0,  0},
    {1,  1,  1,  1,  1,  0,  0,  1},
      {0,  0,  0,  0,  1,  1,  0,  0},
    {1,  1,  1,  0,  1,  0,  0,  1},
      {0,  0,  1,  1,  1,  0,  0,  0},
    {1,  1,  0,  0,  0,  0,  1,  1},
      {0,  1,  0,  0,  1,  1,  1,  0},
    {1,  0,  0,  1,  0,  0,  0,  1},
      {1,  1,  1,  1,  1,  1,  1,  0}
   };
    
    for (int i = 0; i < MAP_SIDE*2+1; i++) {
        for (int j = 0; j < MAP_SIDE; j++){
            map[i][j] = model[i][j];
        }
    }
}


void sample_route(void)
{
    const uint8_t cells[7][2] = {{1,1},{1,2},{1,3},{2,3},{3,3},{3,2},{3,1}};

    for (uint8_t i = 0; i < 7; i++){
        route[i][0] = cells[i][0];
        route[i][1] = cells[i][1];
    }
}

uint8_t loc_is_in_route(uint8_t x, uint8_t y)
{
    for (uint8_t i=0; i<MAP_SIDE*MAP_SIDE; i++)
        if ((x==route[i][1]) && (y==route[i][0]))
            return 1;
    return 0;
}

void print_map(void)
{
    uint8_t location[2];
    get_position(location);

    for (int8_t y = 0; y <= 2*MAP_SIDE; y++)
    {
        for (uint8_t x = 0; x <= MAP_SIDE; x++)
        {
            int8_t maze_val = map[y][x];

            if (y % 2 == 0)
            {
                cprintf("+");
                if (x == MAP_SIDE)
                    continue; // only uneven lines use whole space
                if (maze_val)
                    cprintf("---");
                else
                    cprintf("   ");
            }
            else
            {
                if (maze_val)
                    cprintf("|");
                else
                    cprintf(" ");

                if ((2 * location[1] + 1 == y) && (location[0] == x))
                    cprintf(" x ");
                else if (loc_is_in_route(x, (y-1)/2))
                    cprintf(" o ");
                else
                    cprintf("   ");
            }
        }
        cprintf("\n\r");
    }
}

uint8_t path_to_cell(uint8_t goal_x, uint8_t goal_y, uint8_t origin_x, uint8_t origin_y)
{
    uint8_t astar_f[MAP_SIDE][MAP_SIDE];
    for (uint8_t i=0; i<MAP_SIDE; i++) for (uint8_t j=0; j<MAP_SIDE; j++) astar_f[i][j] = 255;
    astar_f[origin_y][origin_x] = 0;
    uint8_t astar[MAP_SIDE][MAP_SIDE] = {{255}};
    uint8_t directions[MAP_SIDE][MAP_SIDE] = {{255}};
    enum cell_state_t cell_states[MAP_SIDE][MAP_SIDE];
    for (uint8_t i=0; i<MAP_SIDE; i++) for (uint8_t j=0; j<MAP_SIDE; j++) cell_states[i][j] = NONE;
    cell_states[origin_y][origin_x] = CLOSED;
    cprintf("(%i,%i) -> (%i,%i)\n\r", origin_y, origin_x, goal_y, goal_x);
    
    uint8_t walls[4];
    uint8_t af = 0; //distance from start
    int16_t i_smallest = -1;
    uint8_t l[2]; //l[0] = y, l[1] = x
    l[0] = origin_y;
    l[1] = origin_x; 
    uint8_t x = origin_x;
    uint8_t y = origin_y;
    uint8_t goal_not_found = 1;
    uint8_t route_length = 0;
    uint8_t direction = 0;

    while (goal_not_found){
        read_wall(l[1], l[0], walls);
        af = astar_f[l[0]][l[1]] + 1;

        if ((walls[0] == 0) && (l[0]>0)) {    //north
            x = l[1];
            y = l[0] - 1;
            if ((cell_states[y][x] != CLOSED) && (af < astar_f[y][x])) {
                astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
                astar_f[y][x] = af;
                directions[y][x] = 2;
                cell_states[y][x] = OPEN;
            }
        }
        if ((walls[1] == 0) && (l[1]<(MAP_SIDE-1))) {    //east
            x = l[1] + 1;
            y = l[0];
            if ((cell_states[y][x] != CLOSED) && (af < astar_f[y][x])) {
                astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
                astar_f[y][x] = af;
                directions[y][x] = 3;
                cell_states[y][x] = OPEN;
            }
        }
        if ((walls[2] == 0) && (l[0]<(MAP_SIDE-1))) {    //south
            x = l[1];
            y = l[0] + 1;
            if ((cell_states[y][x] != CLOSED) && (af < astar_f[y][x])) {
                astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
                astar_f[y][x] = af;
                directions[y][x] = 0;
                cell_states[y][x] = OPEN;
            }
        }
        if ((walls[3] == 0) && (l[1]>0)) {    //west
            x = l[1] - 1;
            y = l[0];
            if ((cell_states[y][x] != CLOSED) && (af < astar_f[y][x])) {
                astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
                astar_f[y][x] = af;
                directions[y][x] = 1;
                cell_states[y][x] = OPEN;
            }
        }
        i_smallest = smallest_element_map(astar, cell_states);
        if (i_smallest < 0) {
            cprintf("No route found!\n\r");
            return 0;
        }
        l[0] = i_smallest/MAP_SIDE;
        l[1] = i_smallest%MAP_SIDE;
        cell_states[l[0]][l[1]] = CLOSED;

        if(l[0] == goal_y && l[1] == goal_x){
            goal_not_found = 0;
            route_length = astar_f[goal_y][goal_x]+1;
        }
    }

    for (uint8_t i = 0; i < MAP_SIDE*MAP_SIDE; i++)
    {
        route[i][0] = 255;
        route[i][1] = 255;
    }
    
    l[0] = goal_y;
    l[1] = goal_x; 
    route[route_length-1][0] = goal_y; 
    route[route_length-1][1] = goal_x;
    for (uint8_t i = 1; i <= route_length; i++) {
        direction = directions[l[0]][l[1]];
        if (direction == 0) {    //north
            x = l[1];
            y = l[0] - 1;
        }
        if (direction == 1) {    //east
            x = l[1] + 1;
            y = l[0];
        }
        if (direction == 2) {    //south
            x = l[1];
            y = l[0] + 1;
        }
        if (direction == 3) {    //west
            x = l[1] - 1;
            y = l[0];        
        }
        route[route_length - i - 1][1] = x;
        route[route_length - i - 1][0] = y; 
        l[1] = x;
        l[0] = y;
    }
    return route_length;
}

int16_t smallest_element_map(uint8_t a[MAP_SIDE][MAP_SIDE], enum cell_state_t cell_states[MAP_SIDE][MAP_SIDE])
{
    int32_t minimum = 2*32-1;
    int16_t index = -1;
    int32_t value;
    enum cell_state_t state;
    for (uint8_t i = 0; i < MAP_SIDE; i++) {
        for (uint8_t j = 0; j < MAP_SIDE; j++)
        {
            value = a[i][j];
            state = cell_states[i][j];

            if ((value < minimum) && (state == OPEN)) {
                minimum = value;
                index = i*MAP_SIDE + j;
            }            
        }
    }
    return index;
}

uint8_t* get_map(void)
{
    return *map;
}

uint8_t* get_route(void)
{
    return route;
}
