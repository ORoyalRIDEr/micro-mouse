#include <programs.h>
#include <main.h>
#include <Lib/cmath.h>

uint8_t map[MAP_SIDE*2][MAP_SIDE] = {{2}};
uint8_t route[MAP_SIDE*MAP_SIDE][2];

uint8_t location[2] = {1,5}; //location[0] = y, location[1] = x
uint8_t heading = 0;

void read_wall(uint8_t x, uint8_t y, uint8_t* walls){
    uint8_t cell_y = y*2;

    walls[0] = map[cell_y][x];
    walls[1] = map[cell_y+1][x+1];
    walls[2] = map[cell_y+2][x];
    walls[3] = map[cell_y+1][x];
}

void write_wall(uint8_t x, uint8_t y, uint8_t wall_n, uint8_t wall_e, uint8_t wall_s, uint8_t wall_w){
    uint8_t cell_y = 2*y; 
    
    map[cell_y][x] = wall_n;
    map[cell_y+1][x+1] = wall_e;
    map[cell_y+2][x] = wall_s;
    map[cell_y+1][x] = wall_w;
}

void sample_map(void){
    for (int i = 0; i < MAP_SIDE*MAP_SIDE*2; i++) {
        uint8_t row = i % MAP_SIDE;
        uint8_t column = i % MAP_SIDE; 
        map[row][column] = i%3;
    }
}

void sample_route(void){
    const uint8_t cells[7][2] = {{1,1},{1,2},{1,3},{2,3},{3,3},{3,2},{3,1}};

    for (uint8_t i = 0; i < 7; i++){
        route[i][0] = cells[i][0];
        route[i][1] = cells[i][1];
    }
}

void print_map(void){
    //i = row, j = column
    for (uint8_t i = 0; i < MAP_SIDE; i++) {
        for (uint8_t j = 0; j < MAP_SIDE; j++){                
            if(i%2 == 0){
                if(j%MAP_SIDE != MAP_SIDE-1){
                    cprintf("+");
                }
                else{
                    cprintf("\n");
                }
                cprintf("%u", map[i][j]);
            }
            else{
                cprintf("%u", map[i]);
                if(j%MAP_SIDE != MAP_SIDE-1){
                    if(i == location[0]*2 && j == location[1]){
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
}

uint8_t path_to_cell(uint8_t goal_x, uint8_t goal_y){
    uint8_t astar_f[MAP_SIDE][MAP_SIDE] = {{255}};
    uint8_t astar[MAP_SIDE][MAP_SIDE] = {{255}};
    uint8_t directions[MAP_SIDE][MAP_SIDE] = {{255}};
    
    uint8_t walls[4];
    uint8_t af = 0; //distance from start
    uint16_t i_smallest = 0;
    uint8_t l[2]; //l[0] = y, l[1] = x
    l[0] = location[0];
    l[1] = location[1]; 
    uint8_t x = location[0];
    uint8_t y = location[1];
    astar_f[l[0]][l[1]] = 0;
    uint8_t goal_not_found = 1;
    uint8_t route_length = 0;
    uint8_t direction = 0;

    while (goal_not_found){
        read_wall(l[0], l[1], walls);
        af = astar_f[l[0]][l[1]];
        if (walls[0] == 0) {    //north
            x = l[1];
            y = l[0] - 1;
            astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
            directions[y][x] = 2;
        }
        if (walls[1] == 0) {    //east
            x = l[1] + 1;
            y = l[0];
            astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
            directions[y][x] = 3;
        }
        if (walls[2] == 0) {    //south
            x = l[1] + 1;
            y = l[0];
            astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
            directions[y][x] = 0;
        }
        if (walls[3] == 0) {    //west
            x = l[1] - 1;
            y = l[0];
            astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
            directions[y][x] = 1;
        }
        i_smallest = smallest_element(astar);
        l[0] = i_smallest/MAP_SIDE;
        l[1] = i_smallest%MAP_SIDE;

        if(l[0] == goal_y && l[1] == goal_x){
            goal_not_found = 0;
            route_length = af + 1;
        }
    }

    for (uint8_t i = 0; i < MAP_SIDE*MAP_SIDE; i++)
    {
        route[i][0] = 255;
        route[i][1] = 255;
    }
    
    for (uint8_t i = 0; i < route_length; i++) {
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
        route[route_length - i][1] = x;
        route[route_length - i][0] = y; 
        l[1] = x;
        l[0] = y;
    }
    return route_length;
}

uint8_t smallest_element_map(uint8_t a[MAP_SIDE][MAP_SIDE]){
    int32_t minimum = 2*32-1;
    uint8_t index = 0;
    int32_t value;
    for (uint8_t i = 0; i < MAP_SIDE; i++) {
        for (uint8_t j = 0; j < MAP_SIDE; j++)
        {
            value = a[i][j];
            if (value < minimum) {
                minimum = value;
                index = i*MAP_SIDE + j;
            }            
        }
    }
    return index;
}

uint8_t* get_map(void){
    return *map;
}

uint8_t* get_route(void){
    return *route;
}

uint8_t* get_position(void){
    return location;
}

uint8_t get_heading(void){
    return heading;
}
