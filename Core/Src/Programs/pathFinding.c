#include <programs.h>
#include <main.h>

uint8_t map[MAP_SIDE*2][MAP_SIDE] = {2};
uint8_t route[MAP_SIDE*MAP_SIDE][2];

uint8_t location[2] = {1,5};
uint8_t heading = 0;


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
