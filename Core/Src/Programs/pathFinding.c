#include <programs.h>
#include <main.h>

uint8_t map[MAP_SIDE*2][MAP_SIDE];
uint8_t route[MAP_SIDE*MAP_SIDE][2];

int8_t location_row = 1;
int8_t location_column = 5;

void sample_map(void){
    for (int i = 0; i < MAP_SIDE*MAP_SIDE*2; i++) {
        int8_t row = i % MAP_SIDE;
        int8_t column = i % MAP_SIDE; 
        map[row][column] = i%3;
    }
    cprintf("Sample map initialised!");
}

void sample_route(void){
    const int8_t cells[7][2] = {{1,1},{1,2},{1,3},{2,3},{3,3},{3,2},{3,1}};

    for (int8_t i = 0; i < 7; i++){
        route[i][0] = cells[i][0];
        route[i][1] = cells[i][1];
    }
            
    cprintf("Sample route initialised!");
}

void print_map(void){
    //i = row, j = column
    for (int8_t i = 0; i < MAP_SIDE; i++) {
        for (int8_t j = 0; j < MAP_SIDE; j++){                
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
                    if(i == location_row*2 && j == location_column){
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