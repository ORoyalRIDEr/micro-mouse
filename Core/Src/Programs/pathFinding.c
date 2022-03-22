#include <stdio.h>
#include <programs.h>
#include <main.h>
#include <Lib/cmath.h>

uint8_t map[MAP_SIDE*2][MAP_SIDE] = {{2}};
uint8_t route[MAP_SIDE*MAP_SIDE][2] = {{255}};

uint8_t location[2] = {6,6}; //location[0] = y, location[1] = x
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
    uint8_t model[MAP_SIDE*2+1][MAP_SIDE] = {
      {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,},
    {1,  0,  1,  0,  1,  0,  0,  1,  0,  0,  0,  0,  0,  0,  1},
      {0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  1,  0,  1,  0,  1,  1,  1,  0,  0,  0,  0,  0,  0,  1},
      {1,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  0,  0,  0,  1,  1,  1,  1,  0,  0,  0,  0,  0,  0,  1},
      {1,  1,  0,  1,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  1,  1,  1,  1,  0,  0,  1,  0,  0,  0,  0,  0,  0,  1},
      {0,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  1,  1,  0,  1,  0,  0,  1,  0,  0,  0,  0,  0,  0,  1},
      {0,  0,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  1,  0,  0,  0,  0,  1,  1,  0,  0,  0,  0,  0,  0,  1},
      {0,  1,  0,  0,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  0,  0,  1,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  1},
      {1,  1,  1,  1,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
      {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
      {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
      {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
      {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
      {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
      {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
      {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,},
    {1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1},
      {1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,},
    };
    
    for (int i = 0; i < MAP_SIDE*2+1; i++) {
        for (int j = 0; j < MAP_SIDE; j++){
            map[i][j] = model[i][j];
        }
    }
}

void sample_route(void){
    const uint8_t cells[8][2] = {{0,1},{1,1},{1,2},{1,3},{1,4},{1,5},{1,6},{2,6}};

    for (uint8_t i = 0; i < 8; i++){
        route[i][0] = cells[i][1];
        route[i][1] = cells[i][0];
    }
}

void print_map(void){
    //i = row, j = column
    for (uint8_t i = 0; i < MAP_SIDE*2+1; i++) {
        for (uint8_t j = 0; j < MAP_SIDE; j++){          
            printf(" ");      
            if(i%2 == 0){
                printf("+ ");
                printf("%u", map[i][j]);
                if(j == MAP_SIDE - 2){
                    printf(" +\n");
                    break;
                }
            }
            else{
                printf("%u", map[i][j]);
                if(j != MAP_SIDE-1){
                    if(i == location[0]*2 + 1 && j == location[1]){
                        printf(" X");
                    }
                    else if(is_in_route(j, i/2)){
                        printf(" ~");
                    }
                    else{
                        printf("  ");
                    }
                }
                else{
                    printf("\n");
                }
            }
        }
    }
}

uint8_t path_to_cell(uint8_t goal_x, uint8_t goal_y){ // 1, 5
    uint8_t astar_f[MAP_SIDE][MAP_SIDE];
    uint8_t astar[MAP_SIDE][MAP_SIDE];
    uint8_t directions[MAP_SIDE][MAP_SIDE];
    uint8_t not_visited[MAP_SIDE][MAP_SIDE];

    for (uint8_t i = 0; i < MAP_SIDE; i++)
    {
        for (uint8_t j = 0; j < MAP_SIDE; j++)
        {
            astar[i][j] = 255;
            astar_f[i][j] = 255; 
            directions[i][j] = 255;
            not_visited[i][j] = 1;
        }
    }
    
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

    uint8_t count = 0;
    while (goal_not_found && count < 100){
        read_wall(l[1], l[0], walls);
        not_visited[l[0]][l[1]] = 0;

        printf("walls ");
        for (uint8_t i = 0; i < 4; i++)
        {
            printf("%u ", walls[i]);
        }
        printf("\n");

        af = astar_f[l[0]][l[1]];
        if (walls[0] == 0) {    //north
            x = l[1];
            y = l[0] - 1;
            if(astar[y][x] == 255){
                astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
                astar_f[y][x] = af + 1;
                directions[y][x] = 2;
            }
        }
        if (walls[1] == 0) {    //east
            x = l[1] + 1;
            y = l[0];
            if(astar[y][x] == 255){
                astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
                astar_f[y][x] = af + 1;
                directions[y][x] = 3;
            }
        }
        if (walls[2] == 0) {    //south
            x = l[1];
            y = l[0] + 1;
            if(astar[y][x] == 255){
                astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
                astar_f[y][x] = af + 1;
                directions[y][x] = 0;
            }
        }
        if (walls[3] == 0) {    //west
            x = l[1] - 1;
            y = l[0];
            if(astar[y][x] == 255){
                astar[y][x] = absolute(x - goal_x) + absolute(y - goal_y) + af;
                astar_f[y][x] = af + 1;
                directions[y][x] = 1;
            }
        }
        i_smallest = smallest_element_map(astar, not_visited);
        l[0] = i_smallest/MAP_SIDE;
        l[1] = i_smallest%MAP_SIDE;

        printf("%u %u %u\n",i_smallest, l[1], l[0]);


        if(l[0] == goal_y && l[1] == goal_x){
            goal_not_found = 0;
            route_length = af + 1;
        }
        count++;
    }

    printf("\nDirections\n");

    for (uint8_t i = 0; i < MAP_SIDE; i++){
        for (uint8_t j = 0; j < MAP_SIDE; j++){
            printf("%u ",directions[i][j]); 
            if (j == MAP_SIDE - 1){
                printf("\n");
            }
        }
    }

    printf("\nAstar\n");

    for (uint8_t i = 0; i < MAP_SIDE; i++){
        for (uint8_t j = 0; j < MAP_SIDE; j++){
            printf("%u ",astar[i][j]); 
            if (j == MAP_SIDE - 1){
                printf("\n");
            }
        }
    }

    for (uint8_t i = 0; i < MAP_SIDE*MAP_SIDE; i++)
    {
        route[i][0] = 255;
        route[i][1] = 255;
    }
    
    route[0][1] = l[1];
    route[0][0] = l[0]; 
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
        printf("direction: %u location: %u %u\n", direction, l[1], l[0]);
    }

    for (uint8_t i = 0; i < 10; i++)
    {
        printf("route: %u %u\n", route[i][0], route[i][1]);
    }

    return route_length;
}

uint8_t smallest_element_map(uint8_t a[MAP_SIDE][MAP_SIDE], uint8_t v[MAP_SIDE][MAP_SIDE]){ // also checks that at least one cell around it has not been visited
    int32_t minimum = 255;
    uint8_t index = 0;
    int32_t value;
    uint8_t walls[4];
    uint8_t not_visited;
    uint8_t x;
    uint8_t y;

    for (uint8_t i = 0; i < MAP_SIDE; i++) {
        for (uint8_t j = 0; j < MAP_SIDE; j++){
            not_visited = v[i][j];
/*            read_wall(i, j, walls);

       
            if (walls[0] == 0) {    //north
                x = j;
                y = i - 1;
                if (a[y][x] == 255){
//                    not_visited = 1;
                }
            }
            if (walls[1] == 0) {    //east
                x = j + 1;
                y = i;
                if (a[y][x] == 255){
//                    not_visited = 1;
                }
            }
            if (walls[2] == 0) {    //south
                x = j;
                y = i + 1;
                if (a[y][x] == 255){
//                    not_visited = 1;
                }
            }
            if (walls[3] == 0) {    //west
                x = j - 1;
                y = i;
                if (a[y][x] == 255){
                    not_visited = 1;
                }
            }
*/
            value = a[i][j];
            uint8_t check = value < minimum; 
            if (check && not_visited) {
                    minimum = value;
                    index = i*MAP_SIDE + j;
                    printf("Not visited: %u check: %u \n", not_visited, check);
            }            
        }
    }
    

    return index;
}

uint8_t is_in_route(uint8_t x, uint8_t y){
    uint8_t i = 0;
    uint8_t ret = 0;
    while (1)
    {
        uint8_t row = route[i][0];
        uint8_t column = route[i][1];
        if(row == y && column == x){
            ret = 1;
            break;
        }
        if(row == 255 || i == MAP_SIDE*MAP_SIDE){
            break;
        }
        i++;
    }
    return ret;
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
