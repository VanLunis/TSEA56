#include "map.h"

// MAP variables
int8_t x = 0;
int8_t y = 0;
int8_t xdir = 0;
int8_t ydir = 1;
int goal[2] = {0,0};



//Shifts the elements up. (Keep in mind that lower left corner is the origin)
//Loops through the map from top to bottom and shifts stuff up. Replaced bottom with zeros.
void shift_up()
{
    for (int j = MAP_SIZE - 1; j > 0; j--)
    {
        for(int i=0; i<MAP_SIZE; i++)
        {
            driveable[i][j]  = driveable[i][j-1];
            explored[i][j] = explored[i][j-1];
        }
    }
    for (int j = 0; j < MAP_SIZE; j++){
        driveable[j][0] = 0;
        explored[j][0] = 0;
    }
    //start[1]++;
    goal[1]++;
    y++;
}

void shift_right(){
    for (int i = MAP_SIZE - 1; i > 0; i--)
    {
        for (int j = 0; j < MAP_SIZE; j++)
        {
            driveable[i][j] = driveable[i-1][j];
            explored[i][j] = explored[i-1][j];
        }
    }
    
    for (int j = 0; j < MAP_SIZE; j++)
    {
        driveable[0][j] = 0;
        explored[0][j] = 0;
    }
    // start[0]++;
    goal[0]++;
    x++;
}

void shift_down()
{
    for (int i = 0; i < MAP_SIZE-1; i++)
    {
        for (int j = 0; j < MAP_SIZE-1; j++)
        {
            driveable[i][j] = driveable[i][j+1];
            explored[i][j] = explored[i][j+1];
        }
    }
    for (int i = 0; i < MAP_SIZE; i++)
    {
        driveable[i][MAP_SIZE - 1] = 0;
        explored[i][MAP_SIZE - 1] = 0;
    }
    //start[1]--;
    goal[1]--;
    y--;
    return;
}

void shift_left()
{
    for (int i = 0; i < MAP_SIZE - 1; i++)
    {
        for (int j = 0; j < MAP_SIZE; j++)
        {
            driveable[i][j] = driveable[i+1][j];
            explored[i][j] = explored[i+1][j];
        }
    }
    for (int j = 0; j < MAP_SIZE; j++)
    {
        driveable[MAP_SIZE-1][j]= 0;
        explored[MAP_SIZE-1][j] = 0;
    }
    //start[0]--;
    goal[0]--;
    x--;
    return;
    
}

void init_map(){
    x = 8;
    y = 8;
    xdir = 0;
    ydir = 1;
    for (int i = 0; i < 17; i++)
    {
        for (int j = 0; j < 17; j++)
        {
            explored[i][j] = 0;
            driveable[i][j] = 0;
        }
    }
}

//updates the robots orientation, call from turn-functions
void update_orientation(char turn){
    //turn right
    if (turn == 'r'){
        if (xdir == 0  && ydir ==1)
        {
            xdir = 1;
            ydir = 0;
        }
        else if (xdir == 1 && ydir ==0)
        {
            xdir = 0;
            ydir = -1;
        }
        else if (xdir == 0 && ydir == -1)
        {
            xdir = -1;
            ydir = 0;
        }
        else if (xdir == -1 && ydir == 0){
            xdir = 0;
            ydir = 1;
        }
    }
    
    
    //LEFT TURN
    else if (turn == 'l')
    {
        if (xdir == 0  && ydir == 1)
        {
            xdir = -1;
            ydir = 0;
        }
        else if (xdir == 1 && ydir == 0)
        {
            xdir = 0;
            ydir = 1;
        }
        else if (xdir == 0 && ydir == -1)
        {
            xdir = 1;
            ydir = 0;
        }
        else if (xdir == -1 && ydir == 0){
            xdir = 0;
            ydir = -1;
        }
    }
    
    //TURN BACK
    else if (turn == 'b')
    {
        ydir = -ydir;
        xdir = -xdir;
    }
    
}

void update_position(){
    x = x + xdir;
    y = y + ydir;
}

void update_map(){
    int lx = x-ydir;
    int ly = y+xdir;
    
    //analog to lx/ly for right cell
    int rx = x+ydir;
    int ry = y-xdir;
    
    //forward cell
    int fx = x+xdir;
    int fy = y+ydir;
    
    
    
    // added map shift function. PLACED WRONG?
    if (y == MAP_SIZE - 2)
    {
        shift_down();
    }
    
    if (x == MAP_SIZE - 2)
    {
        shift_left();
    }
    
    if (y == 1)
    {
        shift_up();
    }
    
    if (x == 1)
    {
        shift_right();
    }
    
    
    
    
    // END OF MAP SHIFT
    
    explored[lx][ly] = 1;
    explored[fx][fy] = 1;
    explored[rx][ry] = 1;
    explored[x][y] = 1;
    driveable[x][y] = 1;
    
    
}
/*
 void print_driveable()
 {
 printf("Driveable: \n");
 for (int j = MAP_SIZE-1; j >= 0; j--)
 {
 for (int i=0; i<MAP_SIZE; i++)
 {
 printf("%2d",  "%i", driveable[i][j]);
 }
 printf("\n");
 }
 
 }
 */
/*
 void print_explored()
 {
 printf("Explored: \n");
 for (int j = MAP_SIZE-1; j >= 0; j--)
 {
 for (int i=0; i<MAP_SIZE; i++)
 {
 printf("%2d", "%i", explored[i][j]);
 }
 printf("\n");
 }
 
 
 */
/*
 void print_costmap()
 {
 printf("Costmap:\n");
 for (int j = MAP_SIZE-1; j >= 0; j--)
 {
 for (int i=0; i<MAP_SIZE; i++)
 {
 if (costmap[i][j] == 255){
 printf("##");
 }
 else{
 printf("%2d", costmap[i][j]);
 }
 }
 printf("\n");
 }
 
 }
 */

