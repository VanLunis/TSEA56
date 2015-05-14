#include "map.h"

// MAP variables
int8_t x = 0;
int8_t y = 0;
int8_t xdir = 0;
int8_t ydir = 1;
int8_t goalx = 140;
int8_t goaly = 140;
int8_t fwall = 0;
int8_t rwall = 0;
int8_t lwall = 0;

unsigned char in_turn = 0;


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
    goaly++;
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
    goalx++;
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
    goaly--;
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
    for (int j =	0; j < MAP_SIZE; j++)
    {
        driveable[MAP_SIZE-1][j]= 0;
        explored[MAP_SIZE-1][j] = 0;
    }
    //start[0]--;
    goalx--;
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
	explored[8][8] = 1;
	driveable[8][8] = 1;
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

void update_map()
{
    
	// added map shift function. PLACED WRONG?
	if (y == MAP_SIZE - 1)
    {
        shift_down();
    }
    
    if (x == MAP_SIZE - 1)
    {
        shift_left();
    }
    
    if (y == 0)
    {
        shift_up();
    }
    
    if (x == 0)
    {
        shift_right();
    }
    
    // END OF MAP SHIFT
    
	int8_t lx = x-ydir;
	int8_t ly = y+xdir;
	
	//analog to lx/ly for right cell
	int8_t rx = x+ydir;
	int8_t ry = y-xdir;
	
	//forward cell
	int8_t fx = x+xdir;
	int8_t fy = y+ydir;
	
    driveable[x][y] = 1;
	explored[x][y] = 1;
	
	if(in_turn)
	{
		if (!lwall)
		{
			driveable[lx][ly] = 1;
		}
		if (!fwall)
		{
			driveable[fx][fy] = 1;
		}
		if (!rwall)
		{
			driveable[rx][ry] = 1;
		}		
	}

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


