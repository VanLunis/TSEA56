#include "map.h"

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
    start[1]++;
    goal[1]++;
    robotpos[1]++;
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
    start[0]++;
    goal[0]++;
    robotpos[0]++;
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
    start[1]--;
    goal[1]--;
    robotpos[1]--;
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
    start[0]--;
    goal[0]--;
    robotpos[0]--;
    return;

}

void init_map()
{
	robot.x = 8;
	robot.y = 8;
	robot.xdir = 0;
	robot.ydir = 1;
    for (int i=0; i<MAP_SIZE; i++)
    {
        for (int j=0; j<MAP_SIZE; j++)
        {
            driveable[i][j] = 1;
            explored[i][j] = 0;
        }
    }
}

void update_map()
{
	robot.x+=robot.xdir;
	robot.y+=robot.ydir;
    //lx is x-coord for cell left or robot, ly is y-coord
    int lx = robot.x-robot.ydir;
    int ly = robot.y+robot.xdir;

    //analog to lx/ly for right cell
    int rx = robot.x+robot.ydir;
    int ry = robot.y-robot.xdir;

    //forward cell
    int fx = robot.x+robot.ydir;
    int fy = robot.y+robot.xdir;

    if (robotpos[1] == MAP_SIZE - 2)
    {
        shift_down();
    }

    if (robotpos[0] == MAP_SIZE - 2)
    {
        shift_left();
    }

    if (robotpos[1] == 1)
    {
        shift_up();
    }

    if (robotpos[0] == 1)
    {
        shift_right();
    }
	
	if (robot.lwall){
		driveable[lx][ly] = 0;
	}
	if (robot.fwall){
		driveable[fx][fy] = 0;
	}
	if (robot.rwall){
		driveable[rx][ry] = 0;
	}
	explored[robot.x][robot.y] = 1;
	explored[lx][ly] = 1;
	explored[fx][fy] = 1;
	explored[rx][ry] = 1;
	

    /*
    if (robot.direction == 'n')




    {

        robotpos[1]++;
        if (robot.lwall)
        {
            driveable[robotpos[0]-1][robotpos[1]] = 1;
        }
        else
        {
            driveable[robotpos[0]-1][robotpos[1]] = 0;
        }
        if (robot.fwall)
        {
            driveable[robotpos[0]][robotpos[1]+1] = 1;
        }
        else
        {
            driveable[robotpos[0]][robotpos[1]+1] = 0;
        }
        if (robot.rwall)
        {
            driveable[robotpos[0]+1][robotpos[1]] = 1;
        }
        else
        {
            driveable[robotpos[0]+1][robotpos[1]] = 0;
        }
        if (robot.bwall)
        {
            driveable[robotpos[0]][robotpos[1]-1] = 1;
        }
        else
        {
            driveable[robotpos[0]][robotpos[1]-1] = 0;
        }

        explored[robotpos[0]][robotpos[1]+1] = 1;


    }
    else if (robot.direction == 'e')
    {

        if (robot.lwall)
        {
            driveable[robotpos[0]][robotpos[1]+1] = 1;
        }
        else
        {
            driveable[robotpos[0]][robotpos[1]+1] = 0;
        }
        if (robot.fwall)
        {
            driveable[robotpos[0]+1][robotpos[1]] = 1;
        }
        else
        {
            driveable[robotpos[0]+1][robotpos[1]] = 0;
        }
        if (robot.rwall)
        {
            driveable[robotpos[0]][robotpos[1]-1] = 1;
        }
        else
        {
            driveable[robotpos[0]][robotpos[1]-1] = 0;
        }
        if (robot.bwall)
        {
            driveable[robotpos[0]-1][robotpos[1]-1] = 1;
        }
        else
        {
            driveable[robotpos[0]-1][robotpos[1]-1] = 0;
        }
    }
    else if (robot.direction == 's')
    {

        robotpos[1]--;
        if (robot.lwall)
        {
            driveable[robotpos[0]+1][robotpos[1]] = 1;
        }
        else
        {
            driveable[robotpos[0]+1][robotpos[1]] = 0;
        }
        if (robot.fwall)
        {
            driveable[robotpos[0]][robotpos[1]-1] = 1;
        }
        else
        {
            driveable[robotpos[0]][robotpos[1]-1] = 0;
        }
        if (robot.rwall)
        {
            driveable[robotpos[0]-1][robotpos[1]] = 1;
        }
        else
        {
            driveable[robotpos[0]-1][robotpos[1]] = 0;
        }
        if (robot.bwall)
        {
            driveable[robotpos[0]][robotpos[1]+1] = 1;
        }
        else
        {
            driveable[robotpos[0]][robotpos[1]+1] = 0;
        }
    }
    else if (robot.direction == 'w')
    {

        robotpos[0]--;
        if (robot.lwall)
        {
            driveable[robotpos[0]][robotpos[1]-1] = 1;
        }
        else
        {
            driveable[robotpos[0]][robotpos[1]-1] = 0;
        }
        if (robot.fwall)
        {
            driveable[robotpos[0]-1][robotpos[1]] = 1;
        }
        else
        {
            driveable[robotpos[0]-1][robotpos[1]] = 0;
        }
        if (robot.rwall)
        {
            driveable[robotpos[0]][robotpos[1]+1] = 1;
        }
        else
        {
            driveable[robotpos[0]][robotpos[1]+1] = 0;
        }
        if (robot.bwall)
        {
            driveable[robotpos[0]+1][robotpos[1]] = 1;
        }
        else
        {
            driveable[robotpos[0]+1][robotpos[1]-1] = 0;
        }

    }
    */
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

