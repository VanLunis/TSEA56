#include "shortest_path.h"
#include "map.h" 

int8_t c = 0;
int8_t un = 0;

int8_t tmplx;
int8_t tmply;
int8_t tmprx;
int8_t tmpry;

//If traceBack have been used this function creates a set of commands
//that the robot can interpret to actually travel the path. 
//This is done by checking in what direction the next point in the path is
//The robots oriantation is also used.
void getCommands(point end){
	int8_t tmpxdir = xdir;
	int8_t tmpydir = ydir;
	c = 0;

	
    for (int i=0; i < costmap[end.x][end.y]; i++)
    {   
			tmplx = path[i].x - tmpydir;
			tmply = path[i].y + tmpxdir;
			tmprx = path[i].x + tmpydir;
			tmpry = path[i].y - tmpxdir;
			
			//GO EAST 
            if(path[i].x < path[i+1].x)
            {

                if(tmpxdir == 0 && tmpydir == 1){
                    command[c] = 'r';
                    c++;
                }
                else if(tmpxdir == 0 && tmpydir == -1){
                    command[c] = 'l';
                    c++;
                }
				else if(tmpxdir == -1 && tmpydir == 0)
				{
					//Turn back
					command[c] = 'b';
					c++;
				}
				else if(tmpxdir == 1 && tmpydir == 0 && !((driveable[tmplx][tmply] == 0) && (driveable[tmprx][tmpry] == 0)))// In decision node?
				{
					command[c] = 'f';
					c++;
				}
				tmpxdir = 1;
				tmpydir = 0;    
            }
			
			//GO WEST
            else if(path[i].x > path[i+1].x)
            {
                if(tmpxdir == 0 && tmpydir == 1){
                    command[c] = 'l';
                    c++;
                }
                else if(tmpxdir == 0 && tmpydir == -1){
                    command[c] = 'r';
                    c++;
                }
				else if(tmpxdir == 1 && tmpydir == 0)
				{
					//Turn back
					command[c] = 'b';
					c++;
				}
				if(tmpxdir == -1 && tmpydir == 0 && !((driveable[tmplx][tmply] == 0) && (driveable[tmprx][tmpry] == 0)))// In decision node?
				{
					command[c] = 'f';
					c++;
				}
				tmpxdir = -1;
				tmpydir = 0;			
            }
			
			//GO NORTH
            else if(path[i].y < path[i+1].y)
            {
                if(tmpxdir == 1 && tmpydir == 0){
                    command[c] = 'l';
                    c++;
                }
                else if(tmpxdir == -1 && tmpydir == 0){
                    command[c] = 'r';
                    c++;
                }
				else if(tmpxdir == 0 && tmpydir == -1)
				{
					//Turn back
					command[c] = 'b';
					c++;
				}
				if(tmpxdir == 0 && tmpydir == 1 && !((driveable[tmplx][tmply] == 0) && (driveable[tmprx][tmpry] == 0)))// In decision node?
				{
					command[c] = 'f';
					c++;
				}
				tmpxdir = 0;
				tmpydir = 1;
            }
			
			//GO SOUTH
            else if(path[i].y > path[i+1].y)
            {
                if(tmpxdir == 1 && tmpydir == 0){
                    command[c] = 'r';
                    c++;

                }
                else if(tmpxdir == -1 && tmpydir == 0){
                    command[c] = 'l';
                    c++;

                }
				else if(tmpxdir == 0 && tmpydir == 1)
				{
					//Turn back
					command[c] = 'b';
					c++;
				}
				else if(tmpxdir == 0 && tmpydir == -1 && !((driveable[tmplx][tmply] == 0) && (driveable[tmprx][tmpry] == 0)))// In decision node?
				{
					command[c] = 'f';
					c++;
				}
				tmpxdir = 0;
				tmpydir = -1;
        }
    }
}


//Returns a path from the start to the end by walking from the end
//Through squares such that the cost between all elements in the path is one
void traceBack(int8_t costmap[17][17], point end)
{
    //Distance is equal to cost
    int8_t cost = costmap[end.x][end.y];
    point p = end;
    path[cost] = p;

    while(cost>0){
        point temp;
        //WEST
        temp.y = p.y;
        if (costmap[p.x-1][p.y] == cost-1){
            temp.x = p.x-1;
            p = temp;
            path[cost-1] = temp;
            cost--;
        }

        //EAST
        else if (costmap[p.x+1][p.y] == cost-1){
            temp.x = p.x+1;
            p = temp;
            path[cost-1] = temp;
            cost--;
        }


        //

        else if (costmap[p.x][p.y-1] == cost-1){
            temp.x = p.x;
            temp.y = p.y - 1;
            p = temp;
            path[cost-1] = temp;
            cost--;
        }

        //NORTH
        else if (costmap[p.x][p.y+1] == cost-1){
            temp.x = p.x;
            temp.y = p.y + 1;
            p = temp;
            path[cost-1] = temp;
            cost--;
        }
    }
    return;
}


//Fills the map with the cost associated with travelling 
//from the start point, until the end point is covered.
void floodfill(point start, point end)
{
    for (int i=0; i<17;i++){
        for (int j=0; j<17;j++){
            costmap[i][j] = 255;
        }
    }
    int8_t notfinished = 1;
    point c = start;
    int8_t q = 0;
    int8_t cost = 0;
    int8_t i = 0;
    point queue[50];
    point temp;
    for (int k = 0; k < 50; k++){
        temp.x = 0;
        temp.y = 0;
        queue[k] = temp;

    }
	//Returns the cost for the point p
	//by checking the cost if it's neighbors
    int8_t getcost(point p){
        uint8_t lcost = 255;
        //WEST
        if (p.x > 0){
            if (costmap[p.x-1][p.y] < lcost){
                lcost = costmap[p.x-1][p.y]+1;
            }
        }

        //EAST
        if (p.x < 16){
            if (costmap[p.x+1][p.y] < lcost){
                lcost = costmap[p.x+1][p.y]+1;
            }
        }

        //SOUTH
        if (p.y > 0){
            if (costmap[p.x][p.y-1] < lcost){
                lcost = costmap[p.x][p.y-1]+1;
            }
        }


        //NORTH
        if (p.y < 16){
            if (costmap[p.x][p.y+1] < lcost){
                lcost = costmap[p.x][p.y+1]+1;
            }
        }
        return lcost;
    }
	//Sets the cost of a square and adds elements
	//to the queue if we are not finished
    void fill_square(point p, int8_t cost)
    {
        point temp;

        if (p.x == end.x && p.y == end.y){
            notfinished = 0;

            costmap[p.x][p.y] = cost;
            c = p;
            return;
        }
        else
        {
            costmap[p.x][p.y] = cost;
            temp.y = p.y;

            //WEST
            if (p.x > 0){
                if ((driveable[p.x-1][p.y] == 1)
                        && (costmap[p.x-1][p.y] > cost+1)){
                    temp.x = p.x - 1;
                    queue[q] = temp;
                    q++;
                }
            }

            //EAST
            if (p.x < 16){
                if ((driveable[p.x+1][p.y] == 1)
                        && (costmap[p.x+1][p.y] > cost+1)){
                    temp.x = p.x+1;
                    queue[q] = temp;
                    q++;

                }
            }

            //SOUTH
            temp.x = p.x;
            if (p.y > 0){
                if ((driveable[p.x][p.y-1] == 1)
                        && (costmap[p.x][p.y-1] > cost+1)){
                    temp.y = p.y - 1;
                    queue[q] = temp;
                    q++;

                }
            }


            //NORTH
            if (p.y < 16){
                if ((driveable[p.x][p.y+1] == 1)
                        && (costmap[p.x][p.y+1] > cost+1)){
                    temp.y = p.y + 1;
                    queue[q] = temp;
                    q++;

                }
            }
            return;
        }

    }



    if (driveable[start.x][start.y] == 0){
        return;
    }

	//the main loop of the algorithm 
	//Fills squares with costs until we find the endpoint
    fill_square(c, cost);
    while (notfinished){
        c = queue[i];
        fill_square(c, getcost(c));
        i++;
    }
    return;

}

