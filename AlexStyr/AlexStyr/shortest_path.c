#include "shortest_path.h"
#include "map.h" 

int c = 0;

void getCommands(point end){
	int direction = 0;
	
	if (robot.xdir == 0 && robot.ydir == 1)
	{
		direction = 0;
	}
	else if (robot.xdir ==1 && robot.ydir == 0)
	{
		direction = 1;	
	}
	else if (robot.xdir == 0 && robot.ydir == -1)
	{
		direction = 2;
	}
	else if (robot.xdir == -1 && robot.ydir == 0)
	{
		direction = 3;
	}
	
    for (int i=0; i < costmap[end.x][end.y]; i++)
    {
           
            if(path[i].x < path[i+1].x)
            {

                if(direction==0){
                  //  printf("Turn right, ");
                    command[c] = 'r';
                    c++;

                }
                else if(direction==2){
                   // printf("Turn left, ");
                    command[c] = 'l';
                    c++;

                }
				else if(direction == 3)
				{
					//Turn back
					command[c] = 'b';
					c++;
				}
            //    printf("Go forwards.\n");
                command[c] = 'f';
                c++;
                direction = 1;

            }
            else if(path[i].x > path[i+1].x)
            {
                if(direction==0){
                //   printf("Turn left, ");
                    command[c] = 'l';
                    c++;

                }
                else if(direction==2){
             //       printf("Turn right, ");
                    command[c] = 'r';
                    c++;

                }
				else if(direction == 1)
				{
					//Turn back
					command[c] = 'b';
					c++;
				}
            //    printf("Go forwards!\n");
                direction=3;
                command[c] = 'f';
                c++;

            }
            else if(path[i].y < path[i+1].y)
            {
                if(direction==1){
                    //printf("Turn left, ");
                    command[c] = 'l';
                    c++;

                }
                else if(direction==3){
                    //printf("Turn right, ");
                    command[c] = 'r';
                    c++;

                }
				else if(direction == 2)
				{
					//Turn back
					command[c] = 'b';
					c++;
				}
             //   printf("Go forwards!\n");
                direction=0;
                command[c] = 'f';
                c++;

            }
            else if(path[i].y > path[i+1].y)
            {
                if(direction==1){
                  //  printf("Turn right, ");
                    command[c] = 'r';
                    c++;

                }
                else if(direction==3){
                   // printf("Turn left, ");
                    command[c] = 'l';
                    c++;

                }
				else if(direction == 0)
				{
					//Turn back
					command[c] = 'b';
					c++;
				}
               // printf("Go forwards!\n");
                direction=2;
                command[c] = 'f';
                c++;


        }
        //printf("Elem %i in path is: %i, %i\n", i, path[i].x, path[i].y);
    }
}

void traceBack(int costmap[17][17], point end)
{
    //Distance is equal to cost
    int cost = costmap[end.x][end.y];
    //printf("COST for end: %i", cost);
    //point path[cost];
    point p = end;
    path[cost] = p;

    while(cost>0){
        point temp;
        //WEST
        temp.y = p.y;
        if (costmap[p.x-1][p.y] == cost-1){
       //     printf("W\n");
            temp.x = p.x-1;
            p = temp;
            path[cost-1] = temp;
            cost--;
        }

        //EAST
        else if (costmap[p.x+1][p.y] == cost-1){
            temp.x = p.x+1;
            p = temp;
       //     printf("E\n");
            path[cost-1] = temp;
            cost--;
        }


        //

        else if (costmap[p.x][p.y-1] == cost-1){
            temp.x = p.x;
            temp.y = p.y - 1;
            p = temp;
            //printf("S\n");

            path[cost-1] = temp;
            cost--;
        }

        //NORTH
        else if (costmap[p.x][p.y+1] == cost-1){
            temp.x = p.x;
            temp.y = p.y + 1;
            p = temp;
            //printf("N\n");
            path[cost-1] = temp;
            cost--;
        }
    }
    return;
}

void floodfill(point start, point end)
{
    for (int i=0; i<17;i++){
        for (int j=0; j<17;j++){
            costmap[i][j] = 255;
        }
    }
    //printf(" (%i, %i), (%i, %i)", start.x, start.y, end.x, end.y);
    int notfinished = 1;
    point c = start;
    int q = 0;
    int cost = 0;
    int i = 0;
    point queue[50];
    point temp;
    for (int k = 0; k < 50; k++){
        temp.x = 0;
        temp.y = 0;
        queue[k] = temp;

    }

    int getcost(point p){
        int lcost = 255;
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
      //  printf ("COST TO THE SOUTH: %i\n", costmap[p.x][p.y-1]);
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

    void fill_square(point p, int cost)
    {
        //printf("x:%i y:%i%",p.x, p.y);
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

      //      printf("updated (%i, %i)'s cost  to %i \n ", p.x, p.y, cost);

            temp.y = p.y;

            //WEST
            //printf("C: x:%i, y:%i", c.x, c.y);
            if (p.x > 0){
                if ((driveable[p.x-1][p.y] == 1)
                        && (explored[p.x-1][p.y] == 1)
                        && (costmap[p.x-1][p.y] > cost+1)){
                    temp.x = p.x - 1;
                   // printf("WEST x:%i y:%i%",p.x, p.y);
                    queue[q] = temp;
                    q++;
                }
            }

            //EAST
            if (p.x < 16){
                if ((driveable[p.x+1][p.y] == 1)
                        && (explored[p.x+1][p.y] == 1)
                        && (costmap[p.x+1][p.y] > cost+1)){
                   // printf("EAST x:%i y:%i%",p.x, p.y);
                    temp.x = p.x+1;
                    queue[q] = temp;
                    q++;

                }
            }

            //SOUTH
            temp.x = p.x;
            if (p.y > 0){
                if ((driveable[p.x][p.y-1] == 1)
                        && (explored[p.x][p.y-1] == 1)
                        && (costmap[p.x][p.y-1] > cost+1)){
                    temp.y = p.y - 1;
                //    printf("SOUTH x:%i y:%i%",p.x, p.y);
                    queue[q] = temp;
                    q++;

                }
            }


            //NORTH
     //       printf("NORTH y+1: d:%i e:%i cm:% c:%i i:%i\n", driveable[p.x][p.y+1], explored[p.x][p.y+1], costmap[p.x][p.y+1], cost, i);

            if (p.y < 16){
                if ((driveable[p.x][p.y+1] == 1)
                        && (explored[p.x][p.y+1] == 1)
                        && (costmap[p.x][p.y+1] > cost+1)){
               //     printf("NORTH - SURPRISE MF x:%i y:%i%",p.x, p.y);
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


    fill_square(c, cost);
    while (notfinished){
 //       printf("F SQUARE: i:%i x:%i y:%i, cx:%i, cy:%i \n%", i, queue[i].x, queue[i].y, c.x, c.y);
        c = queue[i];
        fill_square(c, getcost(c));
        i++;
    }

    //path[cost] = traceBack();

    return;

}
