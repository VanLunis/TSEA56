#ifndef GRID_H_INCLUDED
#define GRID_H_INCLUDED
#define GRID_SIZE 17


/*
 _________ structs ______________
 
 */
typedef struct robot
{
    uint8_t robot_pos_x;
    uint8_t robot_pos_y;
    uint8_t robot_direction; //0 representerar rakt fram från robotens ingång i labyrinten. 1 representerar 90 grader till höger, 2 ytterligare 90 och 3 ytterligare 90 grader.
    //	En sväng till höger påverkar riktningen genom
    // (robot_direction + 1) % 4 och en sväng till vänster påverkar riktningen genom (robot_direction + 3) % 4
} ROBOT;

struct coordinate_struct
{
    
    uint8_t start_pos_x;
    uint8_t start_pos_y;
    uint8_t goal_pos_x;
    uint8_t goal_pos_y;
    uint8_t goal_found; //1 då målet hittat. Annars 0.
};

typedef struct grid
{
    uint8_t grid_matrix[GRID_SIZE][GRID_SIZE];
} GRID;

/*
 ______________ functions _________________
 */

void grid_init(int val, struct grid *my_grid)
{
    for(int i = 0; i < GRID_SIZE;i++)
    {
        for(int j = 0; j < GRID_SIZE;j++)
        {
            my_grid->grid_matrix[i][j]=val;
        }
    }
}

void set_element(int x, int y, int val,struct grid *my_grid)
{
    my_grid->grid_matrix[x][y]=val;
}
void find_goal(int x, int y, struct coordinate_struct *coordinates)
{
    coordinates->goal_pos_x=x;
    coordinates->goal_pos_y=y;
    coordinates->goal_found=1;
}

void shift_right(struct grid *drivable, struct grid *explored, struct coordinate_struct *coordinates)
{
    //skiftar drivable
    for(int i=0;i < GRID_SIZE;i++)
    {
        for(int j=0; j < GRID_SIZE - 1;j++)
        {
            drivable->grid_matrix[GRID_SIZE- 1 - j][i]=drivable->grid_matrix[GRID_SIZE-j - 2][i];
        }
    }
    for(int i=0;i<GRID_SIZE;i++)
    {
        drivable->grid_matrix[0][i]=0;
    }
    
    //skiftar explored
    for(int i=0;i < GRID_SIZE;i++)
    {
        for(int j=0; j < GRID_SIZE - 1;j++)
        {
            explored->grid_matrix[GRID_SIZE- 1 - j][i]=explored->grid_matrix[GRID_SIZE-j - 2][i];
        }
    }
    for(int i=0;i<GRID_SIZE;i++)
    {
        explored->grid_matrix[0][i]=0;
    }
    coordinates->start_pos_x=coordinates->start_pos_x + 1;
    coordinates->goal_pos_x=coordinates->goal_pos_x + 1;
}
void shift_left(struct grid *drivable, struct grid *explored, struct coordinate_struct *coordinates)
{
    //skiftar drivable
    for(int i=0;i < GRID_SIZE;i++)
    {
        for(int j=0; j < GRID_SIZE - 1;j++)
        {
            drivable->grid_matrix[j][i]=drivable->grid_matrix[j + 1][i];
        }
    }
    for(int i=0;i<GRID_SIZE;i++)
    {
        drivable->grid_matrix[GRID_SIZE-1][i]=0;
    }
    //skiftar explored
    for(int i=0;i < GRID_SIZE;i++)
    {
        for(int j=0; j < GRID_SIZE - 1;j++)
        {
            explored->grid_matrix[j][i]=explored->grid_matrix[j + 1][i];
        }
    }
    for(int i=0;i<GRID_SIZE;i++)
    {
        explored->grid_matrix[GRID_SIZE-1][i]=0;
    }
    coordinates->start_pos_x=coordinates->start_pos_x - 1;
    coordinates->goal_pos_x=coordinates->goal_pos_x - 1;
}
void shift_up(struct grid *drivable, struct grid *explored, struct coordinate_struct *coordinates)
{
    //skiftar drivable
    for(int i=0;i < GRID_SIZE;i++)
    {
        for(int j=0; j < GRID_SIZE - 1;j++)
        {
            drivable->grid_matrix[i][GRID_SIZE-1-j]=drivable->grid_matrix[i][GRID_SIZE-j - 2];
        }
    }
    for(int i=0;i<GRID_SIZE;i++)
    {
        drivable->grid_matrix[i][0]=0;
    }
    //skiftar explored
    for(int i=0;i < GRID_SIZE;i++)
    {
        for(int j=0; j < GRID_SIZE - 1;j++)
        {
            explored->grid_matrix[i][GRID_SIZE-1-j]=explored->grid_matrix[i][GRID_SIZE-j - 2];
        }
    }
    for(int i=0;i<GRID_SIZE;i++)
    {
        explored->grid_matrix[i][0]=0;
    }
    coordinates->start_pos_y=coordinates->start_pos_y + 1;
    coordinates->goal_pos_y=coordinates->goal_pos_y + 1;
}
void shift_down(struct grid *drivable, struct grid *explored, struct coordinate_struct *coordinates)
{
    //skiftar drivable
    for(int i=0;i < GRID_SIZE;i++)
    {
        for(int j=0; j < GRID_SIZE - 1;j++)
        {
            drivable->grid_matrix[i][j]=drivable->grid_matrix[i][j+1];
        }
    }
    for(int i=0;i<GRID_SIZE;i++)
    {
        drivable->grid_matrix[i][GRID_SIZE - 1]=0;
    }
    //skiftar explored
    for(int i=0;i < GRID_SIZE;i++)
    {
        for(int j=0; j < GRID_SIZE - 1;j++)
        {
            explored->grid_matrix[i][j]=explored->grid_matrix[i][j+1];
        }
    }
    for(int i=0;i<GRID_SIZE;i++)
    {
        explored->grid_matrix[i][GRID_SIZE - 1]=0;
    }
    coordinates->start_pos_y=coordinates->start_pos_y - 1;
    coordinates->goal_pos_y=coordinates->goal_pos_y - 1;
}


void print_grid(struct grid *my_grid, struct robot *my_robot, struct coordinate_struct *coordinates)
{
    printf("%u", my_robot->robot_pos_x);
    printf("%u", my_robot->robot_pos_y);
    printf("\n\n");
    for(int i=0;i<GRID_SIZE;i++)
    {
        for(int j=0;j<GRID_SIZE;j++)
        {   if((int)my_robot->robot_pos_x==j && GRID_SIZE - 1 - (int)my_robot->robot_pos_y==i)
        {
            printf("X");
        }
        else if((int) coordinates->start_pos_x == j&& GRID_SIZE - 1 - (int)coordinates->start_pos_y==i)
        {
            printf("S");
        }
        else if((int) coordinates->goal_pos_x == j&& GRID_SIZE - 1 - (int)coordinates->goal_pos_y==i && coordinates->goal_found==1)
        {
            printf("G");
        }
        else
        {
            printf("%u", (my_grid->grid_matrix[j][GRID_SIZE - 1 - i]));
        }
            
        }
        printf("\n");
    }
}





/*
 uint8_t * check_decisionPoint(uint8_t x, uint8_t y, struct grid *driveable){
	uint8_t is_decisionPoint = 0;
	
	is_decisionPoint = driveable->grid_matrix[x+1][y] + driveable->grid_matrix[x-1][y] + driveable->grid_matrix[x][y+1] + driveable->grid_matrix[x][y-1];
	if ((is_decisionPoint == 3 || is_decisionPoint == 4)&&driveable->grid_matrix[x][y]==1)
	{
 return 1;
	else
	{
 return 0;
	}
	
 }
 
 */
#endif // GRID_H_INCLUDED
