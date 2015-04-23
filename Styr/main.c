#include <stdio.h>
#include<stdlib.h>
#include <stdint.h>
#include "grid.h"
#include "robot_movement.h"
#include "dijkstras.h"
GRID explored_map; //17 stor för att kunna ta hänsyn till banans periferi.
GRID driveable_map;
GRID hardcoded_map;
NODE * current_node = NULL;
struct coordinate_struct my_coordinates;
struct robot my_robot;
void turn_left() //Uppdatera robotens riktning vid sväng.
{
    my_robot.robot_direction = (my_robot.robot_direction + 3) % 4;
}
void turn_right()
{
    my_robot.robot_direction = (my_robot.robot_direction + 1) % 4;
}
void go_forward() //Uppdaterar robotens position vid förflyttning
{
    if((my_robot.robot_direction % 2)==0)//kör vertikalt.
    {
        if(my_robot.robot_pos_y==GRID_SIZE - 2 && my_robot.robot_direction==0)
        {
            shift_down(&driveable_map, &explored_map, &my_coordinates);
            return;
        }
        else if(my_robot.robot_pos_y==1 && my_robot.robot_direction==2)
        {
            shift_up(&driveable_map, &explored_map, &my_coordinates);
            return;
        }
        my_robot.robot_pos_y = (my_robot.robot_pos_y  + 1 - my_robot.robot_direction);
    }
    else
    {
        if(my_robot.robot_pos_x==GRID_SIZE - 2 && my_robot.robot_direction==1)
        {
            shift_left(&driveable_map, &explored_map, &my_coordinates);

            return;
        }
        else if(my_robot.robot_pos_x==1 && my_robot.robot_direction==3)
        {
            shift_right(&driveable_map, &explored_map, &my_coordinates);
            return;
        }
        my_robot.robot_pos_x = (my_robot.robot_pos_x + 2*(my_robot.robot_direction % 3) - 1);
    }
    if (my_coordinates.goal_pos_x == my_robot.robot_pos_x && my_coordinates.goal_pos_y == my_robot.robot_pos_y)
    {
        my_coordinates.goal_found = 1;
    }
}

struct coordinate_struct my_struct;
void init()
{
    /*for(int i=0; i < 17; i ++)
    {
        for(int j=0;j < 17;j++)
        {
            explored_map[i][j]=0;
            driveable_map[i][j]=1;
        }
    }*/
    my_robot.robot_pos_x=8;
    my_robot.robot_pos_y=8;
    my_coordinates.start_pos_x=my_robot.robot_pos_x;
    my_coordinates.start_pos_y=my_robot.robot_pos_y;
    my_coordinates.goal_found=0;
    my_coordinates.goal_pos_x=14;
    my_coordinates.goal_pos_y=9;
    explored_map.grid_matrix[my_robot.robot_pos_x][my_robot.robot_pos_y]=1;
    driveable_map.grid_matrix[my_robot.robot_pos_x][my_robot.robot_pos_y]=1;
	NODE start_node;
	current_node = & start_node;
	hardcoded_map.grid_matrix[8][8] = 200;
	hardcoded_map.grid_matrix[8][9] = 200;
	hardcoded_map.grid_matrix[9][9] = 200;
	hardcoded_map.grid_matrix[10][9] = 200;
	hardcoded_map.grid_matrix[11][9] = 200;
	hardcoded_map.grid_matrix[12][9] = 200;
	hardcoded_map.grid_matrix[13][9] = 200;
	hardcoded_map.grid_matrix[14][9] = 200;
	hardcoded_map.grid_matrix[14][9] = 200; // goal!
}

void update_robot_position()
{
  /*  if (commando_drive_forward = 1) // uppdatera positionen för roboten
    {
        if((my_robot.robot_direction % 2)==0)//kör vertikalt.
        {
            robot.robot_pos_y =  * (get_driven_square())
        }
        else
        {
            (get_driven_squares())
        }
    }*/
    set_element(my_robot.robot_pos_x,my_robot.robot_pos_y, 1, &explored_map);
}


//////////////////////--------- Shortest path---------------/////////////////
//Matrisrepresentation för korstaste vägen
void find_shortest_path()
{

}

//// Styrbeslut för kortaste vägen
void commandos_for_shortest_path()
{
    // vid nästa beslutspunkt så ta rakt/höger/vänster
}

//////////////////////-------------------------------------/////////////////



int main(void)
{
	grid_init(0,&hardcoded_map); // inits hardcoded_map with zeros
    char temp_char;
    init();
    struct grid map_grid;
    grid_init(0,&explored_map); // inits explored map with zeros
    update_map(&driveable_map,&explored_map,&my_robot); // 
	print_grid(&hardcoded_map, &my_robot, &my_coordinates);
    while(1)
    {
		
        print_grid(&explored_map, &my_robot, &my_coordinates);
        print_grid(&driveable_map, &my_robot, &my_coordinates);
        temp_char=getchar();
        if(temp_char=='w')
        {
            update_robot_position();
            go_forward();
            update_map(&driveable_map,&explored_map,&my_robot);
			// uint8_t possible_directions = check_possible_directions(struct grid *driveable_map, my_robot->robot_pos_x, robot->robot_pos_y);
        }
        else if(temp_char=='a')
        {
            turn_left();
        }
        else if(temp_char=='d')
        {
            turn_right();
        }

    }
    return 0;


}

