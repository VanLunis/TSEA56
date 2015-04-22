/*
 * Markus_Albin_hackerzZ_1337.c
 *
 * Created: 4/17/2015 9:37:19 AM
 *  Author: marpe163
 */ 


#include <avr/io.h>
int explored_map[17][17]; //17 stor f�r att kunna ta h�nsyn till banans periferi.
int driveable_map[17][17];
struct coordinate_struct
{
	
	uint8_t start_pos_x;
	uint8_t start_pos_y;
	uint8_t goal_pos_x;
	uint8_t goal_pos_y;	
	uint8_t goal_found; //1 d� m�let hittat. Annars 0.
};
struct robot
{
	uint8_t robot_pos_x;
	uint8_t robot_pos_y;
	uint8_t robot_direction; //0 representerar rakt fram fr�n robotens ing�ng i labyrinten. 1 representerar 90 grader till h�ger, 2 ytterligare 90 och 3 ytterligare 90 grader.
	//	 En sv�ng till h�ger p�verkar riktningen genom
	// (robot_direction + 1) % 4 och en sv�ng till v�nster p�verkar riktningen genom (robot_direction + 3) % 4
	
};
struct robot my_robot;
void turned_left() //Uppdatera robotens riktning vid sv�ng.
{
	my_robot.robot_direction=(my_robot.robot_direction + 3) % 4;
}
void turned_right()
{
	my_robot.robot_direction=(my_robot.robot_direction + 1) % 4;
}
							
struct coordinate_struct my_struct;
void init()
{
	for(int i=0; i < 17; i ++)
	{
		for(int j=0;j < 17;j++)
		{
			explored_map[i][j]=0;
			driveable_map[i][j]=1;
		}
	}
}


void update_map()//ska k�ras d� roboten k�rt till mitten av en ny ruta.
{
	if((my_robot.robot_direction % 2)==0)//k�r vertikalt.
	{
		if(get_right_distance() < 20*5) //h�gersidan oframkomlig.
		{
			driveable_map[coordinate_struct.robot_pos_x + 1 - my_robot.robot_direction][coordinate_struct.robot_pos_y]=0;
			explored_map[coordinate_struct.robot_pos_x + 1 - my_robot.robot_direction][coordinate_struct.robot_pos_y]=1;
		}
		else
		{
			driveable_map[coordinate_struct.robot_pos_x + 1 - my_robot.robot_direction][coordinate_struct.robot_pos_y]=1;//d� vi eventuellt kan komma att �ndra defaultv�rdet i matrisen till 2.
		}
		if(get_left_distance() < 20*5) //v�nstersidan oframkomlig.
		{
			driveable_map[my_robot.robot_pos_x - 1 + robot.robot_direction][my_robot.robot_pos_y]=0;
			explored_map[my_robot.robot_pos_x - 1 + robot.robot_direction][my_robot.robot_pos_y]=1;
		}
		else
		{
			driveable_map[my_robot.robot_pos_x+1][my_robot.robot_pos_y]=1;//d� vi eventuellt kan komma att �ndra defaultv�rdet i matrisen till 2.
		}
	}
	else //k�r horisontellt.
	{
			if(get_right_distance() < 20*5) //h�gersidan oframkomlig.
			{
				driveable_map[my_robot.robot_pos_x][my_robot.robot_pos_y + 1 - 2*(my_robot.robot_direction % 3)]=0;
				explored_map[my_robot.robot_pos_x][my_robot.robot_pos_y+ 1 - 2*(my_robot.robot_direction % 3)]=1;
			}
			else
			{
				driveable_map[my_robot.robot_pos_x][my_robot.robot_pos_y + 1 - 2*(my_robot.robot_direction % 3)]=1;//d� vi eventuellt kan komma att �ndra defaultv�rdet i matrisen till 2.
			}
			if(get_left_distance() < 20*5) //v�nstersidan oframkomlig.
			{
				driveable_map[my_robot.robot_pos_x][my_robot.robot_pos_y - 1 + 2*(my_robot.robot_direction % 3)]=0;
				explored_map[my_robot.robot_pos_x][my_robot.robot_pos_y - 1 + 2*(my_robot.robot_direction % 3)]=1;
			}
			else
			{
				driveable_map[my_robot.robot_pos_x][my_robot.robot_pos_y - 1 + 2*(my_robot.robot_direction % 3)]=1;//d� vi eventuellt kan komma att �ndra defaultv�rdet i matrisen till 2.
			}
		
	}
}

int main(void)
{
	init();
    while(1)
    {
        //TODO:: Please write your application code 
    }
}