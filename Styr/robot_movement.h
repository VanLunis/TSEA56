#ifndef ROBOT_MOVEMENT_H_INCLUDED
#define ROBOT_MOVEMENT_H_INCLUDED

int get_left_distance()
{
    return 200;
}
int get_right_distance()
{
    return 200;
}
int get_front_distance()
{
    return 200;
}

void update_map(struct grid *driveable, struct grid *explored, struct robot *our_robot)//ska k�ras d� roboten k�rt till mitten av en ny ruta.
{
    if((our_robot->robot_direction % 2)==0)//k�r vertikalt.
    {
        if(get_right_distance() < 20*5) //h�gersidan oframkomlig.
        {
            driveable->grid_matrix[our_robot->robot_pos_x + 1 - our_robot->robot_direction][our_robot->robot_pos_y]=0;
            explored->grid_matrix[our_robot->robot_pos_x + 1 - our_robot->robot_direction][our_robot->robot_pos_y]=1;
        }
        else // no wall
        {
            driveable->grid_matrix[our_robot->robot_pos_x + 1 - our_robot->robot_direction][our_robot->robot_pos_y]=1;//d� vi eventuellt kan komma att �ndra defaultv�rdet i matrisen till 2.
            // l�gga in att vi har utforskat rutan? Eller sk�ts n�r man k�r rakt fram?
        }
        if(get_left_distance() < 20*5) //v�nstersidan oframkomlig.
        {
            driveable->grid_matrix[our_robot->robot_pos_x - 1 + our_robot->robot_direction][our_robot->robot_pos_y]=0;
            explored->grid_matrix[our_robot->robot_pos_x - 1 + our_robot->robot_direction][our_robot->robot_pos_y]=1;
        }
        else
        {
            driveable->grid_matrix[our_robot->robot_pos_x -1 + our_robot->robot_direction][our_robot->robot_pos_y]=1;//d� vi eventuellt kan komma att �ndra defaultv�rdet i matrisen till 2.
            // L�gga in att vi har utforskat rutan? Eller sk�ts det n�r man �ker rakt fram?
        }


        if(get_front_distance() < 20*5) // fronten oframkomlig.
        {
            driveable->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y + 1 - our_robot->robot_direction]=0;
            explored->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y + 1 - our_robot->robot_direction]=1;
        }
        else
        {
            driveable->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y + 1 - our_robot->robot_direction]=1;
            // explored_map[my_robot.robot_pos_x][my_robot.robot_pos_y + 1 - my_robot.robot_direction]=1;

        }
    }
    else //k�r horisontellt.
    {
        if(get_right_distance() < 20*5) //h�gersidan oframkomlig.
        {
            driveable->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y + 1 - 2*(our_robot->robot_direction % 3)]=0;
            explored->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y + 1 - 2*(our_robot->robot_direction % 3)]=1;
        }
        else
        {
            driveable->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y + 1 - 2*(our_robot->robot_direction % 3)]=1;//d� vi eventuellt kan komma att �ndra defaultv�rdet i matrisen till 2.
        }
        if(get_left_distance() < 20*5) //v�nstersidan oframkomlig.
        {
            driveable->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y - 1 + 2*(our_robot->robot_direction % 3)]=0;
            explored->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y - 1 + 2*(our_robot->robot_direction % 3)]=1;
        }
        else
        {
            driveable->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y - 1 + 2*(our_robot->robot_direction % 3)]=1;//d� vi eventuellt kan komma att �ndra defaultv�rdet i matrisen till 2.
        }
        if(get_front_distance() < 20*5) // fronten oframkomlig.
        {
            driveable->grid_matrix[our_robot->robot_pos_x + 2 - our_robot->robot_direction][our_robot->robot_pos_y]=0;
            explored->grid_matrix[our_robot->robot_pos_x + 2 - our_robot->robot_direction][our_robot->robot_pos_y]=1;
        }
        else
        {
            driveable->grid_matrix[our_robot->robot_pos_x + 2 - our_robot->robot_direction][our_robot->robot_pos_y]=1;
            // explored_map[my_robot.robot_pos_x + 2 - my_robot.robot_direction][my_robot.robot_pos_y]=1;
        }
    }
}
// driven_length_since_turn n�dv�ndigt?



#endif // ROBOT_MOVEMENT_H_INCLUDED
