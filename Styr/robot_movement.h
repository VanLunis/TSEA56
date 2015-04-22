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

void update_map(struct grid *driveable, struct grid *explored, struct robot *our_robot)//ska köras då roboten kört till mitten av en ny ruta.
{
    if((our_robot->robot_direction % 2)==0)//kör vertikalt.
    {
        if(get_right_distance() < 20*5) //högersidan oframkomlig.
        {
            driveable->grid_matrix[our_robot->robot_pos_x + 1 - our_robot->robot_direction][our_robot->robot_pos_y]=0;
            explored->grid_matrix[our_robot->robot_pos_x + 1 - our_robot->robot_direction][our_robot->robot_pos_y]=1;
        }
        else // no wall
        {
            driveable->grid_matrix[our_robot->robot_pos_x + 1 - our_robot->robot_direction][our_robot->robot_pos_y]=1;//då vi eventuellt kan komma att ändra defaultvärdet i matrisen till 2.
            // lägga in att vi har utforskat rutan? Eller sköts när man kör rakt fram?
        }
        if(get_left_distance() < 20*5) //vänstersidan oframkomlig.
        {
            driveable->grid_matrix[our_robot->robot_pos_x - 1 + our_robot->robot_direction][our_robot->robot_pos_y]=0;
            explored->grid_matrix[our_robot->robot_pos_x - 1 + our_robot->robot_direction][our_robot->robot_pos_y]=1;
        }
        else
        {
            driveable->grid_matrix[our_robot->robot_pos_x -1 + our_robot->robot_direction][our_robot->robot_pos_y]=1;//då vi eventuellt kan komma att ändra defaultvärdet i matrisen till 2.
            // Lägga in att vi har utforskat rutan? Eller sköts det när man åker rakt fram?
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
    else //kör horisontellt.
    {
        if(get_right_distance() < 20*5) //högersidan oframkomlig.
        {
            driveable->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y + 1 - 2*(our_robot->robot_direction % 3)]=0;
            explored->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y + 1 - 2*(our_robot->robot_direction % 3)]=1;
        }
        else
        {
            driveable->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y + 1 - 2*(our_robot->robot_direction % 3)]=1;//då vi eventuellt kan komma att ändra defaultvärdet i matrisen till 2.
        }
        if(get_left_distance() < 20*5) //vänstersidan oframkomlig.
        {
            driveable->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y - 1 + 2*(our_robot->robot_direction % 3)]=0;
            explored->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y - 1 + 2*(our_robot->robot_direction % 3)]=1;
        }
        else
        {
            driveable->grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y - 1 + 2*(our_robot->robot_direction % 3)]=1;//då vi eventuellt kan komma att ändra defaultvärdet i matrisen till 2.
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
// driven_length_since_turn nödvändigt?



#endif // ROBOT_MOVEMENT_H_INCLUDED
