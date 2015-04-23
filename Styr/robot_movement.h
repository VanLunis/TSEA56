#ifndef ROBOT_MOVEMENT_H_INCLUDED
#define ROBOT_MOVEMENT_H_INCLUDED
#include <time.h>
/*
	functions related to the robot mouvement in the labyrint
 */


#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x)	(0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3




/*
 To set a bit:
	bit_set(foo, 0x01);
 To set bit number 5:
	bit_set(foo, BIT(5));
 To clear bit number 6 with a bit mask:
	bit_clear(foo, 0x40);
 To flip bit number 0:
	bit_flip(foo, BIT(0));
 To check bit number 3:
	if(bit_get(foo, BIT(3)))
	{
	}
 To set or clear a bit based on bit number 4:
 if(bit_get(foo, BIT(4)))
 {
	bit_set(bar, BIT(0));
 }
 else
 {
	bit_clear(bar, BIT(0));
 }
 */



void turn_left() //Uppdatera robotens riktning vid sväng.
{
    my_robot.robot_direction=(my_robot.robot_direction + 3) % 4;
}
void turn_right()
{
    my_robot.robot_direction=(my_robot.robot_direction + 1) % 4;
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

// grid_matrix[our_robot->robot_pos_x][our_robot->robot_pos_y + 1 - 2*(our_robot->robot_direction % 3)]
int get_left_distance()
{
    if(my_robot.robot_direction == 1)
    {
        if (hardcoded_map.grid_matrix[my_robot.robot_pos_x][my_robot.robot_pos_y + 1] == 0) {
            return 10;
        }
        return 200;
    }
    else if(my_robot.robot_direction==0)
    {
        if(hardcoded_map.grid_matrix[my_robot.robot_pos_x-1][my_robot.robot_pos_y]==0){
            return 10;
        }
        return 200;
    }
    
    return 10; //skrŠp
    
}
int get_right_distance()
{
    if(my_robot.robot_direction == 1)
    {
        if (hardcoded_map.grid_matrix[my_robot.robot_pos_x][my_robot.robot_pos_y - 1] == 0) {
            return 10;
        }
        return 200;
    }
    else if(my_robot.robot_direction==0)
    {
        if(hardcoded_map.grid_matrix[my_robot.robot_pos_x+1][my_robot.robot_pos_y]==0){
            return 10;
        }
        return 200;
    }
    return 10;//skrŠpreturn fšr att kompilatorn klagade
}
int get_front_distance()
{
    if (hardcoded_map.grid_matrix[my_robot.robot_pos_x + 1][my_robot.robot_pos_y] == 0) {
        return 10;
    }
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

void turn_robot(uint8_t robot_direction, uint8_t desirable_direction){
    // This function chooses the desired direction based on the robot direction:
    // robot direction = NORTH OR EAST OR SOUTH OR WEST
    // desirable direction = NORTH OR EAST OR SOUTH OR WEST
    switch(robot_direction){
            
            // ROBOT DRIVES NORTH
        case NORTH:
            switch (desirable_direction)
        {
            case NORTH:
                break;
            case EAST:
                turn_right();
                break;
            case SOUTH:
                turn_right();
                turn_right();
                break;
            case WEST:
                turn_left();
                break;
        }
            break;
            
            // ROBOT DRIVES EAST
        case EAST:
            switch (desirable_direction)
        {
            case NORTH:
                turn_left();
                break;
            case EAST:
                break;
            case SOUTH:
                turn_right();
                break;
            case WEST:
                turn_right();
                turn_right();
                break;
        }
            break;
            
            // ROBOT DRIVES SOUTH
        case SOUTH:
            switch (desirable_direction)
        {
            case NORTH:
                turn_right();
                turn_right();
                break;
            case EAST:
                turn_left();
                break;
            case SOUTH:
                break;
            case WEST:
                turn_right();
                break;
        }
            break;
            
            // ROBOT DRIVES SOUTH
        case WEST:
            switch (desirable_direction)
        {
            case NORTH:
                turn_right();
                break;
            case EAST:
                turn_right();
                turn_right();
                break;
            case SOUTH:
                turn_left();
                break;
            case WEST:
                break;
        }
            break;
            
    } // end of switch
    
}


unsigned char check_possible_directions(struct grid *driveable, uint8_t x, uint8_t y)
{
    
    /*
     checks possible directions to travel for the robot. 1 (true) for possible direction
     ----|---x: north
     ----|--x-: east
     ----|-x--: south
     ----|x---: west
     */
    
    uint8_t directions = 0x00;
    if (driveable->grid_matrix[x][y+1] == 1) // check north
    {
        directions |= 0x01;
    }
    if (driveable->grid_matrix[x+1][y] == 1) // check east
    {
        directions |= 0x02;
    }
    if (driveable->grid_matrix[x][y-1] == 1) // check south
    {
        directions |= 0x04;
    }
    if (driveable->grid_matrix[x-1][y] == 1) // checks west
    {
        directions |= 0x08;
    }
    return directions;
}

void take_decision(uint8_t possible_directions, uint8_t robot_direction, GRID *explored, ROBOT *my_robot ){
    
    if (possible_directions == (0x01 | 0x02 | 0x04 | 0x08 ))
    { // just one possible directions, i.e. dead-end => rotate
        turn_left();
        turn_left();
        printf("DEAD END");
    }else if ( bit_get(possible_directions, BIT(0)) + bit_get(possible_directions, BIT(1)) + bit_get(possible_directions, BIT(2)) + bit_get(possible_directions, BIT(3)) == 2  )
    { // two possible directions, i.e. continue with the one not coming from:
        //bit_clear(possible_directions, robot_direction + 2 % 4); // clear the bit the robot came from
        if (possible_directions == 0x01)
        {
            turn_robot(robot_direction, NORTH );
        }
        else if (possible_directions == 0x02)
        {
            turn_robot(robot_direction, EAST );
        }
        else if (possible_directions == 0x04)
        {
            turn_robot(robot_direction, SOUTH );
        }
        else if (possible_directions == 0x08)
        {
            turn_robot(robot_direction, WEST );
        }
        
    }else if (  bit_get(possible_directions, BIT(0)) + bit_get(possible_directions, BIT(1)) + bit_get(possible_directions, BIT(2)) + bit_get(possible_directions, BIT(3)) >= 3  )
    {	// three or four possible directions, i.e. take one random:
        //beware of the case when we have 3 or 4 drivable but all explored directions.
        // checks which directions that not are explored: driveable->grid_matrix[x][y+1] == 1
        unsigned char explored_directions = 0x00;
        if (explored->grid_matrix[my_robot->robot_pos_x][my_robot->robot_pos_y +1] == 0 )
        {
            bit_set(explored_directions,BIT(NORTH)); // explored_char := 0x01
        }
        if (explored->grid_matrix[my_robot->robot_pos_x+1][my_robot->robot_pos_y] == 0 )
        {
            bit_set(explored_directions,BIT(EAST));
        }
        if (explored->grid_matrix[my_robot->robot_pos_x][my_robot->robot_pos_y-1] == 0 )
        {
            bit_set(explored_directions,BIT(SOUTH));
        }
        if (explored->grid_matrix[my_robot->robot_pos_x-1][my_robot->robot_pos_y] == 0 )
        {
            bit_set(explored_directions,BIT(WEST));
        }
        printf("Looking for non explored directions.");
        printf("\n possible directions %i. Current robot position: x: %i, y: %i",possible_directions, my_robot->robot_pos_x, my_robot->robot_pos_y);
        possible_directions &= explored_directions;
        
        srand(time(NULL));

        uint8_t r = rand() % 4;
        while ( !bit_get(possible_directions, BIT(r)) )
        {
            r = rand() % 4;
            printf("I make random: %i. Possible directions: %i", r, possible_directions);
        }
        turn_robot(robot_direction,r);
        
        
    }
    
    // drive forward after the robot has turned:
    go_forward();
}



#endif // ROBOT_MOVEMENT_H_INCLUDED
