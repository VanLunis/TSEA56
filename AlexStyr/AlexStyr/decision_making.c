#include "decision_making.h"

#include "map.h"
#include "shortest_path.h"

int u = 0;

// MAZE FUNCTIONS: -----------------------------------------------
unsigned char get_possible_directions()
{	
    //TODO: Check in all crossings/turns if this really shows correct values
    
    unsigned char possible_directions = 0x00; 
    /*
     
     LRFB	
     possible_direction:
     (-----|--1-) => forward open
     (-----|-1--) => right open
     (-----|1---) => left open
     
     ex: (----|1100) => right AND left open
     */
    if (distance_back > 25)
    {
        possible_directions |= 0x01;
		robot.bwall = 0;
    }
	else{
		robot.bwall = 1;
	}
    if (distance_front > 25)
    {
        // open forward 
        possible_directions |= 0x02;
		robot.fwall = 0;
    }
	else
	{
		robot.fwall = 1;
	}
    if ((distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE )||((distance_right_front > 28 || distance_right_back > 28) && distance_front < 15))//added distance_right_back to test
    {
        // open to right:
        possible_directions |= 0x04;
		robot.rwall = 0;
    }
	else
	{
		robot.rwall = 1;
	}
    if ((distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE)||((distance_left_front > 28 || distance_left_back > 28) && distance_front < 15))//added ditance_left_back to test
    {
        // open to left:
        possible_directions |= 0x08;
		robot.lwall = 0;
    }
	else
	{
		robot.lwall = 1;
	}
    return possible_directions;
}
void make_direction_decision() //OBS: added some code to try to solve if the back sensor doesn't behave well
{
    unsigned char possible_directions = get_possible_directions();
    add_to_buffer(&send_buffer, 0xF8, possible_directions);
    // TODO: Implement the actual algorithm we want to use
    
    if(possible_directions == 0x01)// dead end
    {
        turn_back();
        turn_forward();
    }	
    else if(possible_directions == 0x05 || possible_directions == 0x04)// right turn 90 degrees //OBS: added 4 to test
    {
        turn_right();
        turn_forward();
    }
    else if(possible_directions == 0x07 || possible_directions == 0x06)// closed left t-crossing //OBS: added 6 to test
    {
        turn_right();
        turn_forward();
    }
    else if(possible_directions == 0x09 || possible_directions == 0x08)// left turn 90 degrees //OBS: added 8 to test
    {
        turn_left();
        turn_forward();
    }
    else if(possible_directions == 0x0B || possible_directions == 0x0A)// closed right t-crossing //OBS: added A to test
    {
        turn_forward();
    }
    else if(possible_directions == 0x0D || possible_directions == 0x0C)// closed front t-crossing //OBS: added C to test
    {
        turn_right();
        turn_forward();
    }
    else if(possible_directions == 0x0F || possible_directions == 0x0E)// 4-way-crossing //OBS: added E to test
    {
        turn_right();
        turn_forward();	
    }
    else
    {
        stop();
        _delay_ms(50);
        _delay_ms(50);
        // Indicates that something went wrong
        // TODO: Add functionality to correct position and continue
    }
}
unsigned char update_driven_distance(unsigned char driven_distance, unsigned char wheel_click, unsigned char wheel_click_prior){
    
#define WHEEL_CLICK_DISTANCE 5 //3.14*2*3.1/4 = distance for each click for the wheel
    
    if (wheel_click == 1 && wheel_click_prior == 0)
    {
        driven_distance = driven_distance + WHEEL_CLICK_DISTANCE;
        add_to_buffer(&send_buffer,0xEF, (char)driven_distance);
        if (driven_distance >= 40)
        {
            stop();
            _delay_ms(1000);
            driven_distance = 0;
        }
    }else if (wheel_click == 0 && wheel_click_prior == 1)
    {
        driven_distance = driven_distance + WHEEL_CLICK_DISTANCE;
        add_to_buffer(&send_buffer,0xEF, (char)driven_distance);
        if (driven_distance >= 40)
        {
            stop();
            _delay_ms(1000);
            driven_distance = 0;
        }
    }
    return driven_distance;
}

//Skall kallas i beslutpunkter
void make_decision()
{
	driven_distance = 0;
    int lx = robot.x-robot.ydir;
    int ly = robot.y+robot.xdir;

    //analog to lx/ly for right cell
    int rx = robot.x+robot.ydir;
    int ry = robot.y-robot.xdir;

    //forward cell
    int fx = robot.x+robot.ydir;
    int fy = robot.y+robot.xdir;

    if (!robot.lwall)
    {
        unvisited[un].x = lx;
        unvisited[un++].y = ly;
    }

    if (!robot.fwall)
    {
        unvisited[un].x = fx;
        unvisited[un++].y = fy;
    }

    if (!robot.rwall)
    {
        unvisited[un].x = rx;
        unvisited[un++].y = ry;
    }

    point temp = {robot.x, robot.y};
    floodfill(temp, unvisited[un]);
    traceBack(costmap, unvisited[un]);
    getCommands(unvisited[un--]);
}