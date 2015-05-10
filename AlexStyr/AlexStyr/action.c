#include "action.h"
#include "control.h"

// DIRECTION FUNCTIONS: -----------------------------------------
// Turn forward:
void turn_forward()
{
    // Initiates control variables
    double e = 0; // Position error
    double alpha = 0; // Angle error
    
    double e_prior = 0;
    double alpha_prior = 0;
    double e_prior_prior = 0;
    double alpha_prior_prior = 0;
    
    // TODO: Control in different ways depending on kind of crossing/turn
    
    //  STOPP
    stop(); _delay_ms(50); _delay_ms(50);
    
    if(distance_front > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)//(distance_front > WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE)
    {
        // FORWARD
        while (!(distance_left_front < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE))
        {
            forward_slow();
            update_sensors_and_empty_receive_buffer();
        }
        
        //  STOPP
        stop(); _delay_ms(50); _delay_ms(50);
    }
    
    stop(); _delay_ms(50); _delay_ms(50);
    
    while (!(distance_left_back < WALLS_MAX_DISTANCE && distance_right_back < WALLS_MAX_DISTANCE))
    {
        alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
        go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior );
        update_sensors_and_empty_receive_buffer();
    }
    //  STOPP
    stop(); _delay_ms(50); _delay_ms(50);
}

// Turn back:
void turn_back()
{
    if(distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE && distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE)
    {
        turn_back_control_on_both_walls();
    }
}
void turn_back_control_on_both_walls()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    // Closer to left wall then right? Then rotate right 180 degrees! :
    if ((distance_right_back + distance_right_front) > (distance_left_back + distance_left_front))
    {
        while ( !(distance_front > 30 && abs(distance_right_back - distance_right_front) < ABS_VALUE_RIGHT && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT))
        {
            rotate_right(50);
            update_sensors_and_empty_receive_buffer();
        }
    }
    
    // Closer to left wall then left? Then rotate left 180 degrees! :
    else
    {
        while ( !(distance_front > 30 && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && abs(distance_left_back - distance_left_front) < ABS_VALUE_LEFT))
        {
            rotate_left(50);
            update_sensors_and_empty_receive_buffer();
        }
    }
    // Need to align? //
    stop();
    _delay_ms(50);
    _delay_ms(50);
    while ( !(distance_front > 30 && abs(distance_right_back - distance_right_front) < 1.2 && abs(distance_left_back - distance_left_front) < 1.2))
    {
        if(distance_left_back > distance_left_front)
        {
            rotate_right(40);
        }
        else
        {
            rotate_left(40);
        }
        update_sensors_and_empty_receive_buffer();
    }
    stop();
    _delay_ms(50);
    _delay_ms(50);
}

// Turn left:
void turn_left()
{
    if(distance_front < WALLS_MAX_DISTANCE)
    {
        turn_left_control_on_right_wall();
    }
}
void turn_left_control_on_right_wall()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE))
    {
        rotate_left(60);
        update_sensors_and_empty_receive_buffer();
    }
    
    // Need to align? //
    stop();
    _delay_ms(50);
    _delay_ms(50);
    while (!(distance_front > 26 && abs(distance_right_back - distance_right_front) < 1 && distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE))// OBS added 2 last
    {
        if (distance_front < FRONT_MAX_DISTANCE || distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE)// OBS added 2 last
        {
            if(distance_right_front > distance_right_back)
            {
                while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE))
                {
                    rotate_right(60);
                    update_sensors_and_empty_receive_buffer();
                }
            }
            else
            {
                while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE))
                {
                    rotate_left(60);
                    update_sensors_and_empty_receive_buffer();
                }
            }
            
        }
        else if(distance_right_front > distance_right_back)
        {
            rotate_right(40);
        }
        else
        {
            rotate_left(40);
        }
        update_sensors_and_empty_receive_buffer();
    }
    stop();
    _delay_ms(50);
    _delay_ms(50);
}

// Turn right:
void turn_right()
{
    if(distance_front < WALLS_MAX_DISTANCE)
    {
        turn_right_control_on_left_wall();
    }
    else if (distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back > WALLS_MAX_DISTANCE)
    {
        turn_right_control_on_zero_walls();
    }
}
void turn_right_control_on_left_wall()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    // Rotate right:
    while(!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE))
    {
        rotate_right(60);
        update_sensors_and_empty_receive_buffer();
    }
    // Need to align? //
    stop();
    _delay_ms(50);
    _delay_ms(50);
    while(!(distance_front > 26 && abs(distance_left_back - distance_left_front) < 1 && distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE))// OBS added 2 last
    {
        if (distance_front < FRONT_MAX_DISTANCE || distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE)// OBS added 2 last
        {
            if (distance_left_back > distance_left_front)
            {
                while(!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE))
                {
                    rotate_right(60);
                    update_sensors_and_empty_receive_buffer();
                }
            }
            else
            {
                while(!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE))
                {
                    rotate_left(60);
                    update_sensors_and_empty_receive_buffer();
                }
            }
        }
        else if(distance_left_back > distance_left_front)
        {
            rotate_right(40);
        }
        else
        {
            rotate_left(40);
        }
        update_sensors_and_empty_receive_buffer();
    }
    stop();
    _delay_ms(50);
    _delay_ms(50);
}
void turn_right_control_on_zero_walls()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    // short hard-coded rotate:
    rotate_right(60);
    for (int i = 0; i<400; i++){ _delay_ms(1);}
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    while (!( distance_front > 30 && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back > WALLS_MAX_DISTANCE))
    {
        rotate_right(60);
        update_sensors_and_empty_receive_buffer();
    }
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    rotate_right(60);
    for (int i = 0; i<180; i++){ _delay_ms(1);}
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
}
void turn_right_control_on_back_wall()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    // short hard-coded rotate:
    rotate_right(60);
    for (int i = 0; i<400; i++){ _delay_ms(1);}
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    while (!( distance_front > 30 && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back < WALLS_MAX_DISTANCE))
    {
        rotate_right(60);
        update_sensors_and_empty_receive_buffer();
    }
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    rotate_right(60);
    for (int i = 0; i<75; i++){ _delay_ms(1);}
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
}

void run_command()
{
	unsigned char possible_directions = get_possible_directions();
	add_to_buffer(&send_buffer, 0xF8, possible_directions);
	// TODO: Implement the actual algorithm we want to use
		
	if(command[c--] == 'b')// dead end
	{
		turn_back();
	}
	else if(command[c--] == 'r')// right turn 90 degrees //OBS: added 4 to test
	{
		turn_right();
	}
	else if(command[c--] == 'l')
	{
		turn_left();
	}
	else if(command[c - 1] == 'f' && command[c] == 'f')
	{	
		int driven_distance = 0;
		while(driven_distance < 40){
			alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
			go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior );
		            
			// update driven_distance:
			driven_distance = update_driven_distance(driven_distance, wheel_click, wheel_click_prior);
			wheel_click_prior = wheel_click;
		}
		robot.distance = driven_distance;
		c--;
	}
	else if(command[c--] == 'f')
	{
		turn_forward();	
	}
	
}