#include "styr_defs.h"
#include <util/delay.h>

#ifndef ACTION_H
#define ACTION_H

#define ABS_VALUE_RIGHT 3.5
#define ABS_VALUE_LEFT 3.5

// DIRECTION FUNCTIONS: -----------------------------------------
// Turn forward:
void turn_forward();

// Turn back:
void turn_back();
void turn_back_control_on_both_walls();

// Turn left:
void turn_left();
void turn_left_control_on_right_wall();

// Turn right:
void turn_right();
void turn_right_control_on_left_wall();
void turn_right_control_on_zero_walls();
void turn_right_control_on_back_wall();



// MAZE FUNCTIONS: -----------------------------------------------
unsigned char get_possible_directions();
void make_direction_decision();

#endif