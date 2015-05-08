#include "steering.h"
#include "styr_defs.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

// Steering functions
void forward(){
    PORTC = (1<<PORTC1) | (1<<PORTC0);
    if (distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE || distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE)
    {	// some of the sensors are big => drive slow
        set_speed_right_wheels(SLOW_SPEED);
        set_speed_left_wheels(SLOW_SPEED);
    }
    else
    {
        set_speed_right_wheels(FULL_SPEED);
        set_speed_left_wheels(FULL_SPEED);
    }
}
void forward_slow(){
    PORTC = (1<<PORTC1) | (1<<PORTC0);
    set_speed_right_wheels(SLOW_SPEED);
    set_speed_left_wheels(SLOW_SPEED);
}
void rotate_left(int speed){
    PORTC = (0<<PORTC1) | (1<<PORTC0); // sets right forward, left backwards
    set_speed_right_wheels(speed);
    set_speed_left_wheels(speed);
}
void rotate_right(int speed){
    PORTC = (1<<PORTC1) | (0<PORTC0); // sets left forward, right backwards
    set_speed_right_wheels(speed);
    set_speed_left_wheels(speed);
}
void slight_left(){
    PORTC = (1<<PORTC1) | (1<<PORTC0);
    if (distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE || distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE)
    {
        set_speed_right_wheels(SLOW_SPEED);
        set_speed_left_wheels(VERY_SLOW_SPEED);
    }
    else
    {
        set_speed_right_wheels(FULL_SPEED);
        set_speed_left_wheels(SLOW_SPEED);
    }
    
}
void slight_right(){
    PORTC = (1<<PORTC1) | (1<<PORTC0);
    
    if (distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE || distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE)
    {
        set_speed_right_wheels(VERY_SLOW_SPEED);
        set_speed_left_wheels(SLOW_SPEED);
    }
    else
    {
        set_speed_right_wheels(SLOW_SPEED);
        set_speed_left_wheels(FULL_SPEED);
    }
}
void sharp_left(){
    PORTC = (1<<PORTC1) | (1<<PORTC0);
    if (distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE || distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE)
    {
        set_speed_right_wheels(SLOW_SPEED);
        set_speed_left_wheels(VERY_SLOW_SPEED);
    }
    else
    {
        set_speed_right_wheels(FULL_SPEED);
        set_speed_left_wheels(VERY_SLOW_SPEED);
    }
    
}
void sharp_right(){
    PORTC = (1<<PORTC1) | (1<<PORTC0);
    if (distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE || distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE)
    {
        set_speed_right_wheels(VERY_SLOW_SPEED);
        set_speed_left_wheels(SLOW_SPEED);
    }
    else
    {
        set_speed_right_wheels(VERY_SLOW_SPEED);
        set_speed_left_wheels(FULL_SPEED);
    }
    
    
}
void backwards(){
    PORTC = (0<<PORTC1) | (0<<PORTC0);
    set_speed_right_wheels(FULL_SPEED);
    set_speed_left_wheels(FULL_SPEED);
}
void stop(){
    PORTC = (1<<PORTC1) | (1<<PORTC0);
    set_speed_right_wheels(0);
    set_speed_left_wheels(0);
}