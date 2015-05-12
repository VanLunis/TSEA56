#include "control.h"
#include "styr_defs.h"
#include "styrcomm.h"
#include "steering.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

    // Initiates control variables
double e = 0; // Position error
double alpha = 0; // Angle error
    
double e_prior = 0;
double alpha_prior = 0;
double e_prior_prior = 0;
double alpha_prior_prior = 0;
unsigned char wheel_click_prior = 0;

void go_forward(double * ptr_e, double *ptr_e_prior, double *ptr_e_prior_prior, double* ptr_alpha, double* ptr_alpha_prior, double* ptr_alpha_prior_prior )
{
    // DRIVING IN A CORRIDOR
    if (distance_left_front < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE && distance_left_back < WALLS_MAX_DISTANCE && distance_right_back < WALLS_MAX_DISTANCE)
    {
        *ptr_e =  (distance_left_back + distance_left_front - distance_right_back - distance_right_front)/4 ;
        
        // PID:
        double u = controller(*ptr_e, *ptr_alpha, *ptr_e_prior, *ptr_alpha_prior, *ptr_e_prior_prior,  *ptr_alpha_prior_prior);
        setMotor(u,*ptr_alpha);
        u = (char) u;
        add_to_buffer(&send_buffer,0xF2,u);
        add_to_buffer(&send_buffer,0xF1,(char) *ptr_alpha);
        add_to_buffer(&send_buffer,0xF0,(char) *ptr_e);
        
        // updates values for PD:
        *ptr_e_prior_prior = *ptr_e_prior;
        *ptr_alpha_prior_prior = *ptr_alpha_prior;
        *ptr_e_prior = *ptr_e;
        *ptr_alpha_prior = *ptr_alpha;
    }
    
    // USE ONLY ONE SIDE TO CONTROL
    else if ( (distance_left_front < WALLS_MAX_DISTANCE && distance_left_back < WALLS_MAX_DISTANCE ) || (distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE))
    {	// just use one side in the maze to control
        double u = controller(0, *ptr_alpha, 0, *ptr_alpha_prior, 0,  *ptr_alpha_prior_prior);
        setMotor(u,*ptr_alpha);
        u = (char) u;
        add_to_buffer(&send_buffer,0xF2,u);
        add_to_buffer(&send_buffer,0xF1, (char) *ptr_alpha);
        
        // updates values for derivative:
        *ptr_e_prior_prior = *ptr_e_prior;
        *ptr_alpha_prior_prior = *ptr_alpha_prior;
        *ptr_e_prior = *ptr_e;
        *ptr_alpha_prior = *ptr_alpha;
    }
    
    // DRIVING INTO A CORRIDOR FROM A CROSSING OR A TURN
    else if (distance_left_back > WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE)
    { // control on the front sensors!
        *ptr_e =  (distance_left_front - distance_right_front)/2 ;
        double u = controller(*ptr_e, 0, *ptr_e_prior, 0, *ptr_e_prior_prior,  0);
        setMotor(u,*ptr_alpha);
        u = (char) u;
        add_to_buffer(&send_buffer,0xF2,u);
        add_to_buffer(&send_buffer,0xF1, (char) *ptr_alpha);
        
        // updates values for derivative:
        *ptr_e_prior_prior = *ptr_e_prior;
        *ptr_alpha_prior_prior = *ptr_alpha_prior;
        *ptr_e_prior = *ptr_e;
        *ptr_alpha_prior = *ptr_alpha;
    }
    
    // DRIVING INTO A CROSSING OR A TURN FROM A CORRIDOR
    else if (distance_left_back < WALLS_MAX_DISTANCE && distance_right_back < WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE)
    { // control on the back sensors!
        *ptr_e =  (distance_left_back - distance_right_back)/2 ;
        double u = controller(*ptr_e, 0, *ptr_e_prior, 0, *ptr_e_prior_prior,  0);
        setMotor(u,*ptr_alpha);
        u = (char) u;
        add_to_buffer(&send_buffer,0xF2,u);
        add_to_buffer(&send_buffer,0xF1, (char) *ptr_alpha);
        
        // updates values for derivative:
        *ptr_e_prior_prior = *ptr_e_prior;
        *ptr_alpha_prior_prior = *ptr_alpha_prior;
        *ptr_e_prior = *ptr_e;
        *ptr_alpha_prior = *ptr_alpha;
    }
    
}
double controller(double e, double alpha, double e_prior, double alpha_prior, double e_prior_prior, double alpha_prior_prior){
    
    double derivative_e = (e - e_prior/2 - e_prior_prior/2)/DELTA_T;
    double derivative_alpha = (alpha - alpha_prior/2 - alpha_prior_prior/2 )/DELTA_T;
    
    return K_e_P*e + K_e_D*derivative_e + K_alpha_P*alpha + K_alpha_D*derivative_alpha;
    
}
void setMotor(double u, double alpha){
    
    if ( u < NEGATIVE_LIMIT){
		sharp_right();
		}
    else if ( u < SLIGHT_NEGATIVE_LIMIT){
		slight_right();
		}
    else if ( u < SLIGHT_POSITIVE_LIMIT){
        if (alpha > 5)
        {
            forward_slow();
        }
        else
        {
            forward();
        }
    }
    else if ( u < POSITIVE_LIMIT){ slight_left(); }
    else{ sharp_left(); }
    
}
double set_alpha(unsigned char distance_right_back, unsigned char distance_right_front, unsigned char distance_left_back, unsigned char distance_left_front){
#define ALPHA_MULT_CONSTANT 10 // a constant to increase alpha to make greater impact on the PD-controller
    double alpha;
    if ( distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE) // When one of the right hand side sensors can't see a wall alpha is calculated from the left hand side sensors
    {
        alpha = 5*(distance_left_front - distance_left_back);
        
    }
    else if (distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE) // When one of the right hand side sensors can't see a wall alpha is calculated from the left hand side sensors
    {
        alpha = 5*(distance_right_back - distance_right_front);
    }
    else
    {   // weights angle from both left and right sensors:
        alpha = (distance_left_front - distance_left_back)/2 + (distance_right_back - distance_right_front)/2;
    }
    
    return ALPHA_MULT_CONSTANT*alpha/ROBOT_LENGTH;
    
}

// MOTOR FUNCTIONS: ----------------------------------------------
void set_speed_right_wheels(unsigned char new_speed_percentage){
	if (new_speed_percentage <= 100 && new_speed_percentage >= 0 )
	{
		//speedPercentageRight = newSpeedPercentage;
		OCR2B = round(255*new_speed_percentage/100);
	}
}
void set_speed_left_wheels(unsigned char new_speed_percentage){
	if (new_speed_percentage <= 100 && new_speed_percentage >= 0 )
	{
		//speedPercentageRight = newSpeedPercentage;
		OCR2A = round(255*new_speed_percentage/100);
	}
}
