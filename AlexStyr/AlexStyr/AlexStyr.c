/*
 * AlexStyr.c
 *
 * Created: 5/8/2015 2:58:45 PM
 *  Author: aleer686
 */ 

// Define processor clock


// Include files
#include "styr_defs.h"
#include "bitman.h"
#include "styrcomm.h"
#include "buffer.h"
#include "action.h"
#include "bitman.h"
#include "control.h"
#include "decision_making.h"
#include "motor.h"
#include "steering.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>



// CONSTANTS //////////////////////////////////////////////////////////








// VARIABLES ////////////////////////////////////////////////////////

// SPI communication variables


// Distance variables (from sensors)

/*
 unsigned char distance_driven = 0; No need after new implementation?
*/

// FUNCTION DECLARATIONS: ////////////////////////////////////////////
// INIT FUNCTION: ---------------------------------------------
void init_control_module(void);

// COMMUNICATION FUNCTIONS: -----------------------------------


// MOTOR FUNCTIONS: ----------------------------------------------


// Steering functions

// CONTROL FUNCTIONS: -----------------------------------------------




// GRIPPING ARM FUNCTIONS: ---------------------------------------
// Functions for the arm:



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

//////////////////////////////////////////////////////////////////////
//----------------------------  MAIN -------------------------------//
//////////////////////////////////////////////////////////////////////

int main(void)
{
    init_control_module();
    sei();
    while(distance_front < FRONT_MAX_DISTANCE)
    {
        update_values_from_sensor();
    }
	
	init_map();
	init_floodtest();
	
    
    //Implementation for sending and creating map abstraction. Not done.
    /*int map[17][17];
    
    for (int i = 0; i<17;i++)
    {
        for (int j = 0; j<17; j++)
        {
            map[i][j] = 0;
        }
    }
    map[1][4] = 1;
    map[1][5] = 1;
    map[1][6] = 1;
    map[1 ][7] = 1;
    
    send_map(map);
    */
    
    
    
    
    stop();
    
    for(;;)
    {
        update_sensors_and_empty_receive_buffer();
        
        send_map(driveable);
        
        // In a straight corridor?:
        if(distance_front > FRONT_MAX_DISTANCE) // front > 13
        {
            // Drive forward:
            alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
            go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior );
            
            // update driven_distance:
            driven_distance = update_driven_distance(driven_distance, wheel_click, wheel_click_prior);
            wheel_click_prior = wheel_click;
			robot.distance = driven_distance;
            /*
            // goal (=tape on the floor) found => stop for 1 sec
            if (goal_found == 1)
            {
                stop();
                _delay_ms(100);
                goal_found = 0;
                
            }
			*/
        }
        // In some kind of turn or crossing:
        else // front < 13
        {
            // Stop, check directions and decide which way to go:
            make_direction_decision();
        }
        if (distance_front > 30 && distance_back > 30 &&
            ((distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)||
             (distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE)||
             (distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)))
        {
            make_direction_decision();
        }
        
        /*
         // In remote control mode?:
         else{
         if(!buffer_empty(&receive_buffer))
         {
         if(fetch_from_buffer(&receive_buffer).type == 0x01)
         {
         remote_control(fetch_from_buffer(&receive_buffer).val);
         }
         discard_from_buffer(&receive_buffer);
         }
         }*/
    }
    
}

// INTERRUPT VECTORS: ////////////////////////////////////////////////
ISR(SPI_STC_vect)
{
    PORTB = (1<<PORTB3);
    PORTB = (0<<PORTB3);
    //Depending on the current mode: do things.
    if(mode == 0)
    {
        receive_from_master(&receive_buffer);
    }
    else if(mode == 1)
    {
        send_to_master(&send_buffer);
    }
}

//Set slave to reading mode.
ISR(INT0_vect)
{
    transmission_status = 0;
    mode = 0;
}

//Set slave to sending mode.
ISR(INT1_vect)
{
    transmission_status = 1;
    mode = 1;
    SPDR = fetch_from_buffer(&send_buffer).type;
}


// FUNCTIONS: /////////////////////////////////////////////////////////
// INIT FUNCTION: ---------------------------------------------
void init_control_module(void)
{
    // SPI
    SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(0<<MSTR)|(0<<CPOL)|(0<<CPHA);
    DDRB = (1<<DDB6)|(1<<DDB3);
    PINB = (0<<PINB4);
    
    // IRQ1 and IRQ0 activated on rising edge
    EICRA = (1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(1<<ISC00);
    // Enable IRQ1 and IRQ0
    EIMSK = (1<<INT1)|(1<<INT0);
    
    //Initiate the buffers.
    buffer_init(&receive_buffer);
    buffer_init(&send_buffer);
    
    // PWM init
    DDRD  = (1<<DDD6)|(1<<DDD7);	// sets OCR2A and OCR2B as outputs => needed?
    OCR2A = 0;
    OCR2B = 0;
    
    /*  TCCR2A: [COM2x1, COM2x0] = [1,0] =>	OCR2n clears on compare match
     [WGM22, WGM21, WGM20] = [0,1,1] => fast PWM
     
     TCCR2B: [CS22, CS21, CS20] = [0,1,1] => clk/8   */
    
    TCCR2A = (1<<WGM21)| (1<<WGM20) | (1<<COM2A1) | (0<<COM2A0) | (1<<COM2B1) | (0<<COM2B0);
    TCCR2B = (0<<WGM22) | (0<<CS22) | (1<<CS21) | (1<<CS20);
    
    OCR2A = 0; // init compare, i.e. init speed=0
    OCR2B = 0; // init compare, i.e. init speed=0
    
    // set port 22 and port 23 as right resp. left
    DDRC = (1 << DDC0) | (1 << DDC1);
    // Initiate gear as 11: straight forward
    PORTC = (1<<PORTC1) | (1<<PORTC0);
    
};





// CONTROL FUNCTIONS: -----------------------------------------------



