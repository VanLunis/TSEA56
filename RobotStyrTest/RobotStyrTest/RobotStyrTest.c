/*
 * RobotStyrTest.c
 *
 * Created: 5/5 - 2015
 * 
 * Actual code for merged solution between the modules
 *
 */

// Define processor clock
#define F_CPU 14700000UL

// Include files
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "buffer.h"
#include <stdlib.h>

// CONSTANTS //////////////////////////////////////////////////////////
// Define speeds
#define FULL_SPEED 60
#define SLOW_SPEED 40
#define VERY_SLOW_SPEED 25

// Define distance constants
#define WALLS_MAX_DISTANCE 22
#define FRONT_MAX_DISTANCE 13
#define BACK_MAX_DISTANCE 30
#define ROBOT_LENGTH 10

#define ABS_VALUE_RIGHT 3.5
#define ABS_VALUE_LEFT 3.5

#define NEGATIVE_LIMIT -5
#define SLIGHT_NEGATIVE_LIMIT -1
#define SLIGHT_POSITIVE_LIMIT 1
#define POSITIVE_LIMIT 5

#define AUTONOMOUS_MODE 1
#define REMOTE_CONTROL_MODE 0

// PD-control constants
#define DELTA_T 1
#define K_e_P 15         // proportional gain for position error e
#define K_e_D 50        // derivative gain for position error e
#define K_alpha_P 20    // proportional gain for angular error alpha
#define K_alpha_D 70    // derivative gain for angular error alpha


// VARIABLES //////////////////////////////////////////////////////// 

// SPI communication variables
struct data_buffer receive_buffer;
struct data_buffer send_buffer;
volatile int mode = 0; // 0 receiving, 1 sending.
volatile int transmission_status = 0;
volatile struct data_byte temp_data;
volatile int counter = 0;

// Distance variables (from sensors)
unsigned char distance_right_back = 0;
unsigned char distance_right_front = 0;
unsigned char distance_left_back = 0;
unsigned char distance_left_front = 0;
unsigned char distance_front = 0;
unsigned char distance_back = 0;
unsigned char distance_driven = 0;

// FUNCTION DECLARATIONS: ////////////////////////////////////////////
// Init function:
void init_control_module(void);

// Communication functions:
void send_to_master(struct data_buffer* my_buffer);
void receive_from_master(struct data_buffer* my_buffer);
void update_values_from_sensor();

void remote_control(char control_val);

// Motor functions:
void set_speed_right_wheels(unsigned char new_speed_percentage);
void set_speed_left_wheels(unsigned char new_speed_percentage);
void rotate_right(int speed);
void rotate_left(int speed);
void forward();
void slight_left();
void slight_right();
void sharp_left();
void sharp_right();
void backwards();
void stop();
void go_forward(double * ptr_e, double * ptr_e_prior, double * ptr_e_prior_prior, double* ptr_alpha, double* ptr_alpha_prior, double* ptr_alpha_prior_prior );
double controller(double e, double alpha, double e_prior, double alpha_prior, double e_prior_prior, double alpha_prior_prior);
void setMotor(double u, double alpha);
double set_alpha(unsigned char distance_right_back, unsigned char distance_right_front, unsigned char distance_left_back, unsigned char distance_left_front);
void forward_slow();

// Direction functions:
void turn_forward();
void turn_back();
void turn_back_control_on_both_walls();
void turn_left();
void turn_left_control_on_right_wall();
void turn_right();
void turn_right_control_on_left_wall();

// Maze functions:
unsigned char get_possible_directions();
void make_direction_decision();

// Functions for the arm:
void grip_object();
void drop_down_object();
void move_arm();

// functions for single bit manipulation:
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x)	(0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))


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
    
	// Initiates control variables
	double e = 0; // Position error
	double alpha = 0; // Angle error

	double e_prior = 0;
	double alpha_prior = 0;
	double e_prior_prior = 0;
	double alpha_prior_prior = 0;
	double u = 0;
	
	stop();
	
	for(;;)
	{
		while(!buffer_empty(&receive_buffer))
		{
			update_values_from_sensor();	
		}
			
		// TODO: Check for 3 or 4 way crossing with open front
			
		// In a straight corridor?:
		if(distance_front > FRONT_MAX_DISTANCE) // front > 13
		{
			// Drive forward:
			alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
			go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior );
		}
		// In some kind of turn or crossing:
		else // front < 13
		{
			// Stop, check directions and decide which way to go:
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

//___________________________________________________________________________
// _________________________ INTERRUPTS ______________________________________
ISR(SPI_STC_vect)
{
    PORTB = (1<<PORTB3);
    PORTB = (0<<PORTB3);// OBS changed order so low when not active
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


//_________________________ INIT FUNCTION ____________________________________
void init_control_module(void)
{
    // SPI
    SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(0<<MSTR)|(0<<CPOL)|(0<<CPHA);
    DDRB = (1<<DDB6)|(1<<DDB3);
    PINB = (0<<PINB4); // OBS changed
    
    // IRQ1 and IRQ0 activated on rising edge
    EICRA = (1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(1<<ISC00);
    // Enable IRQ1 and IRQ0
    EIMSK = (1<<INT1)|(1<<INT0);
    
    //Initiate the buffers.
    buffer_init(&receive_buffer);
    buffer_init(&send_buffer);
    
    // PWM init
    DDRD  = (1<<DDD6)|(1<<DDD7);		// sets OCR2A and OCR2B as outputs => needed?
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

//_________________________ COMMUNICATION FUNCTIONS _________________________
void send_to_master(struct data_buffer* my_buffer)
{
    /*if (fetch_from_buffer(my_buffer).type == null_data_byte.type && fetch_from_buffer(my_buffer).val==null_data_byte.val)
     {
     transmission_status = 0;
     return ;
     }*/
    //if Transmission not yet started: fetch type and put it in SPDR.
    if(transmission_status == 0)
    {
        SPDR = fetch_from_buffer(my_buffer).type;
        transmission_status = 1;
    }
    //if Type already sent: fetch val and put it in SPDR.
    else if(transmission_status == 1)
    {
        
        SPDR = fetch_from_buffer(my_buffer).val;
        transmission_status = 2;
    }
    //if the master has accepted both bytes that were sent: discard the data_byte from the buffer.
    else if(transmission_status == 2)
    {
        discard_from_buffer(my_buffer);
        transmission_status = 0;
    }
};

//Method to receive a data_byte. Called when SPI interrupt has occurred.
void receive_from_master(struct data_buffer* my_buffer)
{
    //get type.
    if(transmission_status == 0)
    {
        temp_data.type = SPDR;
        transmission_status = 1;
    }
    //get val.
    else if(transmission_status == 1)
    {
        temp_data.val = SPDR;
        add_to_buffer(my_buffer, temp_data.type, temp_data.val);//add to receive_buffer when
        transmission_status = 0;
    }
}

void update_values_from_sensor(){
    
    if(!buffer_empty(&receive_buffer))
    {
		unsigned char temp_char = fetch_from_buffer(&receive_buffer).type;
        
        switch (temp_char)
        {
            case 0xFF: // = distance to wall: right back
                distance_right_back = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
                
            case 0xFE: // = distance to wall: right front
                distance_right_front = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
                
            case 0xFD: // = distance to wall: front
                distance_front = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
                
            case 0xFC:  // = distance to wall: left front
                distance_left_front = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
                
            case 0xFB: // = distance to wall: left back
                distance_left_back = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
				
			case 0xF7: // = distance to wall: back
			
				distance_back = round(fetch_from_buffer(&receive_buffer).val/5);
				break;	
                
            case 0xFA: // distance driven:
                distance_driven = round(fetch_from_buffer(&receive_buffer).val/5);
                break;	
                
            case 0xF9:  // tejp sensor floor:
                break;
                
        } // end of switch
		
		discard_from_buffer(&receive_buffer);
    } // end of if
}
// ______________________ MAZE FUNCTIONS ____________________________

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
    if (distance_back > 28)
    {
		possible_directions |= 0x01;
    }
    if (distance_front > WALLS_MAX_DISTANCE) 
	{
        // open forward 
        possible_directions |= 0x02;
    }
    if ((distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE )||(distance_right_front > 28 && distance_front < 15))
    {
        // open to right:
        possible_directions |= 0x04;
    }
    if ((distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE)||(distance_left_front > 28 && distance_front < 15))
    {
        // open to left:
        possible_directions |= 0x08;
    }
	return possible_directions;
}

void make_direction_decision()
{
	unsigned char possible_directions = get_possible_directions();
	add_to_buffer(&send_buffer, 0xF8, possible_directions);
	// TODO: Implement the actual algorithm we want to use
	
	if(possible_directions == 0x01)
	{
		turn_back();
		turn_forward();
	}	
	else if(possible_directions == 0x05)
	{
		turn_right();
		turn_forward();
	}
	else if(possible_directions == 0x09)
	{
		turn_left();
		turn_forward();
	}
	else
	{
		stop();
	}
}

//_________________________CONTROL FUNCTIONS _________________________

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
    
    if ( u < NEGATIVE_LIMIT){sharp_right();}
    else if ( u < SLIGHT_NEGATIVE_LIMIT){slight_right();}
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

void remote_control(char control_val){
    
    switch (control_val)
    {
        case 0x41: // = A in ascii => forward
            forward();
            break;
        case 0x42: // B in ascii => forward left
            slight_left();
            break;
        case 0x43: // C in ascii => forward right
            slight_right();
            break;
        case 0x44: // D in ascii => rotate left
            rotate_left(60);
            break;
        case 0x45: // E in ascii => rotate right
            rotate_right(60);
            break;
        case 0x46: // F in ascii => backwards
            backwards();
            break;
        case 0x47: // G in ascii => stop
            stop();
            break;
    }
    
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

//________________________ DIRECTION FUNCTIONS ________________________
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
	
	if(distance_front > WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE)
	{
		// FORWARD
		while (!(distance_left_front < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE))
		{
			forward_slow();
			update_values_from_sensor();
		}
		
		//  STOPP
		stop(); _delay_ms(50); _delay_ms(50);
	}
	
	stop(); _delay_ms(50); _delay_ms(50);
	
	while (!(distance_left_back < WALLS_MAX_DISTANCE && distance_right_back < WALLS_MAX_DISTANCE))
	{
		alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
		go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior );
		update_values_from_sensor();
	}
	//  STOPP
	stop(); _delay_ms(50); _delay_ms(50);
}

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
			update_values_from_sensor();
		}
	}
	
	// Closer to left wall then left? Then rotate left 180 degrees! :
	else
	{
		while ( !(distance_front > 30 && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && abs(distance_left_back - distance_left_front) < ABS_VALUE_LEFT))
		{
			rotate_left(50);
			update_values_from_sensor();
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
		update_values_from_sensor();
	}
	stop();
	_delay_ms(50);
	_delay_ms(50);
}

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
	
	while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back < BACK_MAX_DISTANCE))
	{
		rotate_left(60);
		update_values_from_sensor();
	}
	
	// Need to align? //
	stop();
	_delay_ms(50);
	_delay_ms(50);
	while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < 1 && distance_back < WALLS_MAX_DISTANCE))
	{
		if (distance_front < FRONT_MAX_DISTANCE)
		{
			if(distance_right_front > distance_right_back)
			{
				while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back < BACK_MAX_DISTANCE))
				{
					rotate_right(60);
					update_values_from_sensor();
				}
			}
			else
			{
				while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back < BACK_MAX_DISTANCE))
				{
					rotate_left(60);
					update_values_from_sensor();
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
		update_values_from_sensor();
	}
	stop();
	_delay_ms(50);
	_delay_ms(50);
}

void turn_right()
{
	if(distance_front < WALLS_MAX_DISTANCE)
	{
		turn_right_control_on_left_wall();
	}
}

void turn_right_control_on_left_wall()
{
	
	stop();
	_delay_ms(50);
	_delay_ms(50);
	
	// Rotate right:
	while(!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_back < BACK_MAX_DISTANCE))
	{
		rotate_right(60);
		update_values_from_sensor();
	}
	// Need to align? //
	stop();
	_delay_ms(50);
	_delay_ms(50);
	while(!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < 1 && distance_back < WALLS_MAX_DISTANCE))
	{
		if (distance_front < FRONT_MAX_DISTANCE)
		{
			if (distance_left_back > distance_left_front)
			{
				while(!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_back < BACK_MAX_DISTANCE))
				{
					rotate_right(60);
					update_values_from_sensor();
				}
			}
			else
			{
				while(!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_back < BACK_MAX_DISTANCE))
				{
					rotate_left(60);
					update_values_from_sensor();
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
		update_values_from_sensor();
	}
	stop();
	_delay_ms(50);
	_delay_ms(50);
}

// _________________________ MOTOR FUNCTIONS __________________________
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
