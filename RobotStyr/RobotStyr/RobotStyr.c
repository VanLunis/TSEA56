/*
 * RobotStyr.c
 *
 * Created: 16/4 - 2015
 * 
 * Actual code for merged solution between the modules
 *
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "buffer.h"

#define F_CPU = 16000000UL// OBS need to change
#define FULL_SPEED 75
#define SLOW_SPEED 50
#define VERY_SLOW_SPEED 25


// limits for motor control signals:   VERY_NEGATIVE_LIMIT < SLIGHT_NEGATIVE_LIMIT < SLIGHT_POSITIVE_LIMIT < VERY_POSITIVE_LIMIT
#define VERY_NEGATIVE_LIMIT -20
#define SLIGHT_NEGATIVE_LIMIT -5
#define SLIGHT_POSITIVE_LIMIT 5
#define VERY_POSITIVE_LIMIT 20

#define AUTONOMOUS_MODE 1
#define REMOTE_CONTROL_MODE 0

struct data_buffer receive_buffer;
struct data_buffer send_buffer;
volatile int mode = 0; // 0 receiving, 1 sending. 
volatile int transmission_status = 0;
volatile struct data_byte temp_data;
volatile int counter = 0;

volatile double e = 0; // Position error
volatile double alpha = 0; // Angle error

void init_control_module(void);

// Communication functions:
void send_to_master(struct data_buffer* my_buffer);
void receive_from_master(struct data_buffer* my_buffer);

// Control functions:
void set_speed_right_wheels(unsigned char new_speed_percentage);
void set_speed_left_wheels(unsigned char new_speed_percentage);
void rotate_right();
void rotate_left();
void forward();
void slight_left();
void slight_right();
void sharp_left();
void sharp_right();
void backwards();
void stop();
double controller(double e, double alpha, double e_prior, double alpha_prior);
void setMotor(double u);
void grip_object();
void drop_down_object();
void move_arm();

//////////////////////////////////////////////////////////////////////
//----------------------------  MAIN -------------------------------//
//////////////////////////////////////////////////////////////////////

int main(void)
{
	init_control_module();
	DDRA = 0xFF;
	sei();
	double local_e = 0;
	double local_alpha = 0;
	double e_prior = local_e;
	double alpha_prior = local_alpha;
	double u;
		
	unsigned char e_right_back = 0;
	unsigned char e_right_front = 0;
	unsigned char e_left_back = 0;
	unsigned char e_left_front = 0;
	unsigned char distance_front = 0;
	unsigned char distance_driven = 0;
	unsigned char alpha_left = 0;
	unsigned char alpha_right = 0;
	unsigned char sign_alpha_left = 0;
	unsigned char sign_alpha_right = 0;	
		
	forward();	
		
	for (;;)
	{
		if(!buffer_empty(&receive_buffer))
		{
			char temp_char = fetch_from_buffer(&receive_buffer).type;
			_delay_ms(5);
			
			// Loop this function a couple of times and  calculate control signal u based on several measurementupdates ?
			switch (temp_char)
			{
				case 0xFF: // = distance to wall: right back
					e_right_back = round(fetch_from_buffer(&receive_buffer).val/5);
					discard_from_buffer(&receive_buffer);
					break;
					
				case 0xFE: // = distance to wall: right front
					e_right_front = round(fetch_from_buffer(&receive_buffer).val/5);
					discard_from_buffer(&receive_buffer);
					break;
					
				case 0xFD: // = distance to wall: front
					distance_front = round(fetch_from_buffer(&receive_buffer).val/5);
					discard_from_buffer(&receive_buffer);
					break;
					
				case 0xFC:  // = distance to wall: left front
					e_left_front = round(fetch_from_buffer(&receive_buffer).val/5);
					discard_from_buffer(&receive_buffer);
					break;
				
				case 0xFB: // = distance to wall: left back
					e_left_back = round(fetch_from_buffer(&receive_buffer).val/5);
					discard_from_buffer(&receive_buffer);
					break;
					
				case 0xFA: // distance driven:
					distance_driven = round(fetch_from_buffer(&receive_buffer).val/5);
					discard_from_buffer(&receive_buffer);
					break;
					
				case 0xF9:  // tejp sensor floor:
					discard_from_buffer(&receive_buffer);
					break;
				
				case 0xF8:  // angular velocity right:
					discard_from_buffer(&receive_buffer);
					break;
					
				case 0xF7: // angular velocity left:
					discard_from_buffer(&receive_buffer);
					break;
					
				case 0xF6:  // alpha measured from right:
					alpha_right = round(fetch_from_buffer(&receive_buffer).val/5);
					discard_from_buffer(&receive_buffer);
					break;
					
				case 0xF5:  // alpha measured from left
					alpha_left = round(fetch_from_buffer(&receive_buffer).val/5);
					discard_from_buffer(&receive_buffer);
					break;
					
				case 0xF4:  // the sign for alpha right:
					sign_alpha_right = fetch_from_buffer(&receive_buffer).val;
					discard_from_buffer(&receive_buffer);
					break;
					
				case 0xF3:  // the sign for alpha left:
					sign_alpha_left = fetch_from_buffer(&receive_buffer).val;
					discard_from_buffer(&receive_buffer);
					break;
				default:
					discard_from_buffer(&receive_buffer);
			} // end of switch
		} // end of if	
			
			// set e, alpha etc:
			// e is an approximation, works for small numbers of e.
			
		
			

		
		
		if (distance_front < 15)
		{
			stop();
		}else{
			// PID: calculate and then choose direction:
			
			// set position error:
			e =  (e_left_back + e_left_front - e_right_back - e_right_front)/4 ;
			
			
			// set angular error alpha
			if (e_right_back || e_right_front > 30 ) // When one of the right hand side sensors can't see a wall alpha i calculated from the left hand side sensors
			{
				if (sign_alpha_left == 0)
				{
					alpha = alpha_left;
				}
				else
				{
					alpha = -alpha_left;
				}
			}
			else if (e_left_back || e_left_front > 30 ) // When one of the left hand side sensors can't see a wall alpha i calculated from the right hand side sensors
			{
				if (sign_alpha_right == 0)
				{
					alpha = alpha_right;
				} else
				{
					alpha = (- alpha_right);
				}
				
			}
			else // The sensors can see walls on both sides (corridor)
			{
				alpha = ((pow(-1,sign_alpha_right) * alpha_right) + (pow(-1,sign_alpha_left) * alpha_left))/2;  // [(-1)^{sign_alpha_right} * alpha_right + (-1)^{sign_alpha_left} * alpha_left)]/2
			}
			
			// PID:
			u = controller(e, alpha, e_prior, alpha_prior);
			setMotor(u);
			e_prior = e;
			alpha_prior = alpha;
		} 
		_delay_ms(0.5);
		//PORTA = u;
		//add_to_buffer(&send_buffer,0xF2, u);
		
	
		
		/*
    for(;;)
    {
		if(!buffer_empty(&receive_buffer))
		{
			if(fetch_from_buffer(&receive_buffer).type == 0x01)
			{
				remote_control(fetch_from_buffer(&receive_buffer).val);
				discard_from_buffer(&receive_buffer);
			}
		}
		
		
		_delay_ms(0.5);*/
		
		
		
		
		
	}
}

//////////////////////////////////////////////////////////////////////

/////////////////////// INTERRUPTS ///////////////////////////////////
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


///////////////////// INIT FUNCTION //////////////////////////////////
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
	
///////////////////////// TEST TEST TEST /////////////////////////
	DDRA = 0xFF;
//////////////////////////////////////////////////////////////////	
	
};

/////////////////// COMMUNICATION FUNCTIONS //////////////////////////
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

///////////////////// CONTROL FUNCTIONS ////////////////////////////

double controller(double e, double alpha, double e_prior, double alpha_prior){
	// init:
	double delta_T = 1;
	double K_e_p = 1;
	double K_e_d = 1;
	double K_alpha_p = 10;
	double K_alpha_d = 10;
	double u;
	
	double derivative_e = (e - e_prior)/delta_T;
	double derivative_alpha = (alpha - alpha_prior)/delta_T;
	
	u = K_e_p*e + K_e_d*derivative_e + K_alpha_p*alpha + K_alpha_d*derivative_alpha;
	return u;
}

void setMotor(double u){
	
	if		( u < VERY_NEGATIVE_LIMIT)	{		sharp_right();	}
	else if ( u < SLIGHT_NEGATIVE_LIMIT){		slight_right();	}
	else if ( u < SLIGHT_POSITIVE_LIMIT){		forward();		}
	else if ( u < VERY_POSITIVE_LIMIT)	{		slight_left();	}
	else{										sharp_left();	}
	
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
			rotate_left();
			break;
		case 0x45: // E in ascii => rotate right
			rotate_right();
			break;
		case 0x46: // F in ascii => backwards
			backwards();
			break;
		case 0x47: // G in ascii => stop
			stop();
			break;
	}
	
}

// _________________________ MOTOR FUNCTIONS ___________________________
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
	set_speed_right_wheels(FULL_SPEED);
	set_speed_left_wheels(FULL_SPEED);
}
void rotate_left(){
	PORTC = (0<<PORTC1) | (1<<PORTC0); // sets right forward, left backwards
	set_speed_right_wheels(FULL_SPEED);
	set_speed_left_wheels(FULL_SPEED);
}
void rotate_right(){
	PORTC = (1<<PORTC1) | (0<PORTC0); // sets left forward, right backwards
	set_speed_right_wheels(FULL_SPEED);
	set_speed_left_wheels(FULL_SPEED);
}
void slight_left(){
	PORTC = (1<<PORTC1) | (1<<PORTC0);
	set_speed_right_wheels(FULL_SPEED);
	set_speed_left_wheels(SLOW_SPEED);
}
void slight_right(){
	PORTC = (1<<PORTC1) | (1<<PORTC0);
	set_speed_right_wheels(SLOW_SPEED);
	set_speed_left_wheels(FULL_SPEED);
}
void sharp_left(){
	PORTC = (1<<PORTC1) | (1<<PORTC0);
	set_speed_right_wheels(FULL_SPEED);
	set_speed_left_wheels(VERY_SLOW_SPEED);
}
void sharp_right(){
	PORTC = (1<<PORTC1) | (1<<PORTC0);
	set_speed_right_wheels(VERY_SLOW_SPEED);
	set_speed_left_wheels(FULL_SPEED);
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