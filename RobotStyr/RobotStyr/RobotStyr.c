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
#include "buffer.h"

#define FULL_SPEED 25
#define SLOW_SPEED 10

// limits for motor control signals:   VERY_NEGATIVE_LIMIT < SLIGHT_NEGATIVE_LIMIT < SLIGHT_POSITIVE_LIMIT < VERY_POSITIVE_LIMIT
#define VERY_NEGATIVE_LIMIT -30
#define SLIGHT_NEGATIVE_LIMIT -10
#define SLIGHT_POSITIVE_LIMIT 10
#define VERY_POSITIVE_LIMIT 30

#define AUTONOMOUS_MODE 1
#define REMOTE_CONTROL_MODE 0

struct data_buffer receive_buffer;
struct data_buffer send_buffer;
volatile int mode = 0; // 0 receiving, 1 sending. 
volatile int transmission_status = 0;
volatile struct data_byte temp_data;
volatile int counter = 0;

volatile char e = 0; // Position error
volatile char alpha = 0; // Angle error

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
void backward();
void stopp();
char controller(char e, char alpha, char e_prior, char alpha_prior);
void setMotor(char u);
void grip_object();
void drop_down_object();
void move_arm();

//////////////////////////////////////////////////////////////////////
//----------------------------  MAIN -------------------------------//
//////////////////////////////////////////////////////////////////////

int main(void)
{
	init_control_module();
	sei();
	char local_e = 0;
	char local_alpha = 0;
	char e_prior = local_e;
	char alpha_prior = local_alpha;
	char u;
		
    for(;;)
    {
///////////////////////// TEST TEST TEST /////////////////////////
		PORTA = amount_stored(&receive_buffer);
//////////////////////////////////////////////////////////////////
		local_e = e;
		local_alpha = alpha;
		u = controller(local_e, local_alpha, e_prior, alpha_prior);
		setMotor(u);
		e_prior = local_e;
		alpha_prior = local_alpha;	
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

/////////////////// Communication functions //////////////////////////
void send_to_master(struct data_buffer* my_buffer)
{
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

char controller(char e, char alpha, char e_prior, char alpha_prior){
	// init:
	unsigned char delta_T = 1;
	unsigned char K_e_p = 1;
	unsigned char K_e_d = 1;
	unsigned char K_alpha_p = 1;
	unsigned char K_alpha_d = 1;
	char u;
	
	char derivative_e = (e - e_prior)/delta_T;
	char derivative_alpha = (alpha - alpha_prior)/delta_T;
	
	u = K_e_p*e + K_e_d*derivative_e + K_alpha_p*alpha + K_alpha_d*derivative_alpha;
	return u;
}

void setMotor(char u){
	
	if		( u < VERY_NEGATIVE_LIMIT)	{		sharp_right();	}
	else if ( u < SLIGHT_NEGATIVE_LIMIT){		slight_right();	}
	else if ( u < SLIGHT_POSITIVE_LIMIT){		forward();		}
	else if ( u < VERY_POSITIVE_LIMIT)	{		slight_left();	}
	else{										sharp_left();	}
	
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
	PORTC = (0<<PORTC1) | (1<<PORTC0); // sets right forward, left backward
	set_speed_right_wheels(FULL_SPEED);
	set_speed_left_wheels(FULL_SPEED);
}
void rotate_right(){
	PORTC = (1<<PORTC1) | (0<PORTC0); // sets left forward, right backward
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
	set_speed_left_wheels(0);
}
void sharp_right(){
	PORTC = (1<<PORTC1) | (1<<PORTC0);
	set_speed_right_wheels(0);
	set_speed_left_wheels(FULL_SPEED);
}
void backward(){
	PORTC = (0<<PORTC1) | (0<<PORTC0);
	set_speed_right_wheels(FULL_SPEED);
	set_speed_left_wheels(FULL_SPEED);
}
void stopp(){
	PORTC = (1<<PORTC1) | (1<<PORTC0);
	set_speed_right_wheels(0);
	set_speed_left_wheels(0);
}