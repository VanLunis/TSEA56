/*
 * RobotStyr.c
 *
 * Created: 16/4 - 2015
 * Updated: 16:30 24/4 - 2015 by Ola
 * Actual code for merged solution between the modules
 *
 */
#define F_CPU 14700000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "buffer.h"

#define FULL_SPEED 50
#define SLOW_SPEED 40
#define VERY_SLOW_SPEED 25

#define WALLS_MAX_DISTANCE 22 // max distance in cm to where sensors cant find a wall
#define FRONT_MAX_DISTANCE 11
#define ROBOT_LENGTH 10 // in cm

// limits for motor control signals:   VERY_NEGATIVE_LIMIT < SLIGHT_NEGATIVE_LIMIT < SLIGHT_POSITIVE_LIMIT < VERY_POSITIVE_LIMIT

#define NEGATIVE_LIMIT -10
#define SLIGHT_NEGATIVE_LIMIT -3
#define SLIGHT_POSITIVE_LIMIT 3
#define POSITIVE_LIMIT 10


#define AUTONOMOUS_MODE 1
#define REMOTE_CONTROL_MODE 0

#define DELTA_T 1
#define K_e_P 15         // proportional gain for position error e
#define K_e_D 50        // derivative gain for position error e
#define K_alpha_P 20    // proportional gain for angular error alpha
#define K_alpha_D 70    // derivative gain for angular error alpha

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
double controller(double e, double alpha, double e_prior, double alpha_prior, double e_prior_prior, double alpha_prior_prior);
void setMotor(double u);
double set_alpha(unsigned char distance_right_back, unsigned char distance_right_front, unsigned char distance_left_back, unsigned char distance_left_front);
void grip_object();
void drop_down_object();
void move_arm();
unsigned char get_possible_directions(unsigned char distance_right_back, unsigned char distance_right_front, unsigned char distance_left_back, unsigned char distance_left_front, unsigned char distance_front);
void forward_slow();

void turn_dead_end();
void turn_forward();
void turn_right();
void turn_left();

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
    DDRA = 0xFF;
    sei();
    
    double e_prior = 0;
    double alpha_prior = 0;
    double e_prior_prior = 0;
    double alpha_prior_prior = 0;
    double u = 0;
    
    unsigned char distance_right_back = 0;
    unsigned char distance_right_front = 0;
    unsigned char distance_left_back = 0;
    unsigned char distance_left_front = 0;
    unsigned char distance_front = 0;
    unsigned char distance_driven = 0;
    
    
    
    unsigned char possible_directions = 0x00;
    stop();
    
    for (;;)
    {
        //PORTA = 0x00;
        
        for (int i = 0; i<1; i++)
        {
            
            if(!buffer_empty(&receive_buffer))
            {
                unsigned char temp_char = fetch_from_buffer(&receive_buffer).type;
                
                
                switch (temp_char)
                {
                    case 0xFF: // = distance to wall: right back
                        distance_right_back = round(fetch_from_buffer(&receive_buffer).val/5);
                        discard_from_buffer(&receive_buffer);
                        break;
                        
                    case 0xFE: // = distance to wall: right front
                        distance_right_front = round(fetch_from_buffer(&receive_buffer).val/5);
                        discard_from_buffer(&receive_buffer);
                        break;
                        
                    case 0xFD: // = distance to wall: front
                        distance_front = round(fetch_from_buffer(&receive_buffer).val/5);
                        discard_from_buffer(&receive_buffer);
                        break;
                        
                    case 0xFC:  // = distance to wall: left front
                        distance_left_front = round(fetch_from_buffer(&receive_buffer).val/5);
                        discard_from_buffer(&receive_buffer);
                        break;
                        
                    case 0xFB: // = distance to wall: left back
                        distance_left_back = round(fetch_from_buffer(&receive_buffer).val/5);
                        discard_from_buffer(&receive_buffer);
                        break;
                        
                    case 0xFA: // distance driven:
                        distance_driven = round(fetch_from_buffer(&receive_buffer).val/5);
                        discard_from_buffer(&receive_buffer);
                        break;
                        
                    case 0xF9:  // tejp sensor floor:
                        discard_from_buffer(&receive_buffer);
                        break;
                    default:
                        discard_from_buffer(&receive_buffer);
                } // end of switch
            }
        }
        
        
        
        if( distance_front > FRONT_MAX_DISTANCE){
            
            alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
            
            if (distance_left_front > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE || distance_left_back > WALLS_MAX_DISTANCE || distance_right_back > WALLS_MAX_DISTANCE)
            {
                u = controller(0, alpha, 0, alpha_prior, 0,  alpha_prior_prior);
                setMotor(u);
                u = (char) u;
                add_to_buffer(&send_buffer,0xF2,u);
                
                // updates values for PD:
                e_prior_prior = e_prior;
                alpha_prior_prior = alpha_prior;
                e_prior = e;
                alpha_prior = alpha;
            }
            else
            {
                e =  (distance_left_back + distance_left_front - distance_right_back - distance_right_front)/4 ;
                
                
                // PID:
                u = controller(e, alpha, e_prior, alpha_prior, e_prior_prior,  alpha_prior_prior);
                setMotor(u);
                u = (char) u;
                add_to_buffer(&send_buffer,0xF2,u);
                
                // updates values for PD:
                e_prior_prior = e_prior;
                alpha_prior_prior = alpha_prior;
                e_prior = e;
                alpha_prior = alpha;
            }
            
        }
        else //if// (distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE && distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE)
        {
            
            _delay_ms(1);
            stop();
            
            
            
            if (distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE)
            {
                rotate_right();
                for(int i=0; i<505; i++){
                    _delay_ms(1);
                }
                
                forward();
                for(int i=0; i<50; i++){
                    _delay_ms(1);
                    
                    if(!buffer_empty(&receive_buffer))
                    {
                        unsigned char temp_char = fetch_from_buffer(&receive_buffer).type;
                        
                        
                        switch (temp_char)
                        {
                            case 0xFF: // = distance to wall: right back
                                distance_right_back = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xFE: // = distance to wall: right front
                                distance_right_front = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xFD: // = distance to wall: front
                                distance_front = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xFC:  // = distance to wall: left front
                                distance_left_front = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xFB: // = distance to wall: left back
                                distance_left_back = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xFA: // distance driven:
                                distance_driven = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xF9:  // tejp sensor floor:
                                discard_from_buffer(&receive_buffer);
                                break;
                            default:
                                discard_from_buffer(&receive_buffer);
                        } // end of switch
                    }
                }
                stop();
                
                
            }
            else if (distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE)
            {
                rotate_left();
                for(int i=0; i<505; i++){
                    _delay_ms(1);
                }
                
                forward();
                for(int i=0; i<50; i++){
                    _delay_ms(1);
                    if(!buffer_empty(&receive_buffer))
                    {
                        unsigned char temp_char = fetch_from_buffer(&receive_buffer).type;
                        
                        
                        switch (temp_char)
                        {
                            case 0xFF: // = distance to wall: right back
                                distance_right_back = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xFE: // = distance to wall: right front
                                distance_right_front = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xFD: // = distance to wall: front
                                distance_front = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xFC:  // = distance to wall: left front
                                distance_left_front = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xFB: // = distance to wall: left back
                                distance_left_back = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xFA: // distance driven:
                                distance_driven = round(fetch_from_buffer(&receive_buffer).val/5);
                                discard_from_buffer(&receive_buffer);
                                break;
                                
                            case 0xF9:  // tejp sensor floor:
                                discard_from_buffer(&receive_buffer);
                                break;
                            default:
                                discard_from_buffer(&receive_buffer);
                        } // end of switch
                    }
                }
                stop();
                
                
            }else if (distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE && distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE)
            {
                // dead-end....
                _delay_ms(1);
                
            }
            
            
        }
        
        /*
         possible_directions =  get_possible_directions(distance_right_back, distance_right_front, distance_left_back, distance_left_front, distance_front);
         //PORTA = possible_directions;
         
         switch ( bit_get(possible_directions, BIT(0)) + bit_get(possible_directions, BIT(1)) + bit_get(possible_directions, BIT(2)) )
         {
         case (0): // dead-end!
         stop();
         break;
         
         case (1): // one possible direction:
         if (possible_directions = 0x01)
         {
         /*e =  (distance_left_back + distance_left_front - distance_right_back - distance_right_front)/4 ;
         alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
         // PID:
         u = controller(e, alpha, e_prior, alpha_prior, e_prior_prior,  alpha_prior_prior);
         setMotor(u);
         u = (char) u;
         add_to_buffer(&send_buffer,0xF2,u);
         // updates values for PD:
         e_prior_prior = e_prior;
         alpha_prior_prior = alpha_prior;
         e_prior = e;
         alpha_prior = alpha;
         
         }else if (possible_directions = 0x02)
         {
         turn_right();
         }
         else if (possible_directions = 0x08)
         {
         turn_left();
         }
         
         switch (possible_directions)
         {
         case (0x01):
         e =  (distance_left_back + distance_left_front - distance_right_back - distance_right_front)/4 ;
         alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
         // PID:
         u = controller(e, alpha, e_prior, alpha_prior, e_prior_prior,  alpha_prior_prior);
         setMotor(u);
         u = (char) u;
         add_to_buffer(&send_buffer,0xF2,u);
         // updates values for PD:
         e_prior_prior = e_prior;
         alpha_prior_prior = alpha_prior;
         e_prior = e;
         alpha_prior = alpha;
         break;
         case (0x02):
         turn_right();
         break;
         case (0x08):
         turn_left();
         break;
         }
         break;
         case (2): // three-way crossroad
         // take decision...
         break;
         case (3): // four-way crossrod
         // take decision...
         break;
         }
         
         /*
         
         
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
    
    ///////////////////////// TEST TEST TEST /////////////////////////
    DDRA = 0xFF;
    //////////////////////////////////////////////////////////////////
    
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

// ______________________ MAZE FUNCTIONS ____________________________

unsigned char get_possible_directions(unsigned char distance_right_back, unsigned char distance_right_front, unsigned char distance_left_back, unsigned char distance_left_front, unsigned char distance_front){
    unsigned char possible_directions = 0x00;
    /*
     possible_direction:
     (-----|---1) => forward open
     (-----|--1-) => right open
     (-----|-1--) => left open
     
     ex: (----|0110) => right AND left open
     */
    
    if ( distance_front > FRONT_MAX_DISTANCE) {
        // open forward
        possible_directions |= 0x01;
    }
    if (distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE )
    {
        // open to right:
        possible_directions |= 0x02;
    }
    if (distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE)
    {
        // open to left:
        possible_directions |= 0x04;
    }
    return possible_directions;
}


//_________________________CONTROL FUNCTIONS _________________________


double controller(double e, double alpha, double e_prior, double alpha_prior, double e_prior_prior, double alpha_prior_prior){
    // init:
    //double delta_T = 1;
    //double K_e_p = 1;
    //double K_e_d = 30;
    //double K_alpha_p = 10;
    //double K_alpha_d = 30;
    //double u;
    
    
    double derivative_e = (e - e_prior/2 - e_prior_prior/2)/DELTA_T;
    double derivative_alpha = (alpha - alpha_prior/2 - alpha_prior_prior/2 )/DELTA_T;
    
    return K_e_P*e + K_e_D*derivative_e + K_alpha_P*alpha + K_alpha_D*derivative_alpha;
    
}

void setMotor(double u){
    
    if ( u < NEGATIVE_LIMIT){					sharp_right();      }
    else if ( u < SLIGHT_NEGATIVE_LIMIT){       slight_right();     }
    else if ( u < SLIGHT_POSITIVE_LIMIT){		forward();          }
    else if ( u < POSITIVE_LIMIT)	{           slight_left();      }
    else{										sharp_left();       }
    
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

double set_alpha(unsigned char distance_right_back, unsigned char distance_right_front, unsigned char distance_left_back, unsigned char distance_left_front){
    
    
    if ( distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE) // When one of the right hand side sensors can't see a wall alpha is calculated from the left hand side sensors
    {
        alpha = distance_left_front - distance_left_back;
        
    }
    else if (distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE) // When one of the right hand side sensors can't see a wall alpha is calculated from the left hand side sensors
    {
        alpha = distance_right_front-distance_right_back;
    }
    else
    {   // weights angle from both left and right sensors:
        alpha = (distance_left_front - distance_left_back)/2 + (distance_right_back - distance_right_front)/2;
    }
    
    return alpha/ROBOT_LENGTH;
    
}

// functions for turning in crossroads:
void turn_forward()
{
    
    
}

void turn_right()
{
    /*	
     // function to turn right, e.g. in a crossroad
     #define dt 0.010 // 10 ms
     unsigned char angular_velocity_right = 0;
     double angle_rotated = 0;
     rotate_right();
     
     while (angle_rotated < 90)
     {
     
     if(!buffer_empty(&receive_buffer))
     {
     unsigned char temp_char = fetch_from_buffer(&receive_buffer).type;
     _delay_ms(0.5);
     
     if (temp_char == 0xF8)
     {
				 angular_velocity_right = fetch_from_buffer(&receive_buffer).val;
     }
     
     }
     angle_rotated = angle_rotated + angular_velocity_right*dt;
     // _delay_ms(10);
     }
     */
    
    rotate_right();
    for(int i=0; i<505; i++){
        _delay_ms(1);
    }
    
    forward();
    for(int i=0; i<400; i++){
        _delay_ms(1);
    }
    stop();
    
    
}
void turn_left()
{
    /*	
     // function to turn right, e.g. in a crossroad
     #define dt 0.010 // 10 ms
     unsigned char angular_velocity_right = 0;
     double angle_rotated = 0;
     rotate_right();
     
     while (angle_rotated < 90)
     {
     
     if(!buffer_empty(&receive_buffer))
     {
     unsigned char temp_char = fetch_from_buffer(&receive_buffer).type;
     _delay_ms(0.5);
     
     if (temp_char == 0xF8)
     {
				 angular_velocity_right = fetch_from_buffer(&receive_buffer).val;
     }
     
     }
     angle_rotated = angle_rotated + angular_velocity_right*dt;
     // _delay_ms(10);
     }
     */
    
    rotate_left();
    for(int i=0; i<505; i++){
        _delay_ms(1);
    }
    
    forward();
    for(int i=0; i<400; i++){
        _delay_ms(1);
    }
    stop();
    
    
    
}

void turn_dead_end(){
    
    rotate_left();
    for(int i=0; i<1010; i++){
        _delay_ms(1);
    }
    stop();
    
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
void forward_slow(){
    PORTC = (1<<PORTC1) | (1<<PORTC0);
    set_speed_right_wheels(VERY_SLOW_SPEED);
    set_speed_left_wheels(VERY_SLOW_SPEED);
}
void rotate_left(){
    PORTC = (0<<PORTC1) | (1<<PORTC0); // sets right forward, left backwards
    set_speed_right_wheels(60);
    set_speed_left_wheels(60);
}
void rotate_right(){
    PORTC = (1<<PORTC1) | (0<PORTC0); // sets left forward, right backwards
    set_speed_right_wheels(60);
    set_speed_left_wheels(60);
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