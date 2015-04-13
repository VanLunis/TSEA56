//
//  styr.c
//  
//  Created by Ola Grankvist on 30/03/15.


#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#define F_CPU 1000000

#define DD_SS DDB4
#define DD_MOSI DDB5
#define DD_MISO DDB6
#define DD_SCK DDB7
#define FULL_SPEED 50
#define SLOW_SPEED 25

// limits for motor control signals:   VERY_NEGATIVE_LIMIT < SLIGHT_NEGATIVE_LIMIT < SLIGHT_POSITIVE_LIMIT < VERY_POSITIVE_LIMIT
#define VERY_NEGATIVE_LIMIT -30
#define SLIGHT_NEGATIVE_LIMIT -10
#define SLIGHT_POSITIVE_LIMIT 10
#define VERY_POSITIVE_LIMIT 30


/*
 A = left wheels
 B = right wheels
 
 */


void init();
void pwm_init();
void SPI_slave_init();
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
double controller(int e, int alpha, int e_prior, int alpha_prior);
void setMotor(double u);
void grip_object();
void drop_down_object();
void move_arm();

/*_____________________________________________________________________
 ________________________________ MAIN _________________________________
 _______________________________________________________________________ */

int main(void)
{
    // initialize:
    // sei(); inte?
    init();
    
    double alpha = 0;
    double local_alpha;
    double e = 0;
    double local_e;
    double e_prior = 0;
    double alpha_prior = 0;
    
    double u;
    
    
    DDRC = (1<< DDC7);
    
    
    
    while(1)
    {
        local_e = e;
        local_alpha = alpha;
        u = controller(local_e, local_alpha, e_prior, alpha_prior);
        setMotor(u);
        e_prior = local_e;
        alpha_prior = local_alpha;
        
        
        PORTC = (1 << PORTC7);
        _delay_ms(1);
        PORTC = (0 << PORTC7);
        _delay_ms(20);
    }
}


/*_____________________________________________________________________
 ______________________________ FUNCTIONS ______________________________
 _______________________________________________________________________ */
void init(){
    pwm_init();
    SPI_slave_init();
}
void pwm_init()
{
    OCR2A = 0;
    OCR2B = 0;
    
    /*  TCCR2A: [COM2x1, COM2x0] = [1,0] =>	OCR2n clears on compare match
     [WGM22, WGM21, WGM20] = [0,1,1] => fast PWM
     
     TCCR2B: [CS22, CS21, CS20] = [0,1,1] => clk/8   */
    
    TCCR2A = (1<<WGM21)| (1<<WGM20) | (1<<COM2A1) | (0<<COM2A0) | (1<<COM2B1) | (0<<COM2B0);
    TCCR2B = (0<<WGM22) | (0<<CS22) | (1<<CS21) | (1<<CS20);
    
    OCR2A = 0; // init compare, i.e. init speed=0
    OCR2B = 0; // init compare, i.e. init speed=0
    
    // set port 20 and port 21 as right resp. left
    DDRD = 0xFF;
    // Initiate gear as 11: straight forward
    PORTD = (1<<PORTD0) | (1<<PORTD2);
}
void SPI_slave_init(){
    DDRB = (1 << DD_MOSI) | (0 << DD_SS) | (0 << DD_MISO)| (0 << DD_SCK); // sets output: MIS0; input: SS, MOSI, SCK
    SPCR = (1 << SPE) | (1 << SPIE); // SPIE enables SPI interrupts
}

// _______________________ INTERRUPTS FUNCTIONS ________________________

ISR(SPI_STC_vect){
    PORTA = SPDR;
    
    
    
}


// ________________________ CONTROL FUNCTIONS __________________________
double controller(int e, int alpha, int e_prior, int alpha_prior){
    // init:
    double delta_T = 1;
    double K_e_p = 1;
    double K_e_d = 1;
    double K_alpha_p = 1;
    double K_alpha_d = 1;
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
    PORTD = (1<<PORTD0) | (1<<PORTD2);
    set_speed_right_wheels(FULL_SPEED);
    set_speed_left_wheels(FULL_SPEED);
}
void rotate_left(){
    PORTD = (0<<PORTD0) | (1<<PORTD2); // sets right forward, left backward
    set_speed_right_wheels(FULL_SPEED);
    set_speed_left_wheels(FULL_SPEED);
}
void rotate_right(){
    PORTD = (1<<PORTD0) | (0<PORTD2); // sets left forward, right backward
    set_speed_right_wheels(FULL_SPEED);
    set_speed_left_wheels(FULL_SPEED);
}
void slight_left(){
    PORTD = (1<<PORTD0) | (1<<PORTD2);
    set_speed_right_wheels(FULL_SPEED);
    set_speed_left_wheels(SLOW_SPEED);
}
void slight_right(){
    PORTD = (1<<PORTD0) | (1<<PORTD2);
    set_speed_right_wheels(SLOW_SPEED);
    set_speed_left_wheels(FULL_SPEED);
}
void sharp_left(){
    PORTD = (1<<PORTD0) | (1<<PORTD2);
    set_speed_right_wheels(FULL_SPEED);
    set_speed_left_wheels(0);
}
void sharp_right(){
    PORTD = (1<<PORTD0) | (1<<PORTD2);
    set_speed_right_wheels(0);
    set_speed_left_wheels(FULL_SPEED);
}
void backward(){
    PORTD = (0<<PORTD0) | (0<<PORTD2);
    set_speed_right_wheels(FULL_SPEED);
    set_speed_left_wheels(FULL_SPEED);
}
