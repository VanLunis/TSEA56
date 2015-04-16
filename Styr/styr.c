#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#define F_CPU 8000000UL

#define DD_SS DDB4
#define DD_MOSI DDB5
#define DD_MISO DDB6
#define DD_SCK DDB7
#define FULL_SPEED 25
#define SLOW_SPEED 10

// limits for motor control signals:   VERY_NEGATIVE_LIMIT < SLIGHT_NEGATIVE_LIMIT < SLIGHT_POSITIVE_LIMIT < VERY_POSITIVE_LIMIT
#define VERY_NEGATIVE_LIMIT -30
#define SLIGHT_NEGATIVE_LIMIT -10
#define SLIGHT_POSITIVE_LIMIT 10
#define VERY_POSITIVE_LIMIT 30

#define AUTONOMOUS_MODE 1
#define REMOTE_CONTROL_MODE 0

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
void stopp();
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
    init();
    volatile double e; // 8bits instead?
    volatile double alpha; // 8bits instead?
    
    for (;;)
    {
        local_e = e;
        local_alpha = alpha;;
        u = controller(local_e, local_alpha, e_prior, alpha_prior);
        setMotor(u);
        e_prior = local_e;
        alpha_prior = local_alpha;
    }
    
}


/*
 _______________________________________________________________________
 ______________________________ FUNCTIONS ______________________________
 _______________________________________________________________________
 */


// _______________________ INITILIZES FUNCTIONS ________________________

void init(){
    pwm_init();
    SPI_slave_init();
}

void pwm_init(){
    
    DDRD |= (1<<DDD6)|(1<<DDD7);		// sets OCR2A and OCR2B as outputs => needed?
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
}
void SPI_slave_init(){
    DDRB = (1 << DD_MOSI) | (0 << DD_SS) | (0 << DD_MISO)| (0 << DD_SCK); // sets output: MIS0; input: SS, MOSI, SCK
    SPCR = (1 << SPE) | (1 << SPIE); // SPIE enables SPI interrupts
}


// _______________________ INTERRUPTS FUNCTIONS ________________________




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