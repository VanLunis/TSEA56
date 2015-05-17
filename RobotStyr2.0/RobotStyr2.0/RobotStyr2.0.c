/*
 * RobotStyr2.0.c
 *
 * Created: 11/5 - 2015
 *
 * Try to merge new functionality (i.e. map), step by step into working control code
 *
 */

#include "RobotStyr2.0.h"
//////////////////////////////////////////////////////////////////////
//----------------------------  MAIN -------------------------------//
//////////////////////////////////////////////////////////////////////

int main(void)

{
    init_control_module();
    sei();
    init_map();
    
    // To make the robot stand still when turned on:
    
    if (autonomous_mode == 1)
    {
        while(distance_front < FRONT_MAX_DISTANCE)
        {
            update_values_from_sensor();
        }
        
        stop();
    }
    
    for(;;)
    {
		// In autonomous mode?:
		if(autonomous_mode == 1 && missionPhase == 0)
        {
			mission();
        }
		else if(autonomous_mode == 1)
		{
			stop();
		}
		// In remote control mode?:
		else
		{
			remote_control_mode();
		}
    }
    
}

//////////////////////////////////////////////////////////////////////
//-------------------------  FUNCTIONS  ----------------------------//
//////////////////////////////////////////////////////////////////////

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
    DDRD  = (1<<DDD6)|(1<<DDD7);        // sets OCR2A and OCR2B as outputs => needed?
    OCR2A = 0;
    OCR2B = 0;
    
    /*  TCCR2A: [COM2x1, COM2x0] = [1,0] =>  OCR2n clears on compare match
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

// COMMUNICATION FUNCTIONS: -----------------------------------
// Functions that are used in interrupts caused by the SPI-bus
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
void send_driveable()
{
    unsigned char char_to_send = 0x80; // first bit is set to not be sending nullbyte to PC!
    
    for (int row = 0; row<15; row++) // Loop for each row
    {
        
        // loop for the first 5 columns in each row:
        for (int column = 0; column<5; column++)
        {
            if (driveable[column+1][row+1] == 1) // +1 since the driveable is 17x17
            {
                bit_set(char_to_send, BIT(column));
            }
            else
            {
                bit_clear(char_to_send, BIT(column));
            }
        }
        add_to_buffer(&send_buffer,0xEE - (3*row) ,char_to_send); // EDIT TYPE NUMBER!
        
        char_to_send = 0x80;
        // loop for the following 6-10 columns
        for (int column = 5; column<10; column++)
        {
            
            if (driveable[column+1][row+1] == 1) // +1 since the driveable is 17x17
            {
                bit_set(char_to_send, BIT(column-5));
            }
            else
            {
                bit_clear(char_to_send, BIT(column-5));
            }
        }
        add_to_buffer(&send_buffer,0xEE - (3*row + 1),char_to_send); // EDIT TYPE NUMBER!
        
        char_to_send = 0x80;
        // loop for the following 11-15 columns
        for (int column = 10; column<15; column++)
        {
            if (driveable[column+1][row+1] == 1) // +1 since the driveable is 17x17
            {
                bit_set(char_to_send, BIT(column-10));
            }
            else
            {
                bit_clear(char_to_send, BIT(column-10));
            }
        }
        add_to_buffer(&send_buffer,0xEE - (3*row + 2),char_to_send); // EDIT TYPE NUMBER!
        
        
    }// end of row loop
}
void send_explored()
{
	unsigned char char_to_send = 0x80; // first bit is set to not be sending nullbyte to PC!
	
	for (int row = 0; row<15; row++) // Loop for each row
	{
		
		// loop for the first 5 columns in each row:
		for (int column = 0; column<5; column++)
		{
			if (explored[column+1][row+1] == 1) // +1 since the explored is 17x17
			{
				bit_set(char_to_send, BIT(column));
			}
			else
			{
				bit_clear(char_to_send, BIT(column));
			}
		}
		add_to_buffer(&send_buffer,0xAE - (3*row) ,char_to_send); // EDIT TYPE NUMBER!
		
		char_to_send = 0x80;
		// loop for the following 6-10 columns
		for (int column = 5; column<10; column++)
		{
			
			if (explored[column+1][row+1] == 1) // +1 since the explored is 17x17
			{
				bit_set(char_to_send, BIT(column-5));
			}
			else
			{
				bit_clear(char_to_send, BIT(column-5));
			}
		}
		add_to_buffer(&send_buffer,0xAE - (3*row + 1),char_to_send); // EDIT TYPE NUMBER!
		
		char_to_send = 0x80;
		// loop for the following 11-15 columns
		for (int column = 10; column<15; column++)
		{
			if (explored[column+1][row+1] == 1) // +1 since the explored is 17x17
			{
				bit_set(char_to_send, BIT(column-10));
			}
			else
			{
				bit_clear(char_to_send, BIT(column-10));
			}
		}
		add_to_buffer(&send_buffer,0xAE - (3*row + 2),char_to_send); // EDIT TYPE NUMBER!
		
		
	}// end of row loop
}

// In autonomous mode: get sensor values from receive buffer
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
				wheel_click = fetch_from_buffer(&receive_buffer).val;
				update_driven_distance();
				break;
                
            case 0xF9:  // tejp sensor floor:
				if (fetch_from_buffer(&receive_buffer).val == 1)
				{
					if(missionPhase == 0)
					{
						if(!start_detected)
						{
							start_detected = 1;
							driven_distance = 0;
						}
					}
					else if (missionPhase == 1)
					{
						if(!goal_detected)
						{
							goal_detected = 1;
							goalx = x + xdir;
							goaly = y + ydir;
						}
					}
					else if (missionPhase == 2)
					{
						if(!tape_detected)
						{
							tape_detected = 1;
						}
					}
				}
                break;
                
        } // end of switch
        
        discard_from_buffer(&receive_buffer);
    } // end of if
}
void update_sensors_and_empty_receive_buffer()
{
    while(!buffer_empty(&receive_buffer))
    {
        update_values_from_sensor();
    }
}

// In remote control mode: use remote control values from receive buffer
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
void remote_control_mode()// SWITCH
{
    if(!buffer_empty(&receive_buffer))
    {
        if(fetch_from_buffer(&receive_buffer).type == 0x01)
        {
            remote_control(fetch_from_buffer(&receive_buffer).val);
        }
        discard_from_buffer(&receive_buffer);
    }
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

// Steering functions
void forward(){
    PORTC = (1<<PORTC1) | (1<<PORTC0);
    if (distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE || distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE)
    {   // some of the sensors are big => drive slow
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

// CONTROL FUNCTIONS: -----------------------------------------------
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
        
        // updates values for PD:
        *ptr_e_prior_prior = *ptr_e_prior;
        *ptr_alpha_prior_prior = *ptr_alpha_prior;
        *ptr_e_prior = *ptr_e;
        *ptr_alpha_prior = *ptr_alpha;
    }
    
    // USE ONLY ONE SIDE TO CONTROL
    else if ( (distance_left_front < WALLS_MAX_DISTANCE && distance_left_back < WALLS_MAX_DISTANCE ) || (distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE))
    {   // just use one side in the maze to control
        double u = controller(0, *ptr_alpha, 0, *ptr_alpha_prior, 0,  *ptr_alpha_prior_prior);
        setMotor(u,*ptr_alpha);
        u = (char) u;
        
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


// DIRECTION FUNCTIONS: -----------------------------------------
// Turn forward:
void turn_forward()
{
	//  STOPP
	stop(); _delay_ms(50); _delay_ms(50);
	if(distance_front > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)//(distance_front > WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE)
	{
		// FORWARD
		while (!(distance_left_front < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE))
		{
			forward_slow();
			update_sensors_and_empty_receive_buffer();
		}
		
		//  STOPP
		stop(); _delay_ms(50); _delay_ms(50);
	}
	
	stop(); _delay_ms(50); _delay_ms(50);
	
	if(distance_front > WALLS_MAX_DISTANCE)
	{
		while (!(distance_left_back < WALLS_MAX_DISTANCE && distance_right_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE))// OBS changed
		{
			alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
			go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior );
			update_sensors_and_empty_receive_buffer();
		}
	}
	//  STOPP
	stop(); _delay_ms(50); _delay_ms(50);
	
}

// Turn back:
void turn_back()
{
    update_orientation('b');
	if(lwall && rwall)
	{
		turn_back_control_on_both_walls();
	}
	else if(lwall)
	{
		turn_back_control_on_right_wall();
	}
	else if(rwall)
	{
		turn_back_control_on_left_wall();
	}
	else
	{
		turn_back_control_on_zero_walls();
	}
	/*
    if(distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE
       && distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE)
    {
        turn_back_control_on_both_walls();
    }*/
    
}
void turn_back_control_on_both_walls()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
	
	
    // Closer to left wall then right? Then rotate right 180 degrees! :
    if ((distance_right_back + distance_right_front) > (distance_left_back + distance_left_front))
    {
        if (!fwall)
        {
			// short hard-coded rotate:
			rotate_right(60);
			for (int i = 0; i<700; i++){ _delay_ms(1);}	  
        }
		while ( !(distance_front > 30 && abs(distance_right_back - distance_right_front) < ABS_VALUE_RIGHT && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT))
        {
            rotate_right(50);
            update_sensors_and_empty_receive_buffer();
        }
    }
    
    // Closer to left wall then left? Then rotate left 180 degrees! :
    else
    {
        if (!fwall)
        {
	        // short hard-coded rotate:
	        rotate_left(60);
	        for (int i = 0; i<700; i++){ _delay_ms(1);}
        }
		while ( !(distance_front > 30 && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && abs(distance_left_back - distance_left_front) < ABS_VALUE_LEFT))
        {
            rotate_left(50);
            update_sensors_and_empty_receive_buffer();
        }
    }
    // Need to align? //
    stop();
    _delay_ms(50);
    _delay_ms(50);
    update_sensors_and_empty_receive_buffer();
    
    while (!(distance_front > 30 && abs(distance_right_back - distance_right_front) < 1.2 && abs(distance_left_back - distance_left_front) < 1.2))
    {
        if(distance_left_back > distance_left_front)
        {
            rotate_right(40);
        }
        else
        {
            rotate_left(40);
        }
        update_sensors_and_empty_receive_buffer();
    }
    stop();
    _delay_ms(50);
    _delay_ms(50);
}
void turn_back_control_on_right_wall()
{
	if(fwall)
	{
		turn_right_control_on_left_wall();
	}
	else
	{
		turn_right_control_on_back_wall();
	}
	
	while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE))
	{
		rotate_right(60);
		update_sensors_and_empty_receive_buffer();
	}
	
	// Need to align? //
	stop();
	_delay_ms(50);
	_delay_ms(50);
	update_sensors_and_empty_receive_buffer();
	
	while (!(distance_front > 30 && abs(distance_right_back - distance_right_front) < 1.2 && distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE))
	{
		if (distance_front < FRONT_MAX_DISTANCE || distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE)
		{
			if(distance_right_front > distance_right_back)
			{
				while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE))
				{
					rotate_right(60);
					update_sensors_and_empty_receive_buffer();
				}
			}
			else
			{
				while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE))
				{
					rotate_right(60);
					update_sensors_and_empty_receive_buffer();
				}
			}
			
		}
		else if(distance_left_front > distance_left_back)
		{
			rotate_right(40);
		}
		else
		{
			rotate_right(40);
		}
		update_sensors_and_empty_receive_buffer();
	}
	stop();
	_delay_ms(50);
	_delay_ms(50);
}
void turn_back_control_on_left_wall()
{
	if(fwall)
	{
		turn_left_control_on_right_wall();
	}
	else
	{
		turn_left_control_on_back_wall();
	}
	
	while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_LEFT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE))
	{
		rotate_left(60);
		update_sensors_and_empty_receive_buffer();
	}
	
	// Need to align? //
	stop();
	_delay_ms(50);
	_delay_ms(50);
	update_sensors_and_empty_receive_buffer();
	
	while (!(distance_front > 30 && abs(distance_left_back - distance_left_front) < 1.2 && distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE))
	{
		if (distance_front < FRONT_MAX_DISTANCE || distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE)
		{
			if(distance_left_front > distance_left_back)
			{
				while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_LEFT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE))
				{
					rotate_left(60);
					update_sensors_and_empty_receive_buffer();
				}
			}
			else
			{
				while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_LEFT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE))
				{
					rotate_left(60);
					update_sensors_and_empty_receive_buffer();
				}
			}
			
		}
		else if(distance_left_front > distance_left_back)
		{
			rotate_left(40);
		}
		else
		{
			rotate_left(40);
		}
		update_sensors_and_empty_receive_buffer();
	}
	stop();
	_delay_ms(50);
	_delay_ms(50);
}
void turn_back_control_on_zero_walls()
{
	if(fwall)
	{
		turn_right_control_on_left_wall();
		turn_right_control_on_back_wall();
	}
	else
	{
		turn_right_control_on_zero_walls();
		turn_right_control_on_zero_walls();
	}
}

// Turn left:
void turn_left()
{
    update_orientation('l');
    if(distance_front < WALLS_MAX_DISTANCE)
    {
        turn_left_control_on_right_wall();
    }
    else if(distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back > WALLS_MAX_DISTANCE)
    {
        turn_left_control_on_zero_walls();
    }
    else if(((distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE)||(distance_left_back > 28)||(distance_left_front > 28))
            && distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE && distance_back > WALLS_MAX_DISTANCE)
    {
        turn_left_control_on_back_wall();
    }
}
void turn_left_control_on_right_wall()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE))
    {
        rotate_left(60);
        update_sensors_and_empty_receive_buffer();
    }
    
    // Need to align? //
    stop();
    _delay_ms(50);
    _delay_ms(50);
    update_sensors_and_empty_receive_buffer();
    
    while (!(distance_front > 30 && abs(distance_right_back - distance_right_front) < 1.2 && distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE))
    {
        if (distance_front < FRONT_MAX_DISTANCE || distance_right_back > WALLS_MAX_DISTANCE || distance_right_front > WALLS_MAX_DISTANCE)
        {
            if(distance_right_front > distance_right_back)
            {
                while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE))
                {
                    rotate_right(60);
                    update_sensors_and_empty_receive_buffer();
                }
            }
            else
            {
                while (!(distance_front > WALLS_MAX_DISTANCE && abs(distance_right_back - distance_right_front) < ABS_VALUE_LEFT && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE))
                {
                    rotate_left(60);
                    update_sensors_and_empty_receive_buffer();
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
        update_sensors_and_empty_receive_buffer();
    }
    stop();
    _delay_ms(50);
    _delay_ms(50);
}
void turn_left_control_on_zero_walls()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    // short hard-coded rotate:
    rotate_left(60);
    for (int i = 0; i<380; i++){ _delay_ms(1);}
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    while (!( distance_front > 30 && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back > WALLS_MAX_DISTANCE))
    {
        rotate_left(60);
        update_sensors_and_empty_receive_buffer();
    }
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    rotate_left(60);
    for (int i = 0; i<150; i++){ _delay_ms(1);}
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
}
void turn_left_control_on_back_wall()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    // short hard-coded rotate:
    rotate_left(60);
    for (int i = 0; i<400; i++){ _delay_ms(1);}
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    while (!( distance_front > 30 && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back < WALLS_MAX_DISTANCE))
    {
        rotate_left(60);
        update_sensors_and_empty_receive_buffer();
    }
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    rotate_left(60);
    for (int i = 0; i<45; i++){ _delay_ms(1);}
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
}


// Turn right:
void turn_right()
{
    update_orientation('r');
    if(distance_front < WALLS_MAX_DISTANCE)
    {
        turn_right_control_on_left_wall();
    }
    else if(distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back > WALLS_MAX_DISTANCE)
    {
        turn_right_control_on_zero_walls();
    }
    else if(((distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)||(distance_right_back > 28)||(distance_right_front > 28))
            && distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE && distance_back > WALLS_MAX_DISTANCE)
    {
        turn_right_control_on_back_wall();
    }
}
void turn_right_control_on_left_wall()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    // Rotate right:
    while(!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE))
    {
        rotate_right(60);
        update_sensors_and_empty_receive_buffer();
    }
    // Need to align? //
    stop();
    _delay_ms(50);
    _delay_ms(50);
    update_sensors_and_empty_receive_buffer();
    
    while(!(distance_front > 30 && abs(distance_left_back - distance_left_front) < 1.2 && distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE))
    {
        if (distance_front < FRONT_MAX_DISTANCE || distance_left_back > WALLS_MAX_DISTANCE || distance_left_front > WALLS_MAX_DISTANCE)
        {
            if (distance_left_back > distance_left_front)
            {
                while(!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE))
                {
                    rotate_right(60);
                    update_sensors_and_empty_receive_buffer();
                }
            }
            else
            {
                while(!(distance_front > WALLS_MAX_DISTANCE && abs(distance_left_back - distance_left_front) < ABS_VALUE_RIGHT && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE))
                {
                    rotate_left(60);
                    update_sensors_and_empty_receive_buffer();
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
        update_sensors_and_empty_receive_buffer();
    }
    stop();
    _delay_ms(50);
    _delay_ms(50);
}
void turn_right_control_on_zero_walls()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    // short hard-coded rotate:
    rotate_right(60);
    for (int i = 0; i<380; i++){ _delay_ms(1);}
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    while (!( distance_front > 30 && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back > WALLS_MAX_DISTANCE))
    {
        rotate_right(60);
        update_sensors_and_empty_receive_buffer();
    }
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    rotate_right(60);
    for (int i = 0; i<150; i++){ _delay_ms(1);}
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
}
void turn_right_control_on_back_wall()
{
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    // short hard-coded rotate:
    rotate_right(60);
    for (int i = 0; i<400; i++){ _delay_ms(1);}
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    while (!( distance_front > 30 && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE && distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_back < WALLS_MAX_DISTANCE))
    {
        rotate_right(60);
        update_sensors_and_empty_receive_buffer();
    }
    
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
    
    rotate_right(60);
    for (int i = 0; i<45; i++){ _delay_ms(1);}
    //  STOPP
    stop();
    _delay_ms(50);
    _delay_ms(50);
}

// MAZE FUNCTIONS: -----------------------------------------------

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
	fwall = 1;
	lwall = 1;
	rwall = 1;
	
    if (distance_back > 25)
    {
        possible_directions |= 0x01;
    }
    if (distance_front > 25)
    {
        // open forward
        possible_directions |= 0x02;
		fwall = 0;
    }
    if ((distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE )||((distance_right_front > 28 || distance_right_back > 28) && distance_front < 15))//added distance_right_back to test
    {
        // open to right:
        possible_directions |= 0x04;
		rwall = 0;
    }
    if ((distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE)||((distance_left_front > 28 || distance_left_back > 28) && distance_front < 15))//added ditance_left_back to test
    {
        // open to left:
        possible_directions |= 0x08;
		lwall = 0;
    }
    if (possible_directions == 0x00)
    {
        possible_directions = 0x01;
    }
    return possible_directions;
}
void make_direction_decision() //OBS: added some code to try to solve if the back sensor doesn't behave well
{
    unsigned char possible_directions = get_possible_directions();
    add_to_buffer(&send_buffer, 0xF8, possible_directions);
	
	if (driven_distance > 10)
	{
		update_position();
	}
	
	in_turn = 1;
	driven_distance = 0;
	
	update_map(); 
	send_explored();
	add_unvisited();
	check_if_visited_explored();
	if (un == 0 && goal_detected == 1)
	{
		missionPhase = 2;
	}
	
	add_to_buffer(&send_buffer, 0xB1, (char) x);
	add_to_buffer(&send_buffer, 0xB2, (char) y);
	add_to_buffer(&send_buffer, 0xB3, (char) (xdir + 5)); // +5 since Komm cant send zeroes
	add_to_buffer(&send_buffer, 0xB4, (char) (ydir + 5)); // +5 since Komm cant send zeroes
	add_to_buffer(&send_buffer, 0xF1, (char) goalx);
	add_to_buffer(&send_buffer, 0xF2, (char) goaly);
	add_to_buffer(&send_buffer, 0xF3, (char) firstx);
	add_to_buffer(&send_buffer, 0xF4, (char) firsty);
	
	if (missionPhase == 1)
	{
		int8_t lx = x-ydir;
		int8_t ly = y+xdir;
		
		int8_t rx = x+ydir;
		int8_t ry = y-xdir;
		
		int8_t fx = x+xdir;
		int8_t fy = y+ydir;
		
		if(rwall && fwall && lwall)
		{
			add_to_buffer(&send_buffer, 0xF6, 'b');
			turn_back();
		}
		else if (!rwall && !explored[rx][ry])
		{
			add_to_buffer(&send_buffer, 0xF6, 'r');
			turn_right();
		}
		else if (!fwall && !explored[fx][fy])
		{
			add_to_buffer(&send_buffer, 0xF6, 'f');
		}
		else if(!lwall  && !explored[lx][ly])
		{
			add_to_buffer(&send_buffer, 0xF6, 'l');
			turn_left();
		}
		else if (!rwall && rx != startx && ry !=starty)
		{
			add_to_buffer(&send_buffer, 0xF6, 'r');
			turn_right();
		}
		else if (!fwall && fx != startx && fy !=starty)
		{
			add_to_buffer(&send_buffer, 0xF6, 'f');
		}
		else if(!lwall && lx != startx && ly !=starty)
		{
			add_to_buffer(&send_buffer, 0xF6, 'l');
			turn_left();
		}
		
		in_turn = 0;
		driven_distance = 0;
		
		turn_forward();
	}
	add_to_buffer(&send_buffer, 0x70, (char) un);	 
}
void run_direction_command(unsigned char direction_command)
{
	unsigned char possible_directions = get_possible_directions();
	add_to_buffer(&send_buffer, 0xF8, possible_directions);
	add_to_buffer(&send_buffer, 0xB1, (char) x);
	add_to_buffer(&send_buffer, 0xB2, (char) y);
	add_to_buffer(&send_buffer, 0xB3, (char) (xdir + 5)); // +5 since Komm cant send zeroes
	add_to_buffer(&send_buffer, 0xB4, (char) (ydir + 5)); // +5 since Komm cant send zeroes
	add_to_buffer(&send_buffer, 0xF1, (char) goalx);
	add_to_buffer(&send_buffer, 0xF2, (char) goaly);
	add_to_buffer(&send_buffer, 0xF3, (char) firstx);
	add_to_buffer(&send_buffer, 0xF4, (char) firsty);
	add_to_buffer(&send_buffer, 0xF6, direction_command);
	
	in_turn = 1;
	driven_distance = 0;
	
	unsigned char fail = 0;
	
	if(direction_command == 'r' && !rwall)
	{
		turn_right();
	}
	else if(direction_command == 'f' && !fwall)
	{
		;
	}
	else if(direction_command == 'l' && !lwall)
	{
		turn_left();
	}
	else if(direction_command == 'b')
	{
		turn_back();
	}
	in_turn = 0;
	driven_distance = 0;
	if(!fail)
	{
		turn_forward();	
	}
	add_to_buffer(&send_buffer, 0x70, (char) un);
}
void update_driven_distance()
{
    if (!in_turn)
    {
		if (wheel_click == 1 && wheel_click_prior == 0)
        {
            driven_distance = driven_distance + WHEEL_CLICK_DISTANCE;
		}
		    
        if (driven_distance >= 20 && (wheel_click ^ wheel_click_prior))
        {
                driven_distance = 0;
				if (missionPhase > 0)
				{
					update_position();
					add_to_buffer(&send_buffer, 0xB1, (char) x);
					add_to_buffer(&send_buffer, 0xB2, (char) y);
				}
				if (missionPhase < 2)
				{
					update_map();	
				}
        }
        wheel_click_prior = wheel_click;
    }
}
// Functions for driveable but unvisited positions
void add_unvisited()
{
	int8_t lx = x-ydir;
	int8_t ly = y+xdir;
	
	int8_t rx = x+ydir;
	int8_t ry = y-xdir;
	
	int8_t fx = x+xdir;
	int8_t fy = y+ydir;
	
	
	if (!lwall && !explored[lx][ly])
	{
		unvisited[un].x = lx;
		unvisited[un].y = ly;
		un++;
	}
	if (!fwall && !explored[fx][fy])
	{
		unvisited[un].x = fx;
		unvisited[un].y = fy;
		un++;
	}
	if (!rwall && !explored[rx][ry])
	{
		unvisited[un].x = rx;
		unvisited[un].y = ry;
		un++;
	}
}
int8_t unvisited_already_in_list(int8_t ux,int8_t uy)
{
	int8_t found = 0;
	if (un>0)
	{
		for(int i=0; i<un; i++)
		{
			if(unvisited[i].x == ux && unvisited[i].y == uy)
			{
				found = 1;
				break;
			}
		}
	} 
	return found;
}
void check_if_visited_explored()
{
	if (un>0)
	{
		for(int k=0; k<un; k++)
		{
			if(explored[unvisited[k].x][unvisited[k].y] == 1)
			{
				un--;
				if(k < un)
				{
					for (int l = k; l < un; l++)
					{
						unvisited[l] = unvisited[l+1];
					}
				}
				else
				{
					break;
				}
			}
		}
	}
}

// MISSION FUNCTIONS: --------------------------------------------
void mission()
{	
	stop();
	_delay_ms(50);
	_delay_ms(50);
	mission_phase_0();
	mission_phase_1();
	mission_phase_2();
	mission_phase_3();
	mission_phase_4();
	mission_phase_5();
	mission_phase_6();
	mission_phase_7();
}
void mission_phase_0() // Cross the start line
{
	update_sensors_and_empty_receive_buffer();
	// Too cross the start line so goal and start won't be mixed up:
	while(!start_detected)
	{
		alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
		go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior );
		update_sensors_and_empty_receive_buffer();
	}
	stop();
	driven_distance = 0;
	while (driven_distance < 10)
	{
		alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
		go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior );
		update_sensors_and_empty_receive_buffer();
	}
	driven_distance = 0;
	add_to_buffer(&send_buffer, 0xF3, (char) firstx);// firstx, firsty is the first square after the start line
	add_to_buffer(&send_buffer, 0xF4, (char) firsty);
	explored[8][7] = 1; // the square outside the start line
	missionPhase = 1;	
}
void mission_phase_1() // Explore the maze
{
	for (;;)
	{
		update_sensors_and_empty_receive_buffer();
		
		if (distance_front > 30 && distance_back > 30 &&
		((distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)||
		(distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE)||
		(distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)))
		{
			stop();
			make_direction_decision();
			if (missionPhase != 1)
			{
				break;
			}
		}
		else if(distance_front > FRONT_MAX_DISTANCE) // front > 13
		{
			// Drive forward:
			alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
			go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior );
		}
		// In some kind of turn or crossing:
		else // front < 13
		{
			stop(); // Stop, check directions and decide which way to go:
			make_direction_decision();
			if (missionPhase != 1)
			{
				break;
			}
		}
	}
};
void mission_phase_2() // Go shortest way from current square to start square
{
	point start = {startx, starty};
	point temp = {x, y};
	floodfill(temp, start);
	traceBack(costmap, start);
	getCommands(start);
	
	add_to_buffer(&send_buffer, 0xF5, (char) c);
	for (int i=0; i<c; i++)
	{
		for(;;)
		{
			update_sensors_and_empty_receive_buffer();
			
			if (distance_front > 30 && distance_back > 30 &&
			((distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)||
			(distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE)||
			(distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)))
			{
				stop();
				if(i == c - 1)
				{
					tape_detected == 0;	
				}
				run_direction_command(command[i]);
				if(i == c - 1)
				{
					while (!tape_detected)
					{
						update_sensors_and_empty_receive_buffer();
						alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
						go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior);						
					}
				}
				break;
			}
			else if(distance_front > FRONT_MAX_DISTANCE) // front > 13
			{
				// Drive forward:
				alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
				go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior);
			}
			// In some kind of turn or crossing:
			else // front < 13
			{
				stop(); // Stop, check directions and decide which way to go:
				if(i == c - 1)
				{
					tape_detected == 0;
				}
				run_direction_command(command[i]);
				if(i == c - 1)
				{
					while (!tape_detected)
					{
						update_sensors_and_empty_receive_buffer();
						alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
						go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior);
					}
				}
				break;
			}
		}	
	}
	stop();
	missionPhase = 3;
}
void mission_phase_3() // Grab the object and turn 180 degrees
{
	for (int i=0; i<400; i++)
	{
		update_sensors_and_empty_receive_buffer();
		alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
		go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior);
	}
	// TODO: Add code for grabbing object
	get_possible_directions();
	turn_back(); 
	missionPhase = 4;
}
void mission_phase_4() // Go shortest way from start to goal
{
	point start = {startx, starty};
	point goal = {goalx, goaly};
	floodfill(start, goal);
	traceBack(costmap, goal);
	getCommands(goal);
	
	add_to_buffer(&send_buffer, 0xF5, (char) c);
	for (int i=0; i<c; i++)
	{
		for(;;)
		{
			update_sensors_and_empty_receive_buffer();
			
			if (distance_front > 30 && distance_back > 30 &&
			((distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)||
			(distance_left_back > WALLS_MAX_DISTANCE && distance_left_front > WALLS_MAX_DISTANCE && distance_right_back < WALLS_MAX_DISTANCE && distance_right_front < WALLS_MAX_DISTANCE)||
			(distance_left_back < WALLS_MAX_DISTANCE && distance_left_front < WALLS_MAX_DISTANCE && distance_right_back > WALLS_MAX_DISTANCE && distance_right_front > WALLS_MAX_DISTANCE)))
			{
				stop();
				run_direction_command(command[i]);
				break;
			}
			else if(distance_front > FRONT_MAX_DISTANCE) // front > 13
			{
				// Drive forward:
				alpha = set_alpha(distance_right_back, distance_right_front, distance_left_back, distance_left_front);
				go_forward(&e, &e_prior, &e_prior_prior, &alpha, &alpha_prior, &alpha_prior_prior );
			}
			// In some kind of turn or crossing:
			else // front < 13
			{
				stop(); // Stop, check directions and decide which way to go:
				run_direction_command(command[i]);
				break;
			}
		}
	}
	stop();
	missionPhase = 5;
}
void mission_phase_5() // Back until see tape, leave object, back and turn 180 degrees
{
	missionPhase = 6;
}
void mission_phase_6() // Go shortest way from current square to start square
{
	missionPhase = 7;
}
void mission_phase_7() // Stop, also default value before start mission
{
	stop();
}