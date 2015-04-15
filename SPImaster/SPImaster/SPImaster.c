/*
 * Test1masterDatabuss.c
 *
 * Last edited: 15-04-2015
 * Author: Frida Sundberg, Markus Petersson
 * 
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "buffer.h"


#define F_CPU = 16000000UL
#define delay_time 0.5

volatile int slave1_ready = 1;
volatile int slave2_ready = 1;
volatile int failed_attempts = 0;
volatile int transmission_status = 0; // 0 to send .type, 1 to send .val, 2 when both sent
volatile int counter = 0;
volatile struct data_byte temp_data;

struct data_buffer slave1_buffer;
struct data_buffer slave2_buffer;
struct data_buffer pc_buffer;

void init_master(void)
{
	// SCK=f/64
	SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR1)|(0<<SPR0);
	SPSR = (0<<SPI2X);
	DDRB = (1<<DDB7)|(1<<DDB5)|(1<<DDB4)|(1<<DDB3)|(1<<DDB0);
	PORTB =(1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
	
	// PC0 output, PD7 output (ext int to slave 2 to change receive/send-variable in slave 2)
	DDRC = (1<<DDC0);
	PORTC = (0<<PORTC0);
	
	DDRD = (1<<DDD7)|(1<<DDD6);
	PORTD = (0<<PORTD7)|(0<<PORTD6);
	
	// IRQ1 and IRQ0 activated on rising edge
	EICRA = (1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(1<<ISC00);
	// Enable IRQ1 and IRQ0
	EIMSK = (1<<INT1)|(1<<INT0);
	// IRQ0 activated on rising edge
	
	// Init data buffers in master
	buffer_init(&slave1_buffer);
	buffer_init(&slave2_buffer);
	buffer_init(&pc_buffer);
	
	// Let PA be outputs for testing
	DDRA = 0xFF;
};


void send(struct data_buffer* my_buffer, int slave)
{
	int *current_slave_ready;
	if (slave == 1)
	{
		current_slave_ready = &slave1_ready;
		PORTB = (1<<PORTB4)|(1<<PORTB3)|(1<<PORTB0);// Order slave 1 to adapt receive mode
		PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0);
	}
	else if(slave == 2)
	{
		current_slave_ready = &slave2_ready;
		PORTC = (1<<PORTC0);// Order slave 2 to adapt receive mode
		PORTC = (0<<PORTC0);	
	}
	struct data_byte send_byte = fetch_from_buffer(my_buffer); // Fetch one byte from buffer without discarding
	transmission_status = 0; // Integer to indicate how far the send process has proceeded, updated when acknowledgment received from slave (in external interrupt) 
	volatile int local_failed_attempts = 0; 
	while(1)
	{
		if (slave == 1)
		{
			PORTB =(1<<PORTB4)|(0<<PORTB3)|(0<<PORTB0); // Pulling SS1 low
		}
		else if(slave==2)
		{
			PORTB =(0<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 low	
		}
		if(*current_slave_ready==1)
		{
			if(transmission_status == 0) // Ready to send .type part
			{
				*current_slave_ready = 0;
				SPDR = send_byte.type;
				_delay_ms(delay_time);
			}
			else if(transmission_status == 1) // Ready to send .val part
			{
				*current_slave_ready = 0;
				SPDR = send_byte.val;
				_delay_ms(delay_time);	
			}
			else if(transmission_status == 2) // Full send_byte transmitted correctly
			{
				discard_from_buffer(my_buffer); // Discard byte from buffer when full transmission succeeded
				PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
				transmission_status = 0;
				break;
			}
		}
		else
		{
			failed_attempts++;
			local_failed_attempts++;
			
			if(local_failed_attempts > 1000) // tries allowed before leaving function
			{
				PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
				transmission_status = 0;
				break;
			}
		}
	}
	
};

void send_to_slave1()
{
	send(&slave1_buffer, 1);
};

void send_to_slave2()
{
	send(&slave2_buffer, 2);
};

void receive(int slave)
{
	int local_failed_attempts = 0;
	int *current_slave_ready;
	if (slave == 1)
	{
		current_slave_ready = &slave1_ready;
		PORTD =(0<<PORTD7)|(0<<PORTD6); // Order slave 1 to adapt send mode
		PORTD =(0<<PORTD7)|(1<<PORTD6);
	}
	else if(slave == 2)
	{
		current_slave_ready = &slave2_ready;
		PORTD = (0<<PORTD7)|(0<<PORTD6); // Order slave 2 to adapt send mode
		PORTD = (1<<PORTD7)|(0<<PORTD6);
	}
	transmission_status=0;
	while(1)
	{
		//Select the right slave.
		if (slave == 1)
		{
			PORTB =(1<<PORTB4)|(0<<PORTB3)|(0<<PORTB0); // Pulling SS1 low
		}
		else if(slave==2)
		{
			PORTB =(0<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 low
		}
		//check if the current slave is ready to send. The slave_ready status is set through acknowledgments.
		if(*current_slave_ready == 1)
		{
			//if statement to check where in the transmission process we are currently at.
			if(transmission_status == 0)
			{
				*current_slave_ready = 0;
				SPDR = 0x00;//random data to send to slave, since the slave is sending it does not care what it gets.
				_delay_ms(delay_time);
			}
			else if(transmission_status == 1)
			{
				temp_data.type = SPDR;//read the data that the slave has sent. Note that SPDR should be read here and not where transmission_status==0 
									  //to make sure that we have gotten an acknowledgment from the slave
				*current_slave_ready = 0;
				SPDR = 0x00;//random data to send to slave, since the slave is sending it does not care what it gets.
				_delay_ms(delay_time);
				
			}
			else if(transmission_status==2)
			{
				temp_data.val = SPDR; //read the data that the slave has sent.
				transmission_status = 0;
				PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
				if(temp_data.type == 0 && temp_data.val == 0) //if we have received a null_data_byte: ignore.
				{
					
				}
				else
				{
					if (slave == 1)
					{
						add_to_buffer(&slave2_buffer, temp_data.type, temp_data.val);
					}
					_delay_ms(delay_time);
					add_to_buffer(&pc_buffer, temp_data.type, temp_data.val); //otherwise add to buffer!	
				}
				_delay_ms(delay_time);
				break;
			}
		}
		else //Safety net so that we don't get stuck in a never ending loop if something with the transmission goes to... hell.
		{
			local_failed_attempts++;
			
			if(local_failed_attempts > 1000) // tries allowed before leaving function
			{
				PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
				transmission_status = 0;
				break;
			}
		}
	}
};

void receive_from_slave1()
{
	receive(1);
};

void receive_from_slave2()
{
	receive(2);
};

//////////////////////////////////////////////////////////////////////
//----------------------------  MAIN -------------------------------//
//////////////////////////////////////////////////////////////////////

int main(void)
{
	init_master();
	sei();	

///////////////////////// TEST ////////////////////////////////////////		
	for (int i=1; i<3; i++)
	{
		receive_from_slave1();
	}

	while(buffer_empty(&slave2_buffer) == 0)
	{	
		send_to_slave2();
	}
///////////////////////////////////////////////////////////////////////	
		
	while(1)
    {
		PORTA = amount_stored(&slave2_buffer);
    }
}

ISR(INT0_vect)
{
	slave2_ready = 1;
	transmission_status++;
}

ISR(INT1_vect)
{
	slave1_ready = 1;
	transmission_status++;
}

ISR(SPI_STC_vect)
{	
	PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
}
