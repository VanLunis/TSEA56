/*
 * Test1masterDatabuss.c
 *
 * Created: 02-04-2015
 * Author: Frida Sundberg, Markus Petersson
 * 
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "buffer.h"
#define F_CPU = 16000000UL

volatile int slave2_ready = 1;
volatile int failed_attempts = 0;
volatile int transmission_status = 0; // 0 to send .type, 1 to send .val, 2 when both sent
volatile int counter=0;

struct data_buffer slave2_buffer;
struct data_buffer receive_buffer;

void init_master(void)
{
	// SCK=f/16
	SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0);
	SPSR = (0<<SPI2X);
	DDRB = (1<<DDB7)|(1<<DDB5)|(1<<DDB4);
	PORTB = (1<<PORTB4); // Pulling SS2 high
	
	// PC0 output, PD7 output (ext int to slave 2 to change receive/send-variable in slave 2)
	DDRC = (1<<DDC0);
	DDRD = (1<<DDD7);
	
	// IRQ0 activated on rising edge
	EICRA = (1<<ISC01)|(1<<ISC00);
	// Enable IRQ0
	EIMSK = (1<<INT0);
	
	// Init data buffers in master
	buffer_init(&slave2_buffer);
	buffer_init(&receive_buffer);
	
	// Let PA be outputs for testing
	DDRA = 0xFF;
};

void send_to_slave2()
{
	send(&slave2_buffer, 2);
};

void send(struct data_buffer* my_buffer, int slave)
{
	int *current_slave_ready;
	if(slave == 2)
	{
		current_slave_ready = &slave2_ready;
		PORTC = (0<<PORTC0); // Order slave 2 to adapt receive mode
		PORTC = (1<<PORTC0);	
	}
	struct data_byte send_byte = fetch_from_buffer(my_buffer); // Fetch one byte from buffer without discarding
	transmission_status = 0; // Integer to indicate how far the send process has proceeded, updated when acknowledgment received from slave (in external interrupt) 
	volatile int local_failed_attempts = 0; 
	while(1)
	{
		if(slave==2)
		{
			PORTB = (0<<PORTB4); // Pulling SS2 low	
		}
		if(*current_slave_ready==1)
		{
			if(transmission_status == 0) // Ready to send .type part
			{
				*current_slave_ready = 0;
				SPDR = send_byte.type;
				_delay_us(30);
			}
			else if(transmission_status == 1) // Ready to send .val part
			{
				*current_slave_ready = 0;
				SPDR = send_byte.val;
				_delay_us(30);	
			}
			else if(transmission_status == 2) // Full send_byte transmitted correctly
			{
				discard_from_buffer(my_buffer); // Discard byte from buffer when full transmission succeeded
				counter++;
				PORTA = counter;
				transmission_status = 0;
				break;
			}
		}
		else
		{
			failed_attempts++;
			local_failed_attempts++;
			if(local_failed_attempts > 10) // Ten tries allowed before leaving function
			{
				transmission_status = 0;
				break;
			}
		}
	}
	
};


/*
void receive_from_slave2(void)
{
	send_to_slave2(0xAA);	
};*/

int main(void)
{
	init_master();
	sei();
	
	
	for(int i=1; i<32; i++)
	{
		add_to_buffer(&slave2_buffer,i,i);
	}
	
	while(buffer_empty(&slave2_buffer)==0)
	{	
		send_to_slave2();
	}
	
		
	while(1)
    {
		;
    }
}

ISR(INT0_vect)
{
	slave2_ready = 1;
	transmission_status++;
}

ISR(SPI_STC_vect)
{	
	PORTB = (1<<PORTB4); // Pulling SS2 high
}
