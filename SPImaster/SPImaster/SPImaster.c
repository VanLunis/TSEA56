/*
 * Test1masterDatabuss.c
 *
 * Created: 31-03-2015
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

struct data_buffer slave2_buffer;

void init_master(void)
{
	// SCK=f/16
	SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0);
	SPSR = (0<<SPI2X);
	DDRB = (1<<DDB7)|(1<<DDB5)|(1<<DDB4);
	PORTB = (1<<PORTB4); // Pulling SS2 high
	
	// IRQ0 activated on rising edge
	EICRA = (1<<ISC01)|(1<<ISC00);
	// Enable IRQ0
	EIMSK = (1<<INT0);
	
	// Init data buffers in master
	buffer_init(&slave2_buffer);
	
	// Let PA be outputs for testing
	DDRA = 0xFF;
};


void send_to_slave2()
{
	struct data_byte send_byte = fetch_from_buffer(&slave2_buffer); // Fetch one byte from buffer without discarding
	transmission_status = 0; // Integer to indicate how far the send process has proceeded, updated when acknowledgment received from slave (in external interrupt) 
	volatile int local_failed_attempts = 0; 
	while(1)
	{
		PORTB = (0<<PORTB4); // Pulling SS2 low
		if(slave2_ready==1)
		{
			if(transmission_status == 0) // Ready to send .type part
			{
				send(send_byte.type);
			}
			else if(transmission_status == 1) // Ready to send .val part
			{
				send(send_byte.val);	
			}
			else if(transmission_status == 2) // Full send_byte transmitted correctly
			{
				discard_from_buffer(&slave2_buffer); // Discard byte from buffer when full transmission succeeded
				transmission_status = 0;
				break;
			}
		}
		else
		{
			failed_attempts++;
			local_failed_attempts++;
			if(local_failed_attempts > 5) // Five tries allowed before leaving function
			{
				break;
			}
		}
	}
	
};

void send(volatile char send_data)
{
	slave2_ready = 0;
	SPDR = send_data;
	_delay_us(30);
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
	
	
	for(int i=0;i<255;i++)
	{
		add_to_buffer(&slave2_buffer,i,i);
	}
	
	while(buffer_empty(&slave2_buffer)==0)
	{
		send_to_slave2();
	}
	
	PORTA = failed_attempts;
		
	while(1)
    {
       
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
