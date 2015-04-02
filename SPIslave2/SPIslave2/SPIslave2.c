/*
 * Test1slave2Databuss.c
 *
 * Created: 02-04-2015
 * Author: Frida Sundberg, Markus Petersson
 * 
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include "buffer.h"
volatile struct data_buffer receive_buffer;
volatile int mode = 0; // 0 receiving, 1 sending. 
volatile int transmission_status = 0;
volatile struct data_byte temp_data;
volatile int counter = 0;

void init_slave2(void)
{
	SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(0<<MSTR)|(0<<CPOL)|(0<<CPHA);
	DDRB = (1<<DDB6)|(1<<DDB3);
	PINB = (1<<PINB4);
	
	// IRQ1 and IRQ0 activated on rising edge
	EICRA = (1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(1<<ISC00);
	// Enable IRQ1 and IRQ0
	EIMSK = (1<<INT1)|(1<<INT0);
	
	// Let PA be outputs for testing
	DDRA = 0xFF;
	//Initiate the reception buffer.
	buffer_init(&receive_buffer);
	
};

// todo: functions for transmit and receive

void send_to_master(volatile char send_data)
{
	SPDR = send_data;	
};

void receive_data()
{
	if(transmission_status == 0)
	{
		temp_data.type = SPDR;
		transmission_status = 1;
	}
	else if(transmission_status == 1)
	{
		temp_data.val = SPDR;
		add_to_buffer(&receive_buffer, temp_data.type, temp_data.val);	
		transmission_status = 0;	
	}
}


int main(void)
{
	init_slave2();
	sei();
	
    while(1)
    {
		PORTA = amount_stored(&receive_buffer);
	}
}

ISR(SPI_STC_vect)
{
	counter++;
	
	PORTB = (0<<PORTB3);
	PORTB = (1<<PORTB3);		
	if(mode==0)
	{
		receive_data();
	}
}

ISR(INT0_vect)
{
	mode = 0;
}

ISR(INT1_vect)
{
	mode = 1;
}