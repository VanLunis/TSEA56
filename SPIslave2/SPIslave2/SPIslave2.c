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
volatile struct data_buffer send_buffer;
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
	
	//Initiate the buffers.
	buffer_init(&receive_buffer);
	buffer_init(&send_buffer);
	
};

// todo: functions for transmit and receive

void send_to_master(struct data_buffer* my_buffer)
{
		//if Transmission not yet started: fetch type and put it in SPDR.
		if(transmission_status==0)
		{
			SPDR = fetch_from_buffer(my_buffer).type;
		}
		//if Type already sent: fetch val and put it in SPDR.
		else if(transmission_status==1)
		{
			SPDR = fetch_from_buffer(my_buffer).val;
		}
		//if the master has accepted both bytes that were sent: discard the data_byte from the buffer.
		else if(transmission_status==2)
		{
			discard_from_buffer(my_buffer);
			transmission_status=0;
		}
	
	
};

//Method to receive a data_byte. Called when SPI interrupt has occurred.
void receive_data(struct data_buffer* my_buffer)
{
	//get type.
	if(transmission_status == 0)
	{
		temp_data.type = SPDR;
	}
	//get val.
	else if(transmission_status == 1)
	{
		counter++;
		temp_data.val = SPDR;
		add_to_buffer(my_buffer, temp_data.type, temp_data.val);//add to receive_buffer when	
		transmission_status = 0;
		PORTA = counter;	
	}
}

//////////////////////////////////////////////////////////////////////
//----------------------------  MAIN -------------------------------//
//////////////////////////////////////////////////////////////////////

int main(void)
{
	init_slave2();
	sei();
	/*add_to_buffer(&send_buffer, 0xAA, 0xAA);
	add_to_buffer(&send_buffer, 0xAA, 0xAA);
	add_to_buffer(&send_buffer, 0xAA, 0xAA);
	add_to_buffer(&send_buffer, 0xAA, 0xAA);
	add_to_buffer(&send_buffer, 0xAA, 0xAA);*/
    while(1)
    {
		//PORTA = amount_stored(&receive_buffer);
	}
}

ISR(SPI_STC_vect)
{
	PORTB = (0<<PORTB3);
	PORTB = (1<<PORTB3);
	//Depending on the current mode: do things.		
	if(mode == 0)
	{
		receive_data(&receive_buffer);
	}
	else if(mode == 1)
	{
		send_to_master(&send_buffer);
	}
	transmission_status++;//Control int to remember where in the current transmission we are currently at. Should be reset when a full data_byte has been transmitted.
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
	transmission_status = 0;
	mode = 1;
	SPDR = fetch_from_buffer(&send_buffer).type;
}