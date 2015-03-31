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
#define F_CPU = 16000000UL

volatile int slave_ready = 1;
volatile int failed_attempts = 0;

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
	
	// Let PA be outputs for testing
	DDRA = 0xFF;
};

void send_to_slave2(volatile char send_data)
{
	slave_ready = 0;
	PORTB = (0<<PORTB4); // Pulling SS2 low
	SPDR = send_data;
	_delay_us(20);
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
	
	for (int i=0; i<255; i++)
	{
		if(slave_ready==1)
		{
			send_to_slave2(i);	
		}
		else
		{
			failed_attempts++;
		}
	}
	
	PORTA = failed_attempts;
		
	while(1)
    {
       
    }
}

ISR(INT0_vect)
{
	slave_ready = 1;
}

ISR(SPI_STC_vect)
{	
	PORTB = (1<<PORTB4); // Pulling SS2 high
}
