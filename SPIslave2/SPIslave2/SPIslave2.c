/*
 * Test1slave2Databuss.c
 *
 * Created: 31-03-2015
 * Author: Frida Sundberg, Markus Petersson
 * 
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>

void init_slave2(void)
{
	SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(0<<MSTR)|(0<<CPOL)|(0<<CPHA);
	DDRB = (1<<DDB6)|(1<<DDB3);
	PINB = (1<<PINB4);
	
	// Let PA be outputs for testing
	DDRA = 0xFF;
};

// todo: functions for transmit and receive

void send_to_master(volatile char send_data)
{
	SPDR = send_data;	
};


int main(void)
{
	init_slave2();
	sei();
	
    while(1)
    {
        ;
    }
}

ISR(SPI_STC_vect)
{
	PORTB = (0<<PORTB3);
	PORTB = (1<<PORTB3);
	PORTA = SPDR;
}
