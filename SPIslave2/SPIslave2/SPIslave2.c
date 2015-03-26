/*
 * Test1slave2Databuss.c
 *
 * Created: 25-03-2015
 * Author: Frida Sundberg, Markus Petersson
 * 
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>

void init_slave2(void)
{
	SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(0<<MSTR)|(0<<CPOL)|(0<<CPHA);
	DDRB = (1<<DDB6);
	PINB = (1<<PINB4);
};

// todo: functions for transmit and receive

int main(void)
{
	init_slave2();
	sei();
		
	SPDR=0xFF;	
			
    while(1)
    {
        ;
    }
}

ISR(INT0_vect)
{
	;
}

ISR(SPI_STC_vect)
{
	DDRA = 0xFF;
	PORTA = SPDR;
}
