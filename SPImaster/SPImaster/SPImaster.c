/*
 * Test1masterDatabuss.c
 *
 * Created: 25-03-2015
 * Author: Frida Sundberg
 * 
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

void init_master(void)
{
	// SCK=f/16
	SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(1<<SPR0);
	SPSR = (0<<SPI2X);
	DDRB = (1<<DDB7)|(1<<DDB5)|(1<<DDB4);
	PORTB = (1<<PORTB4); // Pulling SS2 high
};

void send_to_slave1(volatile char send_data)
{
	PORTB = (0<<PORTB4); // Pulling SS2 low
	SPDR = send_data;
};

// todo: functions for transmit and receive

int main(void)
{
	init_master();
	sei();
	
	// 
	send_to_slave1(0xAA);
	//
	
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
	PORTB = (1<<PORTB4); // Pulling SS2 high
	DDRA = 0xFF;
	PORTA = SPDR;
}
