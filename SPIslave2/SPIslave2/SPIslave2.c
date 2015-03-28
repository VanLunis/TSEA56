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
	DDRB = (1<<DDB6)|(1<<DDB3);
	PINB = (1<<PINB4);
	
	// Let PA be outputs for testing
	DDRA = 0xFF;
	// Initvalue for testing
	SPDR = 0xFF;
};

// todo: functions for transmit and receive

void send_to_master(volatile char send_data)
{
	SPDR = send_data;
	PORTB = (0<<PORTB3);
	PORTB = (1<<PORTB3);	
};


int main(void)
{
	init_slave2();
	sei();
	
	/*send_to_master(0x01);
	send_to_master(0x02);
	send_to_master(0x03);
	send_to_master(0x04);*/
	
    while(1)
    {
        ;
    }
}

ISR(SPI_STC_vect)
{
	PORTA = SPDR;
}
