/*
 * GccApplication1.c
 *
 * Created: 3/29/2015 2:45:15 PM
 *  Author: oscha927
 */ 


#include <avr/io.h>

int main(void)
{
	volatile unsigned double voltage; // ´variabel för att spara spänningen i
 	DDRD = 0xff; // Sätter D pinnar till utgångar till Ledlamporna;
	PORTD= 0x00; // Rensa D pinnarna;
	
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) |(1<< ADPS0); // ADEN= 1 gör så att ADC sätts på. ADPS2 och ADPS0  16MHz (ändra beroende på vad vi vill ha här)
	
	ADLAR = 0; // Vill spara 10 bitar för att kunna representera talen i mm (1024)
	REFS1= 0;
	REFS0= 0; // refs1 och refs0 gör så att arefen används.0
	ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 0<<MUX0;// aktiverar ADC0- ingången
	
	ADCSRA = (1<<ADSC) // Startar A/D omvandligen
	voltage = ADC // Spara resultatet 
	//ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 1<<MUX0; // aktiverar adc1 ingången
	//ADMUX = 0<<MUX3 0<<MUX2 1<<MUX2 0<<MUX0; // aktiverar adc2 ingången
	//ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 0<<MUX0; // aktiverar adc3 ingången
	//ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 0<<MUX0; // aktiverar adc4 ingången
	//ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 0<<MUX0; // aktiverar adc5 ingången
	//ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 0<<MUX0; // aktiverar adc6 ingången
    
        //TODO:: Please write your application code 
    
}