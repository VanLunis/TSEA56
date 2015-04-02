/*
 * GccApplication1.c
 *
 * Created: 3/29/2015 2:45:15 PM
 *  Author: oscha927
 */ 


#include <avr/io.h>

int main(void)
{
	volatile unsigned double voltage; // �variabel f�r att spara sp�nningen i
 	DDRD = 0xff; // S�tter D pinnar till utg�ngar till Ledlamporna;
	PORTD= 0x00; // Rensa D pinnarna;
	
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) |(1<< ADPS0); // ADEN= 1 g�r s� att ADC s�tts p�. ADPS2 och ADPS0  16MHz (�ndra beroende p� vad vi vill ha h�r)
	
	ADLAR = 0; // Vill spara 10 bitar f�r att kunna representera talen i mm (1024)
	REFS1= 0;
	REFS0= 0; // refs1 och refs0 g�r s� att arefen anv�nds.0
	ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 0<<MUX0;// aktiverar ADC0- ing�ngen
	
	ADCSRA = (1<<ADSC) // Startar A/D omvandligen
	voltage = ADC // Spara resultatet 
	//ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 1<<MUX0; // aktiverar adc1 ing�ngen
	//ADMUX = 0<<MUX3 0<<MUX2 1<<MUX2 0<<MUX0; // aktiverar adc2 ing�ngen
	//ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 0<<MUX0; // aktiverar adc3 ing�ngen
	//ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 0<<MUX0; // aktiverar adc4 ing�ngen
	//ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 0<<MUX0; // aktiverar adc5 ing�ngen
	//ADMUX = 0<<MUX3 0<<MUX2 0<<MUX2 0<<MUX0; // aktiverar adc6 ing�ngen
    
        //TODO:: Please write your application code 
    
}