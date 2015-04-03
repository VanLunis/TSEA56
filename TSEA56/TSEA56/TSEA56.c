#include <avr/io.h>
#include <avr/interrupt.h>
#include<stdio.h>
#include<stdlib.h>

double sens0[25];
int i = 25;	
double out0 = 0;

int main(void)
{
	DDRD = 0xFF ;
	
	PORTD = 0x00;
	
	
	/*
	int pcount = 0;
	int *adcselect = 0;
	
	  
	//TIMER-FREQ = CPU FREKVENS 
	TCCR0A =0; //(0<<CS02 | 0<<CS01 | 1<<CS00);
	TCCR0B =0; //(1<<CS01| 1<<CS00);/ CTC mode
	//ENABLE OVERFLOW INTERRUPT ENABLE
	//TIMSK0|=(1<<TOIE0);
	//INITIALIZE COUNTER
	TCNT0=0;// 
	OCR0A = 10;
	TCCR0A |= (1<<WGM12);
	TCCR0B= (1<<CS21|1<<CS20); 
	TIMSK0 |= (1<<OCIE0A);// enable timer compare interupt
	
	*/
	//PORTD = 0x00;

	
	/*
	char *psens1 = $1100;
	char *psens2 = $1200;
	char *psens3 = $1300;
	char *psens4 = $1400;
	//Reflex
	char *psens5 = $1500;
	char *psens6 = $1600;
	//Gyro
	char *psens7 = $1700;
	*/
	
	
	/*
	double out1 = 0;
	double out2 = 0;
	double out3 = 0;
	double out4 = 0;
	double out5 = 0;
	double out6 = 0;
	double out7 = 0;
	*/
	
	
	MCUCR = 1<<ISC01 | 1<<ISC00;
	
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	
	ADCSRA = 0 << ADIF | 1<<ADEN | 1<<ADIE | 0<<ADPS2 | 0<<ADPS1 | 0<<ADPS0 | 1<<ADATE;
	
	//DDRB = 0xFF; //enl. AVR-tutorial DDRB = 1 << PD2
	

	sei();
	
	ADCSRA = 1<<ADEN | 1<<ADSC ; // start ADC conversion
	
	while(!(ADCSRA & (1<<ADIF)))
	{
		PORTD = 0x00;
	}
	PORTD = ADCH;

	
	/*
	while(1)
	{ 
		if (ADCSRA == 1<<ADIF)
			{
				ADCSRA = 0 << ADIF;
				//PORTD = ADCH;
				PORTD  = 0xFF;
				sens0[12] = ADCL;
				PORTD = sens0[12];			
				if (i == 25)
				{
					//INSERT INSERTION SORT
					PORTD = sens0[12];
					while(i > 0){
						i--;,
						sens0[i]=0;
					}
					
				}
			}
			else{
				PORTD = 0x00;
			}
	}
	*/
	
}
	
		/*
	while(1){
		
		if (index = 25)
		{
			index = 0;
			
			*psens1 = $1100;
			*psens2 = $1200;
			*psens3 = $1300;
			*psens4 = $1400;
			*psens5 = $1500;
			*psens6 = $1600;
			*psens7 = $1700;
	
			
			//h'a'mtar medianv'a'rdet
			out0 = sens0[13];
			*out1 = $1113;
			out2 = $1213;
			out3 = $1313;
			out4 = $1413;
			out5 = $1513;
			out6 = $1613;
			out7 = $1713;
		
		}
		sens0[index] = ADCL; 
		
		*psens1++ = ADC1;
		*psens2++ = ADC2;
		*psens3++ = ADC3;
		*psens4++ = ADC4;
		*psens5++ = ADC5;
		*psens6++ = ADC6;
		*psens7++ = ADC7;
		*/
 	//}
	


ISR(TIMER0_COMPA_vect){
	//PORTD = 0x00;
		ADCSRA = 1<<ADEN | 1<<ADSC ; // start ADC conversion
}

/*
ISR(ADC_vect)
	{
		PORTD = 0xFF;
		sens0[i] = ADCL;			
		if (i == 25)
		{
			//INSERT INSERTION SORT
			PORTD = sens0[12];
			PORTD = 0xFF;
			while(i > 0){
					i--;
					sens0[i]=0;
			}
		}
			
						
	}
	*/





