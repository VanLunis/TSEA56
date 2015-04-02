int main(void)
{
	/*
	int pcount = 0;
	int *adcselect = 0;
	*/
	
	//TIMER-FREQ = CPU FREKVENS 
	TCCR0 = (0<<CS02 | 0<<CS01 | 1<<CS00);
	//ENABLE OVERFLOW INTERRUPT ENABLE
	TIMSK|=(1<<TOIE0);
	//INITIALIZE COUNTER
	TCNT=0;
		
	char *psens0 = $1000;
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
	
	double out0 = 0;
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
	
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0;
	
	GICR = 1<<INT0;
	
	ADCSRA = 1<<ADEN | 1<<ADIE | 0<<ADPS2 | 0<<ADPS1 | 0<<ADPS0;
	
	//DDRB = 0xFF; //enl. AVR-tutorial DDRB = 1 << PD2
	
	DDRD = 0xFF;
	
	sei();
	
	while(1){
		
	}
	
		
	while(1){
		if (psens0 = $100A)
		{
			*psens0 = $1000;
			/*
			*psens1 = $1100;
			*psens2 = $1200;
			*psens3 = $1300;
			*psens4 = $1400;
			*psens5 = $1500;
			*psens6 = $1600;
			*psens7 = $1700;
			*/
			
			//h'a'mtar medianv'a'rdet
			out0 = $1013;
			/*out1 = $1113;
			out2 = $1213;
			out3 = $1313;
			out4 = $1413;
			out5 = $1513;
			out6 = $1613;
			out7 = $1713;
			*/
		}
		*psens0++ = ADC0; 
		/*
		*psens1++ = ADC1;
		*psens2++ = ADC2;
		*psens3++ = ADC3;
		*psens4++ = ADC4;
		*psens5++ = ADC5;
		*psens6++ = ADC6;
		*psens7++ = ADC7;
		*/
 	}
	
}

ISR(TIMER0_OVF_VECT){
		ADCSRA = 1<<ADEN | 1<<ADSC ; // start ADC conversion
}

ISR(ADC_complete)
	{
		psens0++ = ADCL;			
		if (*psens0 == $1019)
		{
			//INSERT INSERTION SORT
			PORTD = $100D;
			while(psens0>=$1000){
					--*psens0 = 0;
			}
		}
			
						
		}
	}




