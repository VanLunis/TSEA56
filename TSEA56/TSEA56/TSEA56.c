#include <avr/io.h>
#include <avr/interrupt.h>
#include<stdio.h>
#include<stdlib.h>
#include <stdio.h>
#include <math.h>
#include<stdlib.h>
#include<avr/delay.h>
#include<util/delay.h>
#define F_CPU 1000000UL

double lookuptable[5][29] = {
   
    {3.019, 2.46, 2.18, 2.771, 1.61, 1.438, 1.27, 1.143, 1.11, 1.01, 0.91, 0.84, 0.82, 0.791, 0.75, 0.698, 0.655, 0.615, 0.585, 0.550, 0.530, 0.510, 0.506, 0.502, 0.49, 0.485, 0.46, 0.43, 0},
   
    {3.01, 2.4, 2.07, 1.77, 1.65, 1.43, 1.28, 1.14, 1.1, 0.99, 0.9, 0.83, 0.82, 0.79, 0.735, 0.69, 0.645, 0.6, 0.570, 0.545, 0.52, 0.505, 0.5, 0.49, 0.47, 0.45, 0.43, 0.415, 0},
   
    {2.95, 2.4, 2.05, 1.75, 1.6, 1.42, 1.25, 1.14, 1.1, 1.0, 0.89, 0.83, 0.815, 0.79, 0.72, 0.675, 0.640, 0.61, 0.58, 0.54, 0.52, 0.51, 0.505, 0.49, 0.48, 0.46, 0.45, 0.44, 0},
   
    {3.05, 2.5, 2.12, 1.75, 1.6, 1.44, 1.3, 1.15, 1.11, 1.01, 0.92, 0.83, 0.82, 0.80, 0.75, 0.69, 0.65, 0.59, 0.57, 0.55, 0.52, 0.505, 0.5, 0.49, 0.47, 0.45, 0.43, 0.42, 0},
    
    {2.95, 2.5, 2.1, 1.77, 1.64, 1.44, 1.3, 1.14, 1.11, 1.07, 0.95, 0.89, 0.84, 0.83, 0.81, 0.76, 0.70, 0.65, 0.65, 0.64, 0.63, 0.5, 0.49, 0.48, 0.48, 0.47, 0.44, 0.41, 0}
    
};

//För tester

uint8_t outputarray[8];
	double sensorvalue[8][9];
	uint8_t input;
	double tempvoltage;
	uint8_t binaryout;

	



int realdistance[33] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};
int modifieddistance[33] = {15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180};
int lookup(double voltage, int sensorindex);



double cmp(const void* a, const void* b)
{
    double va = *(const double*) a;
    double vb = *(const double*) b;
    return (va > vb) - (va < vb);
}


double medianera(int n, double svalue[]){ // Får in en vektor, sorterar den och returnerar medianen = voltage
    double t;
	//int vectorindex;
    //qsort (sensorvalue, 7, sizeof(double), cmp ); // sortera vektor---- lägg till efter test 
  
 
   for(int i=1;i<n;i++){
	  for(int j=0;j<n-i;j++){
			//PORTD=sensorvalue[j];
			//PORTD= sensorvalue[j+1];
		 if(svalue[j]>=svalue[j+1])
	     {
	      t=svalue[j];
	      svalue[j]=svalue[j+1];
	      svalue[j+1]=t;
	     }
		  //PORTD=sensorvalue[j];
		  //PORTD= sensorvalue[j+1];
	  }
   }
  
  
  
  
  
  
   //------------------Test---------------------------------
    
    // printf("Testsensorvektor\n");
    
    //for ( vectorindex = 0; vectorindex < 8; vectorindex++ )// testa sensormatrisen
    //{
      
       
     //       printf("sensor[%d] = %f\n", vectorindex,sensorvalue[vectorindex] );
    //}
     //-----------------------------------------------------------
     

    double median = svalue[4];// ta fram medianen
   // printf("medianen är = %f\n", median);
    return median; // returnera medianen
    
    }



double lookup (double voltage, int sensorindex){ // tar in en spänning (median) och index på sensornc och returnerar avståndet i form av 0,2 cm mellanrum
  // int vectorindex = 0;
    int i=0;
    double p =0;
    double distance=0;
    double distance1= 0;
    double distance2=0;
    double distance3=0;
 
    //------------------Test-----------------------------------
   // printf("voltage is %f\n", voltage);
   
    //printf("Lookuptable\n");
    //for ( vectorindex = 0; vectorindex < 33; vectorindex++ )// testa och skriver ut lookupvektorn för sensorn
    //{
        
        
      //  printf("sensor[%d] = %f\n", vectorindex, lookuptable[sensorindex][vectorindex]);
   //}
    //-----------------------------------------------------------
     
    
   while(voltage <= lookuptable[sensorindex][i])
    {
        

        i++;
       // printf("index i är %d\n",i);
            }
  
   
    
    int indexet = i;
    p = (lookuptable[sensorindex][i-1] - voltage)/(lookuptable[sensorindex][i-1]-lookuptable[sensorindex][i]);
    distance1 = (p*modifieddistance[indexet-1]);
    distance2 = (1-p)*modifieddistance[indexet];
    distance = distance1+distance2;
    //distance = (p* modifieddistance[i-1])+((1-p)* modifieddistance[i]));
   // printf("Det modifierade avståndet är: %f\n", distance);
    return distance;
}




int main(void)
{
	
	double voltage[8];
	double out;
	int sensorindex = 0;
	
 	DDRD = 0xFF ;
	DDRB = 0xFF;
	
	PORTD = 0x00;
	PORTD = 0xFF;
	PORTD= 0x00;
	
	PORTB= 0x00;
	
	//int pcount = 0;
	//int *adcselect = 0;
	
	  
	//TIMER-FREQ = CPU FREKVENS 
	TCCR0A =0; //(0<<CS02 | 0<<CS01 | 1<<CS00);
	TCCR0B =0; //(1<<CS01| 1<<CS00);/ CTC mode prescaler sätter tiden
	//ENABLE OVERFLOW INTERRUPT ENABLE
	//TIMSK0|=(1<<TOIE0);
	//INITIALIZE COUNTER
	TCNT0=0;// räknas upp eller ner beroende på vilken timer som används
	OCR0A = 10;
	TCCR0A |= (1<<WGM02);// Avgör hur timern ska arbeta timer sequence
	TCCR0B= (1<<CS21|1<<CS20);// Avgör hur timern ska arbeta timer sequence
	TIMSK0 |= (1<<OCIE0A);// enable timer compare interupt talar om att avbrott ska genbereras 
	
	
	


	
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
	
	
	//MCUCR = 1<<ISC01 | 1<<ISC00;
	
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	
	ADCSRA = 0 << ADIF | 1<<ADEN | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0 | 0<<ADATE;
	

	

	//sei();
	
	ADCSRA = (1<<ADEN) | (1<<ADSC) | 7 ; // start ADC conversion
	/*
	while(!(ADCSRA & (1<<ADIF))) // När adc omvandlingen är klar
	{
		PORTD = 0x00;
	}
	PORTD = ADCH;

	*/
	
	while(1)
	{ 
		if (ADCSRA & (1<<ADIF))
			{
				
			
			
			
		
			
				
				ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;//ADC0
				//_delay_ms(1);
				for(int kolumn= 0; kolumn<9; kolumn++){	
					ADCSRA |= (1<<ADIF) | (1<<ADSC); // ADC0
					_delay_ms(15);				
					//PORTD = ADCH; //ADC0
					input= ADCH;
					tempvoltage= ((double)input*4.94/256);
					sensorvalue[0][kolumn]= tempvoltage;
					//PORTD=0x00;
					//PORTB = sensorvalue[0][kolumn];
				}
		
				ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;//ADC1
				//_delay_ms(1);
				for(int kolumn= 0; kolumn<9; kolumn++){	
					ADCSRA |= (1<<ADIF) | (1<<ADSC); // ADC1
					_delay_ms(15);
					//PORTD = ADCH; //ADC1
					input = ADCH;
					//PORTB=input;
					
					tempvoltage= ((double)input*4.94/256);
					sensorvalue[1][kolumn]= tempvoltage;
					//PORTD=0x00;
					//PORTB = sensorvalue[1][kolumn];
				}
				
				ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 1<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;//ADC2
			    // _delay_ms(1);
				for(int kolumn= 0; kolumn<9; kolumn++){	
					ADCSRA |= (1<<ADIF) | (1<<ADSC); // ADC2
					_delay_ms(15);
					input= ADCH;
					//PORTD=input;
					tempvoltage= ((double)input*4.94/256);
					sensorvalue[2][kolumn]= tempvoltage;
					//PORTD = ADCH; //ADC2
					//PORTB = sensorvalue[2][kolumn];
				}
				
				ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 1<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;//ADC3
				//_delay_ms(1);
				for(int kolumn= 0; kolumn<9; kolumn++){	
					ADCSRA |= (1<<ADIF) | (1<<ADSC); // ADC3
					_delay_ms(15);
					input= ADCH;
					//PORTD=input;
					tempvoltage= ((double)input*4.94/256);
					sensorvalue[3][kolumn]= tempvoltage;
					//PORTD = ADCH; //ADC3
				}
				
				ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;//ADC4
				//_delay_ms(1);
				for(int kolumn= 0; kolumn<9; kolumn++){	
					ADCSRA |= (1<<ADIF) | (1<<ADSC); // ADC4
					_delay_ms(15);
					input= ADCH;
					//PORTD=input;
					tempvoltage= ((double)input*4.94/256);
					sensorvalue[4][kolumn]= tempvoltage;
					PORTD= sensorvalue[4][kolumn];
					//PORTD = ADCH; //ADC4
				} 
				
				ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 0<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;//ADC5 - REFLEX
				for (int i=0; i<9; i++){
					ADCSRA |= (1<<ADIF) | (1<<ADSC);
					_delay_ms(15);
					sensorvalue[5][i] = ((double)ADCH*4.94/256);
				}
				
				ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 1<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;//ADC6 - REFLEX
				for (int i=0; i<9; i++){
					ADCSRA |= (1<<ADIF) | (1<<ADSC); 
					_delay_ms(15);
					sensorvalue[6][i] = ((double)ADCH*4.94/256);
				}
				
				ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;//ADC7 - GYRO
				for (int i=0; i<9; i++){
					ADCSRA |= (1<<ADIF) | (1<<ADSC); 
					_delay_ms(15);
					sensorvalue[7][i] = ((double)ADCH*4.94/256);
				}
				

				
				
			
				//PORTB=0x00;
				//PORTD=0x00;
				
				
				
				
				for(int sindex=0; sindex<8; sindex++){
 					voltage[sindex] = medianera(9, sensorvalue[sindex]);
				}
				for(int sindex=0; sindex<5; sindex++){
					out = (double)lookup(voltage[sindex], sindex);
					binaryout = (uint8_t)out;
					outputarray[sindex] = binaryout;
					PORTD= binaryout;		
				}
				
				//hjultejpsensor, returnerar 1 då tejp hittas (tejp ger utspänning ~4.3V)
				if (voltage[5] < 4.4){
					outputarray[5] = 1;
				}
				else{
					outputarray[5] = 0;
				}
				//Golv-tejpsensor, returnerar 1 då tejp hittas (tejp ger utspänning ~4.3V)
				if (voltage[6] > 4.2)
					outputarray[6] = 1;
				}
				else{
					outputarray[6] = 0;
				}
				//gyro, returnerar 4.5V då 300grad/sek och 0.5V då -300grad/sek
				outputarray[7] = ((uint8_t)(300*(voltage[7]-2.5)/2));
				
				
				for (int i = 0; i < 5; i++){
					for (int j = 0; j < 9; j++){
						sensorvalue[i][j] = 0;
					}
				}
				//PORTD = gyro;
				
				
				
					
				
				}
		
			
			
				
			
			
		}
				//PORTD  = 0xCC;
				//sens0[12] = ADCL;
			//	PORTD = sens0[12];			
			//	if (i == 25)
			//	{
					//INSERT INSERTION SORT
				//	PORTD = sens0[12];
				//	while(i > 0){
				//		i
				//		sens0[i]=0;
				//	}
					
				
			//}
			
			
	
	
	

	
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
	


//ISR(TIMER0_COMPA_vect){
	//PORTD = 0x00;
		//ADCSRA = 1<<ADEN | 1<<ADSC ; // start ADC conversion
//}

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
		}mnkl
			
						
	}
	*/