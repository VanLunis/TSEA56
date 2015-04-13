/*
 * sensormodul.c
 *
 * Created: 4/13/2015 4:56:37 PM
 *  Author: aleer686
 */ 

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

//Sp�nningar associerade med avst�nden nedan f�r de olika IR-senorerna. (En rad/sensor)
double lookuptable[5][29] = {
	
	{3.019, 2.46, 2.18, 2.771, 1.61, 1.438, 1.27, 1.143, 1.11, 1.01, 0.91, 0.84, 0.82, 0.791, 0.75, 0.698, 0.655, 0.615, 0.585, 0.550, 0.530, 0.510, 0.506, 0.502, 0.49, 0.485, 0.46, 0.43, 0},
	
	{3.01, 2.4, 2.07, 1.77, 1.65, 1.43, 1.28, 1.14, 1.1, 0.99, 0.9, 0.83, 0.82, 0.79, 0.735, 0.69, 0.645, 0.6, 0.570, 0.545, 0.52, 0.505, 0.5, 0.49, 0.47, 0.45, 0.43, 0.415, 0},
	
	{2.95, 2.4, 2.05, 1.75, 1.6, 1.42, 1.25, 1.14, 1.1, 1.0, 0.89, 0.83, 0.815, 0.79, 0.72, 0.675, 0.640, 0.61, 0.58, 0.54, 0.52, 0.51, 0.505, 0.49, 0.48, 0.46, 0.45, 0.44, 0},
	
	{3.05, 2.5, 2.12, 1.75, 1.6, 1.44, 1.3, 1.15, 1.11, 1.01, 0.92, 0.83, 0.82, 0.80, 0.75, 0.69, 0.65, 0.59, 0.57, 0.55, 0.52, 0.505, 0.5, 0.49, 0.47, 0.45, 0.43, 0.42, 0},
	
	{2.95, 2.5, 2.1, 1.77, 1.64, 1.44, 1.3, 1.14, 1.11, 1.07, 0.95, 0.89, 0.84, 0.83, 0.81, 0.76, 0.70, 0.65, 0.65, 0.64, 0.63, 0.5, 0.49, 0.48, 0.48, 0.47, 0.44, 0.41, 0}
	
};

//Avst�nd tillh�rande IR-sensorernas sp�nningar, i enheten 0.2cm
int distances[33] = {15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180};
int median_amount = 9;
double input[5][9];

//funktion som sorterar en m�ngd v�rden i en array och returnerar medianen.
double median(int n, double value[]){
	double temp;
	
	//Sorterar
	for(int i=1;i<n;i++){
		for(int j=0;j<n-i;j++){
			if(value[j]>=value[j+1])
			{
				temp=value[j];
				value[j]=value[j+1];
				value[j+1]=temp;
			}
		}
	}
	//returnerar median
	return value[4];
}
	
	
//Funktion som returnerar ett linj�rinterpolerat avst�nd f�r insp�nningar (f�r IR-sensorer). Beh�ver f�rutom sp�nning �ven sensorindex.
double lookup (double voltage, int sensorindex){ 
	int i;
    double p =0;

//Stegar igenom tabellen tills vi hittar ett v�rde som �r mindre �n v�r insp�nning
while(voltage <= lookuptable[sensorindex][i])
    {
        i++;
    }
  
	//Om vi l�ter v�r insp�nning vara en linj�rkombination av en den st�rsta mindre och en den minsta st�rre sp�nning som existerar
	//i lookuptable �r p andelen av den st�rre och p-1 andelen av den mindre som kr�vs f�r att f� insp�nningen.
	//Allts�: Insp�nning = p*V_st�rre+(1-p)*V_mindre.
	//Avst�ndet till identifierat objekt �r d� 
	//	p*D_st�rre+(1-p)*D_mindre, d�r D �r de avst�nd �r de avst�nd som h�r ihop med sp�nningarna.
    p = (lookuptable[sensorindex][i-1] - voltage)/(lookuptable[sensorindex][i-1]-lookuptable[sensorindex][i]);
    return (p*distances[i-1])+(1-p)*distances[i];


}

void read(){
	//L�ser fr�n ADC0/Sensor1
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0; 	
	for (int i=0; i<median_amount; i++){
		//Nollst�ller ADIF och startar konvertering
		ADCSRA |= (1<<ADIF) | (1<<ADSC); 
		//V�ntar, s� att vi inte g�r in i ny loop och f�rs�ker starta
		//ni konvertering innan den ovan �r f�rdig
		_delay_ms(15);
		input[0][i] = ((double)ADCH*4.94/256);
	}
	
	// ADC1/Sensor2
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(15);
		input[1][i] = ((double)ADCH*4.94/256);
	}
	
	// ADC2/Sensor3
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 1<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(15);
		input[2][i] = ((double)ADCH*4.94/256);
	}
	
	// ADC3/Sensor4
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 1<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(15);
		input[3][i] = ((double)ADCH*4.94/256);
	}
	
	// ADC4/Sensor5
	ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(15);
		input[4][i] = ((double)ADCH*4.94/256);
	}
}

void send(){
	double voltage[8];
	uint8_t output[8];
	
	for (int i=0; i<8; i++){
		voltage[i] = median(median_amount, i);
	}
	
	for (int i=0; i<5; i++){
		output[i] = (uint8_t)lookup(voltage[i], i);
		PORTD = output[i];
	}
	
	//hjultejpsensor, returnerar 1 d� tejp hittas (tejp ger utsp�nning ~4.3V)
	if (voltage[5] < 4.4){
		output[5] = 1;
	}
	else{
		output[5] = 0;
	}
	
	//Golv-tejpsensor, returnerar 1 d� tejp hittas (tejp ger utsp�nning ~4.3V)
	if (voltage[6] > 4.2){
		output[6] = 1;
		}
	else{
		output[6] = 0;
		}
		
	//gyro, returnerar 4.5V d� 300grad/sek och 0.5V d� -300grad/sek
	output[7] = ((uint8_t)(300*(voltage[7]-2.5)/2));
	
}


int main(void)
{
	DDRD = 0xFF;
 	DDRB = 0xFF;
	 
	PORTD = 0xFF;
	 
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;	
	ADCSRA = 0 << ADIF | 1<<ADEN | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0 | 0<<ADATE;
	
	ADCSRA = (1<<ADEN) | (1<<ADSC) | 7 ;
	 
    while(1)
    {
        if (ADCSRA & (1<<ADIF))
        {
			read();
			send();
		}
    }
}

