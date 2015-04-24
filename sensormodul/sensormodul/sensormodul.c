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
#define M_PI 3.14159265358979323846
//Spänningar associerade med avstånden nedan för de olika IR-senorerna. (En rad/sensor)
double lookuptable[5][50] = {
	
	//{3.019, 2.46, 2.18, 2.771, 1.61, 1.438, 1.27, 1.143, 1.11, 1.01, 0.91, 0.84, 0.82, 0.791, 0.75, 0.698, 0.655, 0.615, 0.585, 0.550, 0.530, 0.510, 0.506, 0.502, 0.49, 0.485, 0.46, 0.43, 0},
	
//	{3.01, 2.4, 2.07, 1.77, 1.65, 1.43, 1.28, 1.14, 1.1, 0.99, 0.9, 0.83, 0.82, 0.79, 0.735, 0.69, 0.645, 0.6, 0.570, 0.545, 0.52, 0.505, 0.5, 0.49, 0.47, 0.45, 0.43, 0.415, 0},
	{3.014, 2.684, 2.268, 1.959, 1.724, 1.540, 1.369, 1.257, 1.145, 1.087, 0.991, 0.934, 0.865, 0.819, 0.742, 0.755, 0.717, 0.679, 0.640, 0.621, 0.582, 0.562, 0.543, 0.510, 0.488, 0.469, 0.456, 0.444, 0.424, 0.405, 0.401, 0.386, 0},
	{3.020, 2.960,  2.411, 2.043, 1.803, 1.495, 1.392, 1.250, 1.139, 1.059, 0.965, 0.903, 0.849, 0.794, 0.731, 0.740, 0.704, 0.672, 0.634, 0.598, 0.577, 0.557, 0.536, 0.506, 0.490, 0.480, 0.462, 0.441, 0.425, 0.419, 0.402, 0.387, 0},
	
	{2.543, 2.351, 2.135, 1.976, 1.849, 1.740, 1.644, 1.552, 1.496, 1.423, 1.352, 1.292, 1.254, 1.197, 1.160, 1.122, 1.079, 1.041, 1.022, 0.984, 0.965, 0.927, 0.908, 0.888, 0.869, 0.847, 0.831, 0.812, 0.793, 0.775, 0.755, 0.736, 0.716, 0},
	
	{3.105, 2.735, 2.372, 2.081, 1.790, 1.589, 1.401, 1.271, 1.158, 1.082, 0.996, 0.919, 0.842, 0.804, 0.746, 0.708, 0.666, 0.627, 0.589, 0.569, 0.546, 0.510, 0.491, 0.471, 0.452, 0.432, 0.413, 0.394, 0.391, 0.374, 0.373, 0},
																																														
	{3.033, 2.721, 2.106, 1.794, 1.560, 1.404, 1.269, 1.155, 1.073, 0.977, 0.919, 0.847, 0.802, 0.748, 0.700, 0.660, 0.623, 0.602, 0.565, 0.544, 0.523, 0.497, 0.473, 0.466, 0.447, 0.428, 0.422, 0.406, 0.388, 0.384, 0.365, 0}
	
};

/* {
	
	{3.06, 2.25, 1.87, 1.691, 1.503, 1.351. 1.239, 1.128, 1.051. 0.974. 0.917. 0.854. 0.803, 0.742, 0.701, 0.662, 0.628, 0.603. 0.575, 0.544, 0.525, 0.506, 0.486, 0.466, 0.445, 0.426, 0.412, 0.407, 0.388, 0.380, 0};
	{3.042, 2.13, 1.643, 1.606. 1.404. 1.287. 1.213, 1.079, 1.023, 0.909, 0.86, 0.814, 0.77, 0.731, 0.693, 0.654, 0.635, 0.596, 0.576, 0.547, 0.520, 0.502, 0.485, 0.463, 0.449, 0.440, 0.420, 0.410, 0.402, 0.384, 0}
	// Från 10.
	{5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 2.551, 2.350, 2.169, 2.045, 2.012, 1.887, 1.756, 1.661, 1.569, 1.495, 1.492, 1.365, 1.308, 1.253, 1.215, 1.158, 1.120, 1.077, 1.039, 1.020, 0.982, 0.957, 0.925, 0.905, 0.868, 0.848, 
		0.829, 0.810, 0.791, 0.773, 0.753, 0.740, 0.734, 0.715, 0.705, 0.695, 675, 0.675, 0}
		
	}
*/

//Avstånd tillhörande IR-sensorernas spänningar, i enheten 0.2cm
int distances[36] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 180, 185};
//Antalet värden som skall användas för median-bildning. 
int median_amount = 9;
//Array för att lagra input från IR-sensorer, är en spänning.  
double input[5][9];

double AVCC = 5.03;

//funktion som sorterar en mängd värden i en array och returnerar medianen.
double median(double value[]){
	double temp;
	
	//Bubbelsortering
	for(int i=1; i<median_amount; i++){
		for(int j=0; j<median_amount-i; j++){
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
	
	
//Funktion som returnerar ett linjärinterpolerat avstånd för inspänningar (för IR-sensorer). Behöver förutom spänning även sensorindex.
double lookup (double voltage, int sensorindex){ 
	int i;
    double p =0;

//Stegar igenom tabellen tills vi hittar ett värde som är mindre än vår inspänning
while(voltage <= lookuptable[sensorindex][i])
    {
        i++;
    }
  
	//Om vi låter vår inspänning vara en linjärkombination av en den största mindre och en den minsta större spänning som existerar
	//i lookuptable är p andelen av den större och p-1 andelen av den mindre som krävs för att få inspänningen.
	//Alltså: Inspänning = p*V_större+(1-p)*V_mindre.
	//Avståndet till identifierat objekt är då 
	//	p*D_större+(1-p)*D_mindre, där D är de avstånd är de avstånd som hör ihop med spänningarna.
    p = (lookuptable[sensorindex][i-1] - voltage)/(lookuptable[sensorindex][i-1]-lookuptable[sensorindex][i]);
    return (p*distances[i-1])+(1-p)*distances[i];
}

void read(){
	//Läser från ADC0/Sensor1 - IR

   	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	_delay_ms(5);
	for (int i=0; i<median_amount; i++){
		
		//Nollställer ADIF och startar konvertering
		ADCSRA |= (1<<ADIF) | (1<<ADSC); 
		
		//Väntar, så att vi inte går in i ny loop och försöker starta
		//ni konvertering innan den ovan är färdig
		_delay_ms(0.5);
		
		input[0][i] = ((double)ADCH*AVCC/256);
	}
	
	//
	
	// ADC1/Sensor2 - IR
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;
	_delay_ms(5);
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(0.5);
		input[1][i] = ((double)ADCH*AVCC/256);
	}
	
	// ADC2/Sensor3 - IR
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 1<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	_delay_ms(5);
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(0.5);
		input[2][i] = ((double)ADCH*AVCC/256);
	}
	
	// ADC3/Sensor4 - IR
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 1<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;
	_delay_ms(5);
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(0.5);
		input[3][i] = ((double)ADCH*AVCC/256);
	}
	
	// ADC4/Sensor5 - IR
	ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	_delay_ms(5);
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(0.5);
		input[4][i] = ((double)ADCH*AVCC/256);
	}
	
	// ADC5/Sensor6 - reflexsensor
	ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 0<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;
	_delay_ms(5);
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(0.5);
		input[5][i] = ((double)ADCH*AVCC/256);
	}
	
	// ADC6/Sensor7 - reflexsensor
	ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 1<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	_delay_ms(5);
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(0.5);
		input[6][i] = ((double)ADCH*AVCC/256);
	}
	
		// ADC5/Sensor6 - gyro
	ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 0<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;
	_delay_ms(5);
	for (int i=0; i<median_amount; i++){
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		_delay_ms(0.5);
		input[7][i] = ((double)ADCH*AVCC/256);
	}
}

void send(){
	double voltage[8];
	uint8_t output[10];
	int waswhite;
	double distance;
	double alpha[2];
	uint8_t sign[2];
	PORTD = 0x00;
	
	for (int i=0; i<8; i++){
		voltage[i] = median(input[i]);
	}
	
	for (int i=0; i<5; i++){
		output[i] = (uint8_t)lookup(voltage[i], i);
		//PORTD = output[i];
	}
	
	//Hjultejpsensor, returnerar längd då tejp hittas (svart ger utspänning ~3.9V, ljusgrå ger ~0.2V )
	if (voltage[5] > 2 && waswhite){
		distance += 2*3.1*3.14159/8;
		if (distance >= 40)
		{
			distance = 0;
		}
		output[5] = distance;
		waswhite = 0;
	}
	else if (voltage < 1)
	{
	 	waswhite = 1;
	}
	
	//Golv-tejpsensor, returnerar 1 då tejp hittas (tejp ger utspänning ~4.3V, golv oklart)
	if (voltage[6] >= 4){
		output[6] = 1;
		} 
	else{
		output[6] = 0;
	}
		

	

	
	//gyro, returnerar 4.5V då 300grad/sek och 0.5V då -300grad/sek
	output[7] = ((uint8_t)(300*(voltage[7]-2.5)/2));
	
	//Beräknar vinkeln mot väggen (Alpha, i designspec.), det antas att avståndet mellan S1 och S2 är 5cm.  
	//Definierad som positiv om robot riktad åt vänster och negativ om riktad åt höger.
	if (output[0] > output[1])
	{
		sign[0] = 1;
		alpha[0] = atan((double)(output[0]-output[1])/(11.5*5))*180/M_PI;
	}
	else
	{
		sign[0] = 0;
		alpha[0] = atan((double)(output[1]-output[0])/(11.5*5))*180/M_PI;
	}
	
	alpha[1] = atan((double)(output[4]-output[3])/(11.5*5))*180/M_PI;

	PORTD = output[3];
	PORTD = output[4];
	 
	
	
}


int main(void)
{
	//Sätter port D och B till utgångar.
	DDRD = 0xFF;
 	DDRB = 0xFF;
	 
	PORTD = 0xFF;
	
	//Ställer in muxen så att sensor0 används som input
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;	
	ADCSRA = 0 << ADIF | 1<<ADEN | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0 | 0<<ADATE;
	
	//Startar en läsning från sensor0
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

