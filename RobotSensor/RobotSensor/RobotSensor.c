/*
 * RobotSensor.c
 *
 *  Created: 16/4 - 2015
 *
 * Actual code for merged solution between the modules
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "buffer.h"
#include<stdio.h>
#include<stdlib.h>
#include <stdio.h>
#include <math.h>
#include<stdlib.h>
#include<avr/delay.h>
#include<util/delay.h>
#define F_CPU 14745000UL
#define M_PI 3.14159265358979323846

//Spänningar associerade med avstånden nedan för de olika IR-sensorerna. (En rad/sensor)
double lookuptable[5][29] = {
	
	{3.06, 2.83, 2.25, 1.87, 1.691, 1.503, 1.351, 1.239, 1.128, 1.051, 0.974, 0.917, 0.854, 0.803, 0.742, 0.701, 0.662, 0.628, 0.603, 0.575, 0.544, 0.525, 0.506, 0.486, 0.466, 0.445, 0.426, 0.412, 0.407, 0.388, 0.380, 0},
	
	{3.042, 2.67,  2.13, 1.643, 1.606, 1.404, 1.287, 1.213, 1.079, 1.023, 0.909, 0.86, 0.814, 0.77, 0.731, 0.693, 0.654, 0.635, 0.596, 0.576, 0.547, 0.520, 0.502, 0.485, 0.463, 0.449, 0.440, 0.420, 0.410, 0.402, 0.384, 0},
	
	{5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 2.551, 2.350, 2.169, 2.045, 2.012, 1.887, 1.756, 1.661, 1.569, 1.495, 1.492, 1.365, 1.308, 1.253, 1.215, 1.158, 1.120, 1.077, 1.039, 1.020, 0.982, 0.957, 0.925, 0.905, 0.868, 0.848,
	0.829, 0.810, 0.791, 0.773, 0.753, 0.740, 0.734, 0.715, 0.705, 0.695, 675, 0.675, 0},
	
	{3.05, 2.5, 2.12, 1.75, 1.6, 1.44, 1.3, 1.15, 1.11, 1.01, 0.92, 0.83, 0.82, 0.80, 0.75, 0.69, 0.65, 0.59, 0.57, 0.55, 0.52, 0.505, 0.5, 0.49, 0.47, 0.45, 0.43, 0.42, 0},
	
	{2.95, 2.5, 2.1, 1.77, 1.64, 1.44, 1.3, 1.14, 1.11, 1.07, 0.95, 0.89, 0.84, 0.83, 0.81, 0.76, 0.70, 0.65, 0.65, 0.64, 0.63, 0.5, 0.49, 0.48, 0.48, 0.47, 0.44, 0.41, 0}
	
};

//Avstånd tillhörande IR-sensorernas spänningar, i enheten 0.2cm
int distances[33] = {15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180};
//Antalet värden som skall användas för median-bildning.
int median_amount = 9;
//Array för att lagra input från IR-sensorer, är en spänning.
double input[5][9];

double AVCC = 5.03;

struct data_buffer SPI_receive_buffer;
struct data_buffer SPI_send_buffer;

volatile int mode = 0; // 0 receiving, 1 sending.
volatile int transmission_status = 0;
volatile struct data_byte temp_data;
volatile int counter = 0;
volatile char driven_distance = 0;
volatile char goal_detected = 0;

void init_sensor_module(void);

void send_to_master(struct data_buffer* my_buffer);
void receive_from_master(struct data_buffer* my_buffer);

double median(double value[]);
double lookup (double voltage, int sensorindex);
void read();
void queue_to_send();

//////////////////////////////////////////////////////////////////////
//----------------------------  MAIN -------------------------------//
//////////////////////////////////////////////////////////////////////

int main(void)
{
	init_sensor_module();
	sei();
	
    while(1)
    {
		 if (ADCSRA & (1<<ADIF))
		 {
			 read();
			 queue_to_send();
			 for (int i=0; i<200; i++)
			 {
				 _delay_ms(10);
			 }
		 }
	}
}

///////////////////////////////////////////////////////////////////////

void init_sensor_module(void)
{
	// SPI Init
	SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(0<<MSTR)|(0<<CPOL)|(0<<CPHA);
	DDRB = (1<<DDB6)|(1<<DDB3);
	PINB = (1<<PINB4);
	
	// IRQ1 and IRQ0 activated on rising edge
	EICRA = (1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(1<<ISC00);
	// Enable IRQ1 and IRQ0
	EIMSK = (1<<INT1)|(1<<INT0);
	
	//Initiate the buffers.
	buffer_init(&SPI_receive_buffer);
	buffer_init(&SPI_send_buffer);
	
	// Sensor Init
	//Ställer in muxen så att sensor0 används som input
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	ADCSRA = 0 << ADIF | 1<<ADEN | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0 | 0<<ADATE;
	
	//Startar en läsning från sensor0
	ADCSRA = (1<<ADEN) | (1<<ADSC) | 7 ;
};

/////////////////// SPI FUNCTIONS /////////////////////////////////
void send_to_master(struct data_buffer* my_buffer)
{
	//if Transmission not yet started: fetch type and put it in SPDR.
	if(transmission_status == 0)
	{
		SPDR = fetch_from_buffer(my_buffer).type;
		transmission_status = 1;
	}
	//if Type already sent: fetch val and put it in SPDR.
	else if(transmission_status == 1)
	{
		SPDR = fetch_from_buffer(my_buffer).val;
		transmission_status = 2;
	}
	//if the master has accepted both bytes that were sent: discard the data_byte from the buffer.
	else if(transmission_status == 2)
	{
		discard_from_buffer(my_buffer);
		transmission_status = 0;
	}
};


//Method to receive a data_byte. Called when SPI interrupt has occurred.
void receive_from_master(struct data_buffer* my_buffer)
{
	//get type.
	if(transmission_status == 0)
	{
		temp_data.type = SPDR;
		transmission_status = 1;
	}
	//get val.
	else if(transmission_status == 1)
	{
		temp_data.val = SPDR;
		add_to_buffer(my_buffer, temp_data.type, temp_data.val);//add to receive_buffer when
		transmission_status = 0;
	}
}

/////////////////// INTERRUPTS /////////////////////////////////////
ISR(SPI_STC_vect)
{
	PORTB = (1<<PORTB3);
	PORTB = (0<<PORTB3); // OBS changed order so low when not active
	//Depending on the current mode: do things.
	if(mode == 0)
	{
		receive_from_master(&SPI_receive_buffer);
	}
	else if(mode == 1)
	{
		send_to_master(&SPI_send_buffer);
	}
}

//Set slave to reading mode.
ISR(INT0_vect)
{
	transmission_status = 0;
	mode = 0;
}

//Set slave to sending mode.
ISR(INT1_vect)
{
	transmission_status = 1;
	mode = 1;
	SPDR = fetch_from_buffer(&SPI_send_buffer).type;
}

/////////////////// SENSOR FUNCTIONS ////////////////////////////////
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

void queue_to_send(){
	double voltage[8];
	int waswhite;
	double distance;
	
	for (int i=0; i<8; i++){
		voltage[i] = median(input[i]);
	}
	
	char temp_rear_right = (char)lookup(voltage[0], 0);
	char temp_front_right = (char)lookup(voltage[1], 1);
	char temp_front = (char)lookup(voltage[2], 2);
	char temp_front_left = (char)lookup(voltage[3], 3);
	char temp_rear_left = (char)lookup(voltage[4], 4);
	
	add_to_buffer(&SPI_send_buffer, 0x10, temp_rear_right);
	add_to_buffer(&SPI_send_buffer, 0x11, temp_front_right);
	add_to_buffer(&SPI_send_buffer, 0x12, temp_front);
	add_to_buffer(&SPI_send_buffer, 0x13, temp_front_left);
	add_to_buffer(&SPI_send_buffer, 0x14, temp_rear_left);
	
	//Hjultejpsensor, returnerar längd då tejp hittas (svart ger utspänning ~3.9V, ljusgrå ger ~0.2V )
	if (voltage[5] >= 2 && waswhite){
		distance += 2*3.1*3.14159/8;
		if (distance >= 40)
		{
			distance = 0;
		}
		driven_distance = distance;
		waswhite = 0;
	}
	
	else if(voltage[5]<1)
	{
		waswhite = 1;
	}
	add_to_buffer(&SPI_send_buffer, 0x15, driven_distance);
	
	//Golv-tejpsensor, returnerar 1 då tejp hittas (tejp ger utspänning ~4.3V, golv oklart)
	if (voltage[6] >= 3){
		goal_detected = 1;
	}
	else{
		goal_detected = 0;
	}
	add_to_buffer(&SPI_send_buffer, 0x16, goal_detected);
	
	//gyro, returnerar 4.5V då 300grad/sek och 0.5V då -300grad/sek
	add_to_buffer(&SPI_send_buffer, 0x17, (char)(300*(voltage[7]-2.5)/2));
	
	//Beräknar vinkeln mot väggen (Alpha, i designspec.), det antas att avståndet mellan S1 och S2 är 5cm.
	//Definierad som positiv om robot riktad åt vänster och negativ om riktad åt höger.
	//Om positiv är tillhörande sign[i] 0, 1 om negativ.  
	if (temp_rear_right > temp_front_right)
	{
		sign[0] = 1;
		add_to_buffer(&SPI_send_buffer, 0x18, atan((temp_rear_right - temp_front_right)/(11.5*5)));
	}
	else
	{
		sign[0] = 0;
		add_to_buffer(&SPI_send_buffer, 0x18, atan((temp_front_right - temp_rear_right)/(11.5*5)));
	}
	
	if (temp_rear_left < temp_front_left)
	{
		sign[1] = 1;
		add_to_buffer(&SPI_send_buffer, 0x19, atan((temp_rear_left - temp_front_left)/(11.5*5)))
	}
	else
	{
		sign[1] = 0;
		add_to_buffer(&SPI_send_buffer, 0x19, atan((temp_front_left - temp_rear_left)/(11.5*5)));
	}
	
	add_to_buffer(&SPI_send_buffer, 0x20, sign[0]));
	add_to_buffer(&SPI_send_buffer, 0x21, sign[1]));
	
	
	
	
}
