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
	
	{3.019, 2.46, 2.18, 2.771, 1.61, 1.438, 1.27, 1.143, 1.11, 1.01, 0.91, 0.84, 0.82, 0.791, 0.75, 0.698, 0.655, 0.615, 0.585, 0.550, 0.530, 0.510, 0.506, 0.502, 0.49, 0.485, 0.46, 0.43, 0},
	
	{3.01, 2.4, 2.07, 1.77, 1.65, 1.43, 1.28, 1.14, 1.1, 0.99, 0.9, 0.83, 0.82, 0.79, 0.735, 0.69, 0.645, 0.6, 0.570, 0.545, 0.52, 0.505, 0.5, 0.49, 0.47, 0.45, 0.43, 0.415, 0},
	
	{2.95, 2.4, 2.05, 1.75, 1.6, 1.42, 1.25, 1.14, 1.1, 1.0, 0.89, 0.83, 0.815, 0.79, 0.72, 0.675, 0.640, 0.61, 0.58, 0.54, 0.52, 0.51, 0.505, 0.49, 0.48, 0.46, 0.45, 0.44, 0},
	
	{3.05, 2.5, 2.12, 1.75, 1.6, 1.44, 1.3, 1.15, 1.11, 1.01, 0.92, 0.83, 0.82, 0.80, 0.75, 0.69, 0.65, 0.59, 0.57, 0.55, 0.52, 0.505, 0.5, 0.49, 0.47, 0.45, 0.43, 0.42, 0},
	
	{2.95, 2.5, 2.1, 1.77, 1.64, 1.44, 1.3, 1.14, 1.11, 1.07, 0.95, 0.89, 0.84, 0.83, 0.81, 0.76, 0.70, 0.65, 0.65, 0.64, 0.63, 0.5, 0.49, 0.48, 0.48, 0.47, 0.44, 0.41, 0}
	
};

//Avstånd tillhörande IR-sensorernas spänningar, i enheten 0.2cm
int distances[33] = {15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180};
//Antalet värden som skall användas för median-bildning.
int median_amount = 9;
//Array för att lagra input från IR-sensorer, är en spänning.
double input[5][9];

double AVCC = 4.94;

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
	if (voltage[5] >= 2 && driven_distance==0){
		driven_distance = 2*3.1*22/(7*8);
	}
	else{
		driven_distance = 0;
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
	add_to_buffer(&SPI_send_buffer, 0x18, atan((temp_front_right - temp_rear_right)/15));
	add_to_buffer(&SPI_send_buffer, 0x19, atan((temp_front_left - temp_rear_left)/15));
	
}
