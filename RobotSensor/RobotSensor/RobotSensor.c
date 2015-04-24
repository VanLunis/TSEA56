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

//Sp�nningar associerade med avst�nden nedan f�r de olika IR-sensorerna. (En rad/sensor)
double lookuptable[5][50] = {
	
	{3.014, 2.684, 2.268, 1.959, 1.724, 1.540, 1.369, 1.257, 1.145, 1.087, 0.991, 0.934, 0.865, 0.819, 0.742, 0.755, 0.717, 0.679, 0.640, 0.621, 0.582, 0.562, 0.543, 0.510, 0.488, 0.469, 0.456, 0.444, 0.424, 0.405, 0.401, 0.386, 0},
	
	{3.020, 2.960,  2.411, 2.043, 1.803, 1.495, 1.392, 1.250, 1.139, 1.059, 0.965, 0.903, 0.849, 0.794, 0.731, 0.740, 0.704, 0.672, 0.634, 0.598, 0.577, 0.557, 0.536, 0.506, 0.490, 0.480, 0.462, 0.441, 0.425, 0.419, 0.402, 0.387, 0},
	
	{2.543, 2.351, 2.135, 1.976, 1.849, 1.740, 1.644, 1.552, 1.496, 1.423, 1.352, 1.292, 1.254, 1.197, 1.160, 1.122, 1.079, 1.041, 1.022, 0.984, 0.965, 0.927, 0.908, 0.888, 0.869, 0.847, 0.831, 0.812, 0.793, 0.775, 0.755, 0.736, 0.716, 0},
	//0		1		2		3		4		5	6		7		8		9	10		11		12	13		14		15		16	17		18		19		20	21		22		23		24	25		26
	{3.105, 2.735, 2.372, 2.081, 1.790, 1.589, 1.401, 1.271, 1.158, 1.082, 0.996, 0.919, 0.842, 0.804, 0.746, 0.708, 0.666, 0.627, 0.589, 0.569, 0.546, 0.510, 0.491, 0.471, 0.452, 0.432, 0.413, 0.394, 0.391, 0.374, 0.373, 0},
	
	{3.033, 2.721, 2.106, 1.794, 1.560, 1.404, 1.269, 1.155, 1.073, 0.977, 0.919, 0.847, 0.802, 0.748, 0.700, 0.660, 0.623, 0.602, 0.565, 0.544, 0.523, 0.497, 0.473, 0.466, 0.447, 0.428, 0.422, 0.406, 0.388, 0.384, 0.365, 0}
	
};


//Avst�nd tillh�rande IR-sensorernas sp�nningar, i enheten 0.2cm
int distances[36] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 180, 185};
//Antalet v�rden som skall anv�ndas f�r median-bildning.
int median_amount = 25;
//Array f�r att lagra input fr�n IR-sensorer, �r en sp�nning.
double input[5][25];

double AVCC = 5.00;

struct data_buffer SPI_receive_buffer;
struct data_buffer SPI_send_buffer;

volatile int mode = 0; // 0 receiving, 1 sending.
volatile int transmission_status = 0;
volatile struct data_byte temp_data;
volatile int counter = 0;
volatile char driven_distance = 0;
volatile char goal_detected = 0;
volatile char sign[2];

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
			 for (int i=0; i<20; i++)
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
	//St�ller in muxen s� att sensor0 anv�nds som input
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	ADCSRA = 0 << ADIF | 1<<ADEN | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0 | 0<<ADATE;
	
	//Startar en l�sning fr�n sensor0
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
//funktion som sorterar en m�ngd v�rden i en array och returnerar medianen.
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
	return value[12];
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
	//L�ser fr�n ADC0/Sensor1 - IR
	ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
	_delay_ms(5);
	for (int i=0; i<median_amount; i++){
		
		//Nollst�ller ADIF och startar konvertering
		ADCSRA |= (1<<ADIF) | (1<<ADSC);
		
		//V�ntar, s� att vi inte g�r in i ny loop och f�rs�ker starta
		//ni konvertering innan den ovan �r f�rdig
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
	///*
	add_to_buffer(&SPI_send_buffer, 0xFF, temp_rear_right);
	
	add_to_buffer(&SPI_send_buffer, 0xFE, temp_front_right);
	add_to_buffer(&SPI_send_buffer, 0xFD, temp_front);
	add_to_buffer(&SPI_send_buffer, 0xFC, temp_front_left);
	add_to_buffer(&SPI_send_buffer, 0xFB, temp_rear_left);
	//*/
	
	/*
	add_to_buffer(&SPI_send_buffer, 0xFF, voltage[0]);
	add_to_buffer(&SPI_send_buffer, 0xFE,voltage[1]);
	add_to_buffer(&SPI_send_buffer, 0xFD, voltage[2]);
	add_to_buffer(&SPI_send_buffer, 0xFC, voltage[3]);
	add_to_buffer(&SPI_send_buffer, 0xFB, voltage[4]);
	*/
	
	//Hjultejpsensor, returnerar l�ngd d� tejp hittas (svart ger utsp�nning ~3.9V, ljusgr� ger ~0.2V )
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
	add_to_buffer(&SPI_send_buffer, 0xFA, driven_distance);
	
	//Golv-tejpsensor, returnerar 1 d� tejp hittas (tejp ger utsp�nning ~4.3V, golv oklart)
	if (voltage[6] >= 3){
		goal_detected = 1;
	}
	else{
		goal_detected = 0;
	}
	add_to_buffer(&SPI_send_buffer, 0xF9, goal_detected);
	
	//gyro, returnerar 4.5V d� 300grad/sek och 0.5V d� -300grad/sek
	if (voltage[7]>=2.5){
		add_to_buffer(&SPI_send_buffer, 0xF8, (char)(300*(voltage[7]-2.5)/2));
		add_to_buffer(&SPI_send_buffer, 0xF7, (char)0);
	}
	else{
		add_to_buffer(&SPI_send_buffer, 0xF8, (char)0);
		add_to_buffer(&SPI_send_buffer, 0xF7, (char)(300*(2.5-voltage[7])/2));
	}
	
	//Ber�knar vinkeln mot v�ggen (Alpha, i designspec.), det antas att avst�ndet mellan S1 och S2 �r 5cm.
	//Definierad som positiv om robot riktad �t v�nster och negativ om riktad �t h�ger.
	//Om positiv �r tillh�rande sign[i] 0, 1 om negativ.  
	if (temp_rear_right > temp_front_right)
	{
		sign[0] = 1;
		add_to_buffer(&SPI_send_buffer, 0xF6, atan((temp_rear_right - temp_front_right)/(11.5*5)));
	}
	else
	{
		sign[0] = 0;
		add_to_buffer(&SPI_send_buffer, 0xF6, atan((temp_front_right - temp_rear_right)/(11.5*5)));
	}
	
	if (temp_rear_left < temp_front_left)
	{
		sign[1] = 1;
		add_to_buffer(&SPI_send_buffer, 0xF5, atan((temp_rear_left - temp_front_left)/(11.5*5)));
	}
	else
	{
		sign[1] = 0;
		add_to_buffer(&SPI_send_buffer, 0xF5, atan((temp_front_left - temp_rear_left)/(11.5*5)));
	}
	
	add_to_buffer(&SPI_send_buffer, 0xF4, sign[0]);
	add_to_buffer(&SPI_send_buffer, 0xF3, sign[1]);
	
	
	
	
}
