/*
 * RobotSensor.c
 *
 *  Created: 16/4 - 2015
 *
 * Actual code for merged solution between the modules
 *
 */
#define F_CPU 14745000UL
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

#define M_PI 3.14159265358979323846

//Spänningar associerade med avstånden nedan för de olika IR-sensorerna. (En rad/sensor)
double lookuptable[6][50] = {
    
    //{3.014, 2.684, 2.268, 1.959, 1.724, 1.540, 1.369, 1.257, 1.145, 1.087, 0.991, 0.934, 0.865, 0.819, 0.742, 0.755, 0.717, 0.679, 0.640, 0.621, 0.582, 0.562, 0.543, 0.510, 0.488, 0.469, 0.456, 0.444, 0.424, 0.405, 0.401, 0.386, -5},
    {1.8875, 1.7875, 1.575, 1.425, 1.2625, 1.15, 1.0875, 1.0125, 0.95, 0.8875, 0.8375, 0.75, 0.7125, 0.675, 0.6375, 0.625,  0.6, 0.5625, 0.5375, 0.5375, 0.525, 0.5, 0.5,   0.4875,  0.4875, 0.4625, 0.4625,   -5},
	
    //{3.020, 2.960,  2.411, 2.043, 1.803, 1.495, 1.392, 1.250, 1.139, 1.059, 0.965, 0.903, 0.849, 0.794, 0.731, 0.740, 0.704, 0.672, 0.634, 0.598, 0.577, 0.557, 0.536, 0.506, 0.490, 0.480, 0.462, 0.441, 0.425, 0.419, 0.402, 0.387, -5},
    {2.4125, 2.0625, 1.775, 1.575, 1.425, 1.2875, 1.15, 1.0625, 0.9875, 0.9375, 0.85,  0.8,   0.75,  0.7125, 0.675, 0.6375, 0.6, 0.575,  0.5375, 0.525,  0.5, 0.4875, 0.4625, 0.4375, 0.425,  0.4,   0.4,   -5},
	
    {2.625, 2.475, 2.225, 2.0875, 1.9625, 1.875, 1.7375, 1.6125, 1.5125, 1.4625, 1.4, 1.3375, 1.2875, 1.225, 1.1625, 1.125, 1.0875, 1.05, 1.0125, 0.9875, 0.975, 0.9375, 0.9125, 0.8875, 0.875, 0.85, 0.8125, 0.8, 0.775, 0.75,  -5},
	//{2.543, 2.351, 2.135, 1.976, 1.849, 1.740, 1.644, 1.552, 1.496, 1.423, 1.352, 1.292, 1.254, 1.197, 1.160, 1.122, 1.079, 1.041, 1.022, 0.984, 0.965, 0.927, 0.908, 0.888, 0.869, 0.847, 0.831, 0.812, 0.793, 0.775, 0.755, 0.736, 0.716, -5},
    //0     1       2       3       4       5   6       7       8       9   10      11      12  13      14      15      16  17      18      19      20  21      22      23      24  25      26
    //{3.105, 2.735, 2.372, 2.081, 1.790, 1.589, 1.401, 1.271, 1.158, 1.082, 0.996, 0.919, 0.842, 0.804, 0.746, 0.708, 0.666, 0.627, 0.589, 0.569, 0.546, 0.510, 0.491, 0.471, 0.452, 0.432, 0.413, 0.394, 0.391, 0.374, 0.373, 0},
    
    //{3.033, 2.721, 2.106, 1.794, 1.560, 1.404, 1.269, 1.155, 1.073, 0.977, 0.919, 0.847, 0.802, 0.748, 0.700, 0.660, 0.623, 0.602, 0.565, 0.544, 0.523, 0.497, 0.473, 0.466, 0.447, 0.428, 0.422, 0.406, 0.388, 0.384, 0.365, 0}
    
    //{3.020, 2.960,  2.411, 2.043, 1.803, 1.495, 1.392, 1.250, 1.139, 1.059, 0.965, 0.903, 0.849, 0.794, 0.731, 0.740, 0.704, 0.672, 0.634, 0.598, 0.577, 0.557, 0.536, 0.506, 0.490, 0.480, 0.462, 0.441, 0.425, 0.419, 0.402, 0.387, -5},
    
	{4,      3.0,   2.55,   2.1625, 1.85,  1.6375, 1.4375, 1.3375, 1.2,    1.1125, 1.025, 0.95, 0.875,  0.8125, 0.775,  0.7125, 0.675, 0.6375, 0.625, 0.575, 0.5625,  0.5375, 0.525, 0.4875, 0.4625, 0.4625, 0.4375, 0.425,  0.4, 0.4, -5},
    //{3.020, 2.960,  2.411, 2.043, 1.803, 1.495, 1.392, 1.250, 1.139, 1.059, 0.965, 0.903, 0.849, 0.794, 0.731, 0.740, 0.704, 0.672, 0.634, 0.598, 0.577, 0.557, 0.536, 0.506, 0.490, 0.480, 0.462, 0.441, 0.425, 0.419, 0.402, 0.387, -5},
    
	{2.5375, 2.125,  1.825,  1.6125, 1.425, 1.2875, 1.15,   1.0625, 0.9875, 0.9125, 0.85,  0.8,  0.7375, 0.675,  0.6375, 0.6,    0.575, 0.5375, 0.525, 0.4875, 0.4625, 0.4375, 0.425, 0.4,    0.4,    0.3875, 0.3625, 0.3625, 0.35, 0.35,  -5},
    //{3.020, 2.960,  2.411, 2.043, 1.803, 1.495, 1.392, 1.250, 1.139, 1.059, 0.965, 0.903, 0.849, 0.794, 0.731, 0.740, 0.704, 0.672, 0.634, 0.598, 0.577, 0.557, 0.536, 0.506, 0.490, 0.480, 0.462, 0.441, 0.425, 0.419, 0.402, 0.387, -5},
	{4, 3.1625, 2.9, 2.75, 2.0825, 1.8125, 1.6125, 1.4375, 1.325, 1.25, 1.125, 1.05, 0.975, 0.9125, 0.875, 0.8125, 0.775, 0.7375, 0.7, 0.6625, 0.6375, 0.6, 0.575, 0.5625, 0.5375, 0.525, 0.5, 0.4875, 0.4625, 0.4375, 0.4735, -5},
	
};


//Avstånd tillhörande IR-sensorernas spänningar, i enheten 0.2cm
int distances[36] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180};

//Antalet värden som skall användas för median-bildning.
int median_amount = 5;
//Array för att lagra input från IR-sensorer, är en spänning.
double input[7][25];

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
            //for (int i=0; i<20; i++)
            //{
            //_delay_us(34);
            //}
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
    return value[2];
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
    //  p*D_större+(1-p)*D_mindre, där D är de avstånd är de avstånd som hör ihop med spänningarna.
    p = (lookuptable[sensorindex][i-1] - voltage)/(lookuptable[sensorindex][i-1]-lookuptable[sensorindex][i]);
    return ((p*distances[i-1])+(1-p)*distances[i]);
}

void read(){
    //Läser från ADC0/Sensor1 - IR
    ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
    _delay_ms(0.34);
    for (int i=0; i<median_amount; i++){
        
        //Nollställer ADIF och startar konvertering
        ADCSRA |= (1<<ADIF) | (1<<ADSC);
        
        //Väntar, så att vi inte går in i ny loop och försöker starta
        //ni konvertering innan den ovan är färdig
        _delay_us(34);
        input[0][i] = ((double)ADCH*AVCC/256);
    }
    
    // ADC1/Sensor2 - IR
    ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;
    _delay_ms(0.34);
    for (int i=0; i<median_amount; i++){
        ADCSRA |= (1<<ADIF) | (1<<ADSC);
        _delay_us(34);
        input[1][i] = ((double)ADCH*AVCC/256);
    }
    
    // ADC2/Sensor3 - IR
    ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 1<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
    _delay_ms(0.34);
    for (int i=0; i<median_amount; i++){
        ADCSRA |= (1<<ADIF) | (1<<ADSC);
        _delay_us(34);
        input[2][i] = ((double)ADCH*AVCC/256);
    }
    
    // ADC3/Sensor4 - IR
    ADMUX = 0<<MUX4 | 0<<MUX3 | 0<<MUX2 | 1<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;
    _delay_ms(0.34);
    for (int i=0; i<median_amount; i++){
        ADCSRA |= (1<<ADIF) | (1<<ADSC);
        _delay_us(34);
        input[3][i] = ((double)ADCH*AVCC/256);
    }
    
    // ADC4/Sensor5 - IR
    ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 0<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
    _delay_ms(0.34);
    for (int i=0; i<median_amount; i++){
        ADCSRA |= (1<<ADIF) | (1<<ADSC);
        _delay_us(34);
        input[4][i] = ((double)ADCH*AVCC/256);
    }
    
    // ADC5/Sensor6 - reflexsensor
    ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 0<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;
    _delay_ms(0.34);
    for (int i=0; i<median_amount; i++){
        ADCSRA |= (1<<ADIF) | (1<<ADSC);
        _delay_us(34);
        input[5][i] = ((double)ADCH*AVCC/256);
    }
    
    // ADC6/Sensor7 - reflexsensor
    ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 1<<MUX1 | 0<<MUX0 | 1<<ADLAR | 1<<REFS0;
    _delay_ms(0.34);
    for (int i=0; i<median_amount; i++){
        ADCSRA |= (1<<ADIF) | (1<<ADSC);
        _delay_us(34);
        input[6][i] = ((double)ADCH*AVCC/256);
    }
    
    // ADC7/Sensor8 - back sensor
    ADMUX = 0<<MUX4 | 0<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0 | 1<<ADLAR | 1<<REFS0;
    _delay_ms(0.34);
    for (int i=0; i<median_amount; i++){
        ADCSRA |= (1<<ADIF) | (1<<ADSC);
        _delay_us(34);
        input[7][i] = ((double)ADCH*AVCC/256);
    }   
}

void queue_to_send(){
    double voltage[8];
    int black = 0;
    double distance;
    
    for (int i=0; i<8; i++){
        voltage[i] = median(input[i]);
    }
	
	char temp_rear_right = (char) lookup(voltage[0], 0);
	char temp_front_right = (char)lookup(voltage[1], 1);
	char temp_front = (char)lookup(voltage[2], 2);
	char temp_front_left = (char)lookup(voltage[3], 3);
	char temp_rear_left = (char)lookup(voltage[4], 4);
	char temp_back = (char)lookup(voltage[7], 5);
    
    add_to_buffer(&SPI_send_buffer, 0xFF, temp_rear_right);
    add_to_buffer(&SPI_send_buffer, 0xFE, temp_front_right);
    add_to_buffer(&SPI_send_buffer, 0xFD, temp_front);
    add_to_buffer(&SPI_send_buffer, 0xFC, temp_front_left);
    add_to_buffer(&SPI_send_buffer, 0xFB, temp_rear_left);
    add_to_buffer(&SPI_send_buffer, 0xF7, temp_back);
	
    
    //Hjultejpsensor, returnerar längd då tejp hittas (svart ger utspänning ~3.9V, ljusgrå ger ~0.2V )
    if ((voltage[5] >= 3.3) && (black == 0)){
        black = 1;
        add_to_buffer(&SPI_send_buffer, 0xFA, black);
    }
    
    else if(voltage[5]<3)
    {
        black = 0;
        add_to_buffer(&SPI_send_buffer, 0xFA, black);
    }
    else
    {
        add_to_buffer(&SPI_send_buffer, 0xFA, (uint8_t)0);
    }
    
    //Golv-tejpsensor, returnerar 1 då tejp hittas (tejp ger utspänning ~4.3V, golv oklart)
    if (voltage[6] >= 4.2){
        goal_detected = 1;
    }
    else{
        goal_detected = 0;
    }
    add_to_buffer(&SPI_send_buffer, 0xF9, goal_detected);
}