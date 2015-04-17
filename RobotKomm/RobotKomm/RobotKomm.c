/*
 * RobotKomm.c
 *
 * Created: 16/4 - 2015
 *
 * Actual code for merged solution between the modules
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "buffer.h"


#define F_CPU = 16000000UL// OBS need to change
#define delay_time 0.5

volatile int sensor_ready = 1;
volatile int control_ready = 1;
volatile int failed_attempts = 0;
volatile int transmission_status = 0; // 0 to send .type, 1 to send .val, 2 when both sent
volatile int counter = 0;
volatile struct data_byte temp_data;

struct data_buffer sensor_buffer;
struct data_buffer control_buffer;
struct data_buffer pc_buffer;

void init_master(void);

// SPI bus functions
void send(struct data_buffer* my_buffer, int slave);
void send_to_sensor();
void send_to_control();
void receive(int slave);
void receive_from_sensor();
void receive_from_control();

// Bluetooth functions
unsigned char USART_Receive(void);
void USART_Transmit(unsigned char data);
void USART_to_SPI(void);

//////////////////////////////////////////////////////////////////////
//----------------------------  MAIN -------------------------------//
//////////////////////////////////////////////////////////////////////

int main(void)
{
	init_master();
	sei();
		
	while(1) //(;;)
	{
///////////////// TEST ///////////////////////////////////////////////		
		//USART_to_SPI();
		if (!buffer_empty(&control_buffer))
		{
			_delay_ms(5);
			send_to_control();
		}
		PORTA = amount_stored(&control_buffer);
//////////////////////////////////////////////////////////////////////		
	}
}

///////////////////////////////////////////////////////////////////////

////////////////// INTERRUPTS ///////////////////////////////////////
ISR(INT0_vect)
{
	control_ready = 1;
	transmission_status++;
}

ISR(INT1_vect)
{
	sensor_ready = 1;
	transmission_status++;
}

ISR(SPI_STC_vect)
{	
	PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
}

ISR(USART0_RX_vect)
{
	unsigned char fire_fly_char;
	fire_fly_char = USART_Receive();
	if((fire_fly_char & 0xf0) == 0x40)
	{
		add_to_buffer(&control_buffer, 0x01, fire_fly_char);
	}
}


//////////////// INIT FUNCTION //////////////////////////////////////
void init_master(void)
{
	// SCK=f/64
	SPCR = (1<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR1)|(0<<SPR0);
	SPSR = (0<<SPI2X);
	DDRB = (1<<DDB7)|(1<<DDB5)|(1<<DDB4)|(1<<DDB3)|(1<<DDB0);
	PORTB =(1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
	
	// PC0 output, PD7 output (ext int to slave 2 to change receive/send-variable in slave 2)
	DDRC = (1<<DDC0);
	PORTC = (0<<PORTC0);
	
	DDRD = (1<<DDD7)|(1<<DDD6)|(1<<DDD5)|(1<<DDD1);
	PORTD = (0<<PORTD7)|(0<<PORTD6);
	
	// IRQ1 and IRQ0 activated on rising edge
	EICRA = (1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(1<<ISC00);
	// Enable IRQ1 and IRQ0
	EIMSK = (1<<INT1)|(1<<INT0);
	
	// Init data buffers in master
	buffer_init(&sensor_buffer);
	buffer_init(&control_buffer);
	buffer_init(&pc_buffer);
	
	// Init serial USART (bluetooth)
	/* Set baud rate */
	unsigned int baud = 7;
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0L = (unsigned char)baud;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
	
///////////////////////// TEST TEST TEST /////////////////////////
	DDRA = 0xFF;
//////////////////////////////////////////////////////////////////		

};

///////////////// COMMUNICATION SPI FUNCTIONS //////////////////////
void send(struct data_buffer* my_buffer, int slave)
{
	int *current_slave_ready;
	if (slave == 1)
	{
		current_slave_ready = &sensor_ready;
		PORTB = (1<<PORTB4)|(1<<PORTB3)|(1<<PORTB0);// Order slave 1 to adapt receive mode
		PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0);
	}
	else if(slave == 2)
	{
		current_slave_ready = &control_ready;
		PORTC = (1<<PORTC0);// Order slave 2 to adapt receive mode
		PORTC = (0<<PORTC0);
	}
	struct data_byte send_byte = fetch_from_buffer(my_buffer); // Fetch one byte from buffer without discarding
	transmission_status = 0; // Integer to indicate how far the send process has proceeded, updated when acknowledgment received from slave (in external interrupt)
	volatile int local_failed_attempts = 0;
	while(1)
	{
		if (slave == 1)
		{
			PORTB =(1<<PORTB4)|(0<<PORTB3)|(0<<PORTB0); // Pulling SS1 low
		}
		else if(slave==2)
		{
			PORTB =(0<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 low
		}
		if(*current_slave_ready==1)
		{
			if(transmission_status == 0) // Ready to send .type part
			{
				*current_slave_ready = 0;
				SPDR = send_byte.type;
				_delay_ms(delay_time);
			}
			else if(transmission_status == 1) // Ready to send .val part
			{
				*current_slave_ready = 0;
				SPDR = send_byte.val;
				_delay_ms(delay_time);
			}
			else if(transmission_status == 2) // Full send_byte transmitted correctly
			{
				discard_from_buffer(my_buffer); // Discard byte from buffer when full transmission succeeded
				PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
				transmission_status = 0;
				break;
			}
		}
		else
		{
			// OBS: add delay
			failed_attempts++;
			local_failed_attempts++;
			
			if(local_failed_attempts > 1000) // tries allowed before leaving function
			{
				PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
				transmission_status = 0;
				break;
			}
		}
	}
	
};

void send_to_sensor()
{
	send(&sensor_buffer, 1);
};

void send_to_control()
{
	send(&control_buffer, 2);
};

void receive(int slave)
{
	int local_failed_attempts = 0;
	int *current_slave_ready;
	if (slave == 1)
	{
		current_slave_ready = &sensor_ready;
		PORTD =(0<<PORTD7)|(0<<PORTD6); // Order slave 1 to adapt send mode
		PORTD =(0<<PORTD7)|(1<<PORTD6);
	}
	else if(slave == 2)
	{
		current_slave_ready = &control_ready;
		PORTD = (0<<PORTD7)|(0<<PORTD6); // Order slave 2 to adapt send mode
		PORTD = (1<<PORTD7)|(0<<PORTD6);
	}
	transmission_status=0;
	while(1)
	{
		//Select the right slave.
		if (slave == 1)
		{
			PORTB =(1<<PORTB4)|(0<<PORTB3)|(0<<PORTB0); // Pulling SS1 low
		}
		else if(slave==2)
		{
			PORTB =(0<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 low
		}
		//check if the current slave is ready to send. The slave_ready status is set through acknowledgments.
		if(*current_slave_ready == 1)
		{
			//if statement to check where in the transmission process we are currently at.
			if(transmission_status == 0)
			{
				*current_slave_ready = 0;
				SPDR = 0x00;//random data to send to slave, since the slave is sending it does not care what it gets.
				_delay_ms(delay_time);
			}
			else if(transmission_status == 1)
			{
				temp_data.type = SPDR;//read the data that the slave has sent. Note that SPDR should be read here and not where transmission_status==0
				//to make sure that we have gotten an acknowledgment from the slave
				*current_slave_ready = 0;
				SPDR = 0x00;//random data to send to slave, since the slave is sending it does not care what it gets.
				_delay_ms(delay_time);
				
			}
			else if(transmission_status==2)
			{
				temp_data.val = SPDR; //read the data that the slave has sent.
				transmission_status = 0;
				PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
				if(temp_data.type == 0 && temp_data.val == 0) //if we have received a null_data_byte: ignore.
				{
					
				}
				else
				{
					if (slave == 1)
					{
						add_to_buffer(&control_buffer, temp_data.type, temp_data.val);
					}
					_delay_ms(delay_time);
					add_to_buffer(&pc_buffer, temp_data.type, temp_data.val); //otherwise add to buffer!
				}
				_delay_ms(delay_time);
				break;
			}
		}
		else //Safety net so that we don't get stuck in a never ending loop if something with the transmission goes to... hell.
		{
			local_failed_attempts++;
			// OBS: add delay
			if(local_failed_attempts > 1000) // tries allowed before leaving function
			{
				PORTB = (1<<PORTB4)|(1<<PORTB3)|(0<<PORTB0); // Pulling SS2 and SS1 high
				transmission_status = 0;
				break;
			}
		}
	}
};

void receive_from_sensor()
{
	receive(1);
};

void receive_from_control()
{
	receive(2);
};

/////////////// BLUETOOTH FUNCTIONS ////////////////////////////////
unsigned char USART_Receive(void)
{
	if ( 1 << RXC0)
	{ // check if there is data to be received
			/* Wait for data to be received */
		while ( !(UCSR0A & (1<<RXC0)) )	;
	
		/* Get and return received data */
		return UDR0;	
	}
	return 0x00;
}

void USART_Transmit( unsigned char data )
{
		/* Wait for empty USART to transmit */
		while ( !( UCSR0A & (1<<UDRE0)) )	;
		/* Put data into USART, sends the data */
		UDR0 = data;
}

