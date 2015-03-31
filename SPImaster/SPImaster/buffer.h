#ifndef HEADER_FILE
#define HEADER_FILE
#define BUFFER_SIZE 10

struct data_byte
{
	char type;
	char val;
};
struct data_buffer
{
	//two variables to keep information about our array that we will traverse
	//in a circle-like way.
	int head; //head indicates the index of the next byte to be sent.
	int tail; //tail indicates the index of the next byte to be added to the buffer.
	int full_revolution; //a "boolean" to tell if the entire buffer is traversed.
	struct data_byte contents[BUFFER_SIZE]; //defines the content of the buffer to be an array of data_bytes.
	
};
struct data_buffer my_buffer;

void buffer_init()
{
	//initiate the buffer so that it is empty.
	my_buffer.head=0;
	my_buffer.tail=0;
	my_buffer.full_revolution=0;
}
int buffer_empty()
{
	if (my_buffer.head==my_buffer.tail && (my_buffer.full_revolution != 1))
	{
		return 1;
	}
	return 0;
}

void add_to_buffer(char new_type, char new_val)
{
	//We allow data to be put on our data-buffer if we have not yet traversed the entire buffer. (The buffer can be viewed as a circle).
	if(my_buffer.full_revolution==0)
	{
		my_buffer.contents[my_buffer.tail].type=new_type;
		my_buffer.contents[my_buffer.tail].val=new_val;
		//If the tail catches up to the head we have a full revolution.
		if(((my_buffer.tail+1) % BUFFER_SIZE)==((my_buffer.head)%BUFFER_SIZE))
		{
			my_buffer.full_revolution=1;
		}
		my_buffer.tail=(my_buffer.tail+1) % BUFFER_SIZE;
		
		PORTA=(char)my_buffer.full_revolution;
	}
}
void send_from_buffer()
{
	//we allow data to be sent if the buffer is non-empty.
	if(buffer_empty()==0)
	{
		
		send_to_slave2(my_buffer.contents[my_buffer.head].type);
		send_to_slave2(my_buffer.contents[my_buffer.head].val);
		my_buffer.head=(my_buffer.head+1) % BUFFER_SIZE;
		my_buffer.full_revolution=0;
	}
}


#endif