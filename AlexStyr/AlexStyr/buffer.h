/*
 * buffer.h
 *
 * Last edited: 15-04-2015
 * Author: Frida Sundberg, Markus Petersson
 *
 */
#ifndef BUFFER_H
#define BUFFER_H
#define BUFFER_SIZE 254

// Byta frÂn char till uint8_t?
struct data_byte
{
    char type;
    char val;
};

struct data_byte null_data_byte;

struct data_buffer
{
    //two variables to keep information about our array that we will traverse
    //in a circle-like way.
    int head; //head indicates the index of the next byte to be sent.
    int tail; //tail indicates the index of the next byte to be added to the buffer.
    int full_revolution; //a "boolean" to tell if the entire buffer is traversed.
    struct data_byte contents[BUFFER_SIZE]; //defines the content of the buffer to be an array of data_bytes.
    
};

//struct data_buffer my_buffer;

void buffer_init(struct data_buffer* my_buffer);

int buffer_empty(struct data_buffer* my_buffer);

// Add garbage collector?
void discard_from_buffer(struct data_buffer* my_buffer);

void add_to_buffer(struct data_buffer* my_buffer, char new_type, char new_val);

int amount_stored(struct data_buffer* my_buffer);

struct data_byte fetch_from_buffer(struct data_buffer* my_buffer);

#endif