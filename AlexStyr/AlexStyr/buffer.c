#include "buffer.h"

struct data_byte null_data_byte = {0x00, 0x00};

void buffer_init(struct data_buffer* my_buffer)
{
    //initiate the buffer so that it is empty.
    my_buffer->head = 0;
    my_buffer->tail = 0;
    my_buffer->full_revolution = 0;
}

int buffer_empty(struct data_buffer* my_buffer)
{
    if (my_buffer->head == my_buffer->tail && (my_buffer->full_revolution != 1))
    {
        return 1;
    }
    return 0;
}

void discard_from_buffer(struct data_buffer* my_buffer)
{
    if(buffer_empty(my_buffer)==0)
    {
        my_buffer->head = (my_buffer->head + 1) % BUFFER_SIZE;
        my_buffer->full_revolution = 0;
    }
};

void add_to_buffer(struct data_buffer* my_buffer, char new_type, char new_val)
{
    //We allow data to be put on our data-buffer if we have not yet traversed the entire buffer. (The buffer can be viewed as a circle).
    if(my_buffer->full_revolution == 0)
    {
        discard_from_buffer(my_buffer);
    }
    my_buffer->contents[my_buffer->tail].type = new_type;
    my_buffer->contents[my_buffer->tail].val = new_val;
    //If the tail catches up to the head we have a full revolution.
    if(((my_buffer->tail + 1) % BUFFER_SIZE) == ((my_buffer->head) % BUFFER_SIZE))
    {
        my_buffer->full_revolution = 1;
    }
    my_buffer->tail = (my_buffer->tail + 1) % BUFFER_SIZE;
    
    //PORTA=(char)my_buffer->full_revolution;
    
};

int amount_stored(struct data_buffer* my_buffer)
{
    if(my_buffer->head <= my_buffer->tail)
    {
        if(my_buffer->full_revolution==0)
        {
            return ((my_buffer->tail) - (my_buffer->head));
        }
        return BUFFER_SIZE;
    }
    
    else
    {
        return(BUFFER_SIZE - my_buffer->head + my_buffer->tail);
    }
}

struct data_byte fetch_from_buffer(struct data_buffer* my_buffer)
{
    if(buffer_empty(my_buffer) == 0)
    {
        return my_buffer->contents[my_buffer->head];
    }
    return null_data_byte;
}
