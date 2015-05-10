#include "styrcomm.h"

volatile int mode = 0; // 0 receiving, 1 sending.
volatile int transmission_status = 0;
volatile int counter = 0;

// COMMUNICATION FUNCTIONS: -----------------------------------
// Functions that are used in interrupts caused by the SPI-bus
void send_to_master(struct data_buffer* my_buffer)
{
    /*if (fetch_from_buffer(my_buffer).type == null_data_byte.type && fetch_from_buffer(my_buffer).val==null_data_byte.val)
     {
     transmission_status = 0;
     return ;
     }*/
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

// In autonomous mode: get sensor values from receive buffer
void update_values_from_sensor(){
    
    if(!buffer_empty(&receive_buffer))
    {
        unsigned char temp_char = fetch_from_buffer(&receive_buffer).type;
        
        switch (temp_char)
        {
            case 0xFF: // = distance to wall: right back
                distance_right_back = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
                
            case 0xFE: // = distance to wall: right front
                distance_right_front = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
                
            case 0xFD: // = distance to wall: front
                distance_front = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
                
            case 0xFC:  // = distance to wall: left front
                distance_left_front = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
                
            case 0xFB: // = distance to wall: left back
                distance_left_back = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
                
            case 0xF7: // = distance to wall: back
                
                distance_back = round(fetch_from_buffer(&receive_buffer).val/5);
                break;
                
            case 0xFA: // distance driven:
                wheel_click = fetch_from_buffer(&receive_buffer).val;
                break;
            /*
            case 0xFA: // distance driven:
                 distance_driven = round(fetch_from_buffer(&receive_buffer).val/5);
                 break;*/
                
            case 0xF9:  // tejp sensor floor:
                goal_found = fetch_from_buffer(&receive_buffer).val;
                break;
            /*
            case 0xF9:  // tejp sensor floor:
                break;*/
                
        } // end of switch
        
        discard_from_buffer(&receive_buffer);
    } // end of if
}
void update_sensors_and_empty_receive_buffer()
{
    while(!buffer_empty(&receive_buffer))
    {
        update_values_from_sensor();
    }
}


// In remote control mode: use remote control values from receive buffer
void remote_control(char control_val){
    
    switch (control_val)
    {
        case 0x41: // = A in ascii => forward
            forward();
            break;
        case 0x42: // B in ascii => forward left
            slight_left();
            break;
        case 0x43: // C in ascii => forward right
            slight_right();
            break;
        case 0x44: // D in ascii => rotate left
            rotate_left(60);
            break;
        case 0x45: // E in ascii => rotate right
            rotate_right(60);
            break;
        case 0x46: // F in ascii => backwards
            backwards();
            break;
        case 0x47: // G in ascii => stop
            stop();
            break;
    }
    
}

void send_map(int map[17][17])
{
    unsigned char char_to_send = 0x80; // first bit is set to not be sending nullbyte to PC!
    
    for (int row = 0; row<15; row++) // Loop for each row
    {
        
        // loop for the first 5 columns in each row:
        for (int column = 0; column<5; column++)
        {
            if (map[row+1][column+1] == 1) // +1 since the map is 17x17
            {
                bit_set(char_to_send, BIT(column));
            }
            else
            {
                bit_clear(char_to_send, BIT(column));
            }
        }
        add_to_buffer(&send_buffer,0xEE - (3*row) ,char_to_send); // EDIT TYPE NUMBER!
        
        char_to_send = 0x80;
        // loop for the following 6-10 columns
        for (int column = 5; column<10; column++)
        {
            
            if (map[row+1][column+1] == 1) // +1 since the map is 17x17
            {
                bit_set(char_to_send, BIT(column-5));
            }
            else
            {
                bit_clear(char_to_send, BIT(column-5));
            }
        }
        add_to_buffer(&send_buffer,0xEE - (3*row + 1),char_to_send); // EDIT TYPE NUMBER!
        
        char_to_send = 0x80;
        // loop for the following 11-15 columns
        for (int column = 10; column<15; column++)
        {
            if (map[row+1][column+1] == 1) // +1 since the map is 17x17
            {
                bit_set(char_to_send, BIT(column-10));
            }
            else
            {
                bit_clear(char_to_send, BIT(column-10));
            }
        }
        add_to_buffer(&send_buffer,0xEE - (3*row + 2),char_to_send); // EDIT TYPE NUMBER!
        
        
    }// end of row loop
}