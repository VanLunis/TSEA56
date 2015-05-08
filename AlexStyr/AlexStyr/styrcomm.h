#ifndef STYRCOMM_H
#define STYRCOMM_H

#include "buffer.h"
#include "bitman.h"
#include "styr_defs.h"
#include <avr/io.h>

struct data_buffer receive_buffer;
struct data_buffer send_buffer;
volatile int mode; // 0 receiving, 1 sending.
volatile int transmission_status;
volatile int counter;
volatile struct data_byte temp_data;


// Functions that are used in interrupts caused by the SPI-bus
void send_to_master(struct data_buffer* my_buffer);
void receive_from_master(struct data_buffer* my_buffer);

// In autonomous mode: get sensor values from receive buffer
void update_values_from_sensor();
void update_sensors_and_empty_receive_buffer();

// In remote control mode: use remote control values from receive buffer
void remote_control(char control_val);

void send_map(int map[17][17]);

#endif