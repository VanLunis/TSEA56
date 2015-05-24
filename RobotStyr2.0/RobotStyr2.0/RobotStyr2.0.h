// Define processor clock
#define F_CPU 14700000UL

// Include files
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "buffer.h"
#include <stdlib.h>
#include "map.h"
#include "shortest_path.h"

// CONSTANTS //////////////////////////////////////////////////////////
// Define speeds
#define FULL_SPEED 60
#define SLOW_SPEED 40
#define VERY_SLOW_SPEED 25

// Define distance constants
#define WALLS_MAX_DISTANCE 22
#define FRONT_MAX_DISTANCE 10
#define BACK_MAX_DISTANCE 30
#define ROBOT_LENGTH 10
#define WHEEL_CLICK_DISTANCE 1.25 //3.14*2*3.1/16 = distance for each click for the wheel

#define ABS_VALUE_RIGHT 3.5
#define ABS_VALUE_LEFT 3.5

#define NEGATIVE_LIMIT -5
#define SLIGHT_NEGATIVE_LIMIT -1
#define SLIGHT_POSITIVE_LIMIT 1
#define POSITIVE_LIMIT 5

// PD-control constants
#define DELTA_T 1
#define K_e_P 15         // proportional gain for position error e
#define K_e_D 50        // derivative gain for position error e
#define K_alpha_P 20    // proportional gain for angular error alpha
#define K_alpha_D 70    // derivative gain for angular error alpha


// VARIABLES ////////////////////////////////////////////////////////

// SPI communication variables
volatile struct data_buffer receive_buffer;
volatile struct data_buffer send_buffer;
volatile int mode = 0; // 0 receiving, 1 sending.
volatile int transmission_status = 0;
volatile struct data_byte temp_data;
volatile int counter = 0;

// Distance to wall variables (from sensors)
volatile unsigned char distance_right_back = 0;
volatile unsigned char distance_right_front = 0;
volatile unsigned char distance_left_back = 0;
volatile unsigned char distance_left_front = 0;
volatile unsigned char distance_front = 0;
volatile unsigned char distance_back = 0;

// Driven distance variables
volatile double driven_distance = 0;
volatile unsigned char wheel_click = 0;
volatile unsigned char wheel_click_prior = 0;
volatile unsigned char no_forward = 0;

// Initiates control variables
volatile double e = 0; // Position error
volatile double alpha = 0; // Angle error

volatile double e_prior = 0;
volatile double alpha_prior = 0;
volatile double e_prior_prior = 0;
volatile double alpha_prior_prior = 0;

// SWITCH:
volatile int autonomous_mode; // 1 true, 0 false: remote control mode

// MISSION PHASES
volatile int8_t missionPhase = 7; // 0, 1, 2, 3, 4, 5, 6 or 7

// Tape variables:
volatile unsigned char tape = 0;
volatile unsigned char start_detected = 0;
volatile unsigned char start_line_crossed = 0;
volatile unsigned char goal_detected = 0;
volatile unsigned char tape_detected = 0;

// FUNCTION DECLARATIONS: ////////////////////////////////////////////
// INIT FUNCTION: ---------------------------------------------
void init_control_module(void);
void reset_values();

// COMMUNICATION FUNCTIONS: -----------------------------------
// Functions that are used in interrupts caused by the SPI-bus
void send_to_master(struct data_buffer* my_buffer);
void receive_from_master(struct data_buffer* my_buffer);

// In autonomous mode: get sensor values from receive buffer
void update_values_from_sensor();
void update_sensors_and_empty_receive_buffer();

// In remote control mode: use remote control values from receive buffer
void remote_control(char control_val);
void remote_control_mode();

// MOTOR FUNCTIONS: ----------------------------------------------
void set_speed_right_wheels(unsigned char new_speed_percentage);
void set_speed_left_wheels(unsigned char new_speed_percentage);

// Steering functions
void rotate_right(int speed);
void rotate_left(int speed);
void forward();
void slight_left();
void slight_right();
void sharp_left();
void sharp_right();
void backwards();
void stop();

// CONTROL FUNCTIONS: -----------------------------------------------
void go_forward(double * ptr_e, double * ptr_e_prior, double * ptr_e_prior_prior, double* ptr_alpha, double* ptr_alpha_prior, double* ptr_alpha_prior_prior );
double controller(double e, double alpha, double e_prior, double alpha_prior, double e_prior_prior, double alpha_prior_prior);
void setMotor(double u, double alpha);
double set_alpha(unsigned char distance_right_back, unsigned char distance_right_front, unsigned char distance_left_back, unsigned char distance_left_front);
void forward_slow();

// DIRECTION FUNCTIONS: -----------------------------------------
// Turn forward:
void turn_forward();

// Turn back:
void turn_back();
void turn_back_control_on_both_walls();
void turn_back_control_on_right_wall();
void turn_back_control_on_left_wall();
void turn_back_control_on_zero_walls();

// Turn left:
void turn_left();
void turn_left_control_on_right_wall();
void turn_left_control_on_zero_walls();
void turn_left_control_on_back_wall();
void turn_left_control_on_left_wall();

// Turn right:
void turn_right();
void turn_right_control_on_left_wall();
void turn_right_control_on_zero_walls();
void turn_right_control_on_back_wall();
void turn_right_control_on_right_wall();

// MAZE FUNCTIONS: -----------------------------------------------
unsigned char get_possible_directions();
void make_direction_decision();
void run_direction_command(unsigned char);
void update_driven_distance();
// Functions for driveable but unvisited positions
void add_unvisited();
int8_t unvisited_already_in_list(int8_t,int8_t);
void check_if_visited_explored();

// MISSION FUNCTIONS: --------------------------------------------
void mission();
void mission_phase_0(); // Cross the start line
void mission_phase_1(); // Explore the maze
void mission_phase_2(); // Go shortest way from current square to start square
void mission_phase_3(); // Roll into the start square, grab the object and turn 180 degrees 
void mission_phase_4(); // Go shortest way from start to goal
void mission_phase_5(); // Back until see tape, leave object, back and turn 180 degrees
void mission_phase_6(); // Go shortest way from current square to start square
void mission_phase_7(); // Stop, default value before start mission

// GRIPPING ARM FUNCTIONS: ---------------------------------------
// Functions for the arm:
void open_claw_gap(); // KLO
void close_claw_gap(); // KLO
void claw_full_open();

// FUNCTIONS FOR SINGLE BIT MANIPULATION: ------------------------
#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x)  (0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))

//MAP FUNCTIONS
void init_map();
void update_orientation(char turn);
