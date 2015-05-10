#ifndef STYR_DEFS_H
#define STYR_DEFS_H

#define F_CPU 14700000UL

#define WALLS_MAX_DISTANCE 22
#define FRONT_MAX_DISTANCE 12
#define BACK_MAX_DISTANCE 30
#define ROBOT_LENGTH 10

#define AUTONOMOUS_MODE 1
#define REMOTE_CONTROL_MODE 0

unsigned char distance_right_back;
unsigned char distance_right_front;
unsigned char distance_left_back;
unsigned char distance_left_front;
unsigned char distance_front;
unsigned char distance_back;
unsigned char wheel_click;
unsigned char goal_found;

unsigned char driven_distance;

#endif
