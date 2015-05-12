#ifndef STEERING_H
#define STEERING_H

#include "control.h"
// Define speeds
#define FULL_SPEED 55 // 60
#define SLOW_SPEED 35 // 40
#define VERY_SLOW_SPEED 20// 25

void rotate_right(int speed);
void rotate_left(int speed);
void forward();
void slight_left();
void slight_right();
void sharp_left();
void sharp_right();
void backwards();
void stop();


#endif
