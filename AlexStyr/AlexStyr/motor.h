#ifndef MOTOR_H
#define MOTOR_H

#define NEGATIVE_LIMIT -5
#define SLIGHT_NEGATIVE_LIMIT -1
#define SLIGHT_POSITIVE_LIMIT 1
#define POSITIVE_LIMIT 5


void set_speed_right_wheels(unsigned char new_speed_percentage);
void set_speed_left_wheels(unsigned char new_speed_percentage);

#endif