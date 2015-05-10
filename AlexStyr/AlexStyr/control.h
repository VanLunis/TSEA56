#ifndef CONTROL_H
#define CONTROL_H

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

// Initiates control variables
double e; // Position error
double alpha; // Angle error
    
double e_prior;
double alpha_prior;
double e_prior_prior;
double alpha_prior_prior;
unsigned char wheel_click_prior;

void go_forward(double * ptr_e, double * ptr_e_prior, double * ptr_e_prior_prior, double* ptr_alpha, double* ptr_alpha_prior, double* ptr_alpha_prior_prior );
double controller(double e, double alpha, double e_prior, double alpha_prior, double e_prior_prior, double alpha_prior_prior);
void setMotor(double u, double alpha);
double set_alpha(unsigned char distance_right_back, unsigned char distance_right_front, unsigned char distance_left_back, unsigned char distance_left_front);
void forward_slow();
unsigned char update_driven_distance(unsigned char driven_distance, unsigned char wheel_click, unsigned char wheel_click_prior );

void set_speed_right_wheels(unsigned char new_speed_percentage);
void set_speed_left_wheels(unsigned char new_speed_percentage);

#endif