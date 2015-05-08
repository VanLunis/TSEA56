#ifndef DECISION_MAKING_H
#define DECISION_MAKING_H

#include "styr_defs.h"
#include "styrcomm.h"
#include <util/delay.h>

#include "map.h"
#include "shortest_path.h"

point unexplored[50];
int u;

void make_decision();

// MAZE FUNCTIONS: -----------------------------------------------
unsigned char get_possible_directions();
void make_direction_decision();



#endif