#ifndef SHORTEST_PATH_H_INCLUDED
#define SHORTEST_PATH_H_INCLUDED
#include "map.h"

typedef struct point
{
    int x;
    int y;
} point;

int command[50];
int c;
point end;
point path[50];

void fill_square(point p, int cost);

void getCommands(point end);

void traceBack(int costmap[17][17], point end);

void floodfill(point start, point end);
#endif
