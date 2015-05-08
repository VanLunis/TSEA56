#include "test.h"
#include "map.h"

void init_floodtest(){
    explored[3][3]=1;
    driveable[3][3]=1;
    explored[3][4]=1;
    driveable[3][4]=1;
    explored[3][5]=1;
    driveable[3][5]=1;
    explored[3][6]=1;
    driveable[3][6]=1;
    explored[4][6]=1;
    driveable[4][6]=1;
    explored[5][6]=1;
    driveable[5][6]=1;
    explored[6][6]=1;
    driveable[6][6]=1;
    explored[2][6]=1;
    driveable[2][6]=1;
    explored[1][6]=1;
    driveable[1][6]=1;
    explored[6][7]=1;
    driveable[6][7]=1;
    explored[6][8]=1;
    driveable[6][8]=1;
    explored[7][8]=1;
    driveable[7][8]=1;
    explored[8][8]=1;
    driveable[8][8]=1;
    explored[6][9]=1;
    driveable[6][9]=1;
    explored[6][10]=1;
    driveable[6][10]=1;
    explored[7][10]=1;
    driveable[7][10]=1;
    explored[8][10]=1;
    driveable[8][10]=1;
    explored[8][9]=1;
    driveable[8][9]=1;
    explored[8][8]=1;
    driveable[8][8]=1;
}