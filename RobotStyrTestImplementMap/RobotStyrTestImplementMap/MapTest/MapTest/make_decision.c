#include "make_decision.h"

//Skall kallas i beslutpunkter
void make_decision()
{
    int lx = robot.x-robot.ydir;
    int ly = robot.y+robot.xdir;

    //analog to lx/ly for right cell
    int rx = robot.x+robot.ydir;
    int ry = robot.y-robot.xdir;

    //forward cell
    int fx = robot.x+robot.ydir;
    int fy = robot.y+robot.xdir;

    if (!robot.lwall)
    {
        unexplored[un].x = lx;
        unexplored[un++].y = ly;
    }

    if (!robot.fwall)
    {
        unexplored[un].x = fx;
        unexplored[un++].y = fy;
    }

    if (!robot.rwall)
    {
        unexplored[un].x = rx;
        unexplored[un].y = ry;
    }
 
    point temp = {robot.x, robot.y};
    floodfill(temp, unexplored[un]);
    traceBack(costmap,  unexplored[un]);
    getCommands(unexplored[un--]);
}
