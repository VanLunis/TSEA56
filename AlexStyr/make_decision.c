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
        unexplored[u].x = lx;
        unexplored[u++].y = ly;
    }

    if (!robot.fwall)
    {
        unexplored[u].x = fx;
        unexplored[u++].y = fy;
    }

    if (!robot.rwall)
    {
        unexplored[u].x = rx;
        unexplored[u].y = ry;
    }

    point temp = {robot.x, robot.y};
    floodfill(temp, unexplored[u]);
    traceBack(costmap,  unexplored[u]);
    getCommands(unexplored[u--]);
}
