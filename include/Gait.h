#include "ros.h"


struct Point{
    float x;
    float y;
    float z;
};

struct Gait{
    Point com;
    Point superior_right;
    Point superior_left;
    Point inferior_right;
    Point inferior_left;
};