#ifndef GAIT
#define GAIT

typedef struct Point{
    double x;
    double y;
    double z;
}Point;


typedef struct Gait{
    Point com;
    Point superior_right;
    Point superior_left;
    Point inferior_right;
    Point inferior_left;
}Gait;
#endif