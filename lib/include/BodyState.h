#include "Gait.h"
#include "stdint.h"

#ifndef BODYSTATE
#define BODYSTATE

enum Leg{
    superior_right,
    superior_left,
    inferior_right,
    inferior_left
};

class BodyState{
public:
    // Setter Functions
    static void update_current_gait();
    static void set_current_gait(Gait current_gait);
    static void set_desired_gait(Gait desired_gait);
    static void set_velocity(double velocity);
    
    // Getter Functions
    static Gait get_current_gait();
    static Gait get_desired_gait();
    static double get_velocity();
    static uint32_t get_update_time();

private:
    static Gait current_gait;
    static Gait desired_gait;
    static double velocity;
    static uint32_t update_time;
};
#endif