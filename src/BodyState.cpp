#include "BodyState.h"
#include <time.h>

Gait BodyState::current_gait = {};
Gait BodyState::desired_gait = {};
double BodyState::velocity = 0;
uint32_t BodyState::update_time = 0;


void BodyState::update_current_gait(){
    BodyState::current_gait = desired_gait;
    update_time = millis();
}
void BodyState::set_current_gait(Gait current_gait){
    BodyState::current_gait = current_gait;
    update_time = millis();
}
void BodyState::set_desired_gait(Gait desired_gait){
    BodyState::desired_gait = desired_gait;
}
void BodyState::set_velocity(double velocity){
    BodyState::velocity = velocity;
}

Gait BodyState::get_current_gait(){
    return current_gait;
}
Gait BodyState::get_desired_gait(){
    return desired_gait;
}
double BodyState::get_velocity(){
    return velocity;
}
uint32_t BodyState::get_update_time(){
    return update_time;
}