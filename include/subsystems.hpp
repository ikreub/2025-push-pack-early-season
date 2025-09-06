#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/colors.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

extern Drive chassis;

//MCL sensors
inline pros::Distance MCLleft(13);
inline pros::Distance MCLright(16);
inline pros::Distance MCLback(15);
inline pros::Distance MCLfront(10);

//Intake motors
inline pros::Motor intake_basket(4);
inline pros::Motor intake_load(-3);
inline pros::Motor intake_front(5);
inline pros::Motor intake_outake(-17);

//color sort
inline pros::Optical sorter(1);

inline double front_read(){
    return MCLfront.get_distance();
}
inline double left_read(){
    return MCLleft.get_distance();
}
inline double right_read(){
    return MCLright.get_distance();
}
inline double back_read(){
    return MCLback.get_distance();
}