#ifndef VEXROBOTICS_ODOMETRY_HPP
#define VEXROBOTICS_ODOMETRY_HPP

#include <vex.h>
#include <autonomous/motor.hpp>
#include <vector>
#include <cmath>

/**
 * @brief The odometry class to track the position and rotation of the robot.
 * @attention The odometry class will only function within scope, so make sure it is managed by another object.
*/
class Odometry {
    friend int track(void* o);
    vex::thread tracking;
    vex::encoder * front_back_encoder;
    vex::encoder * left_right_encoder;
    vex::inertial * inertial_sensor;
    double base_width;
    double wheel_radius;
    double gear_multiplier;
    double x_position;
    double y_position;
    double rotation_value;
    int thread_sleep;
public:
    Odometry(vex::encoder * front_back_encoder, vex::encoder * left_right_encoder, vex::inertial * inertial_sensor, double base_width, double wheel_radius, double gear_multiplier, int thread_sleep);
    double x();
    double y();
    double rotation();
    void set_pose(double x, double y, double rotation);
};

#endif // VEXROBOTICS_ODOMETRY_HPP