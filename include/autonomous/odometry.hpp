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
    /**
     * @brief The tracking function to update the position and rotation of the robot.
     * @param o The odometry object to track as a void pointer.
     */
    friend int track(void* o);

    /**
     * @brief Tracking thread for the odometry object- initialized automatically.
     */
    vex::thread tracking;

    /**
     * @brief The front-back encoder.
     * 
     * Parallel to the robot's velocity.
     */
    vex::encoder * front_back_encoder;

    /**
     * @brief The left-right encoder.
     * 
     * Perpendicular to the robot's velocity.
     */
    vex::encoder * left_right_encoder;

    /**
     * @brief The inertial sensor.
     */
    vex::inertial * inertial_sensor;

    /**
     * @brief The base width of the robot.
     */
    double base_width;

    /**
     * @brief The radius of the wheel.
     */
    double wheel_radius;

    /**
     * @brief The x position of the robot.
     */
    double x_position;

    /**
     * @brief The y position of the robot.
     */
    double y_position;

    /**
     * @brief The rotation of the robot, in radians.
     */
    double rotation_value;

    /**
     * @brief The time in milliseconds to sleep between iterations.
     */
    int thread_sleep;

public:

    /**
     * @brief Constructor for the odometry class.
     * 
     * @param front_back_encoder The front-back encoder.
     * @param left_right_encoder The left-right encoder.
     * @param inertial_sensor The inertial sensor.
     * @param base_width The width of the base.
     * @param wheel_radius The radius of the wheel.
     * @param thread_sleep The time in milliseconds to sleep between iterations.
     */
    Odometry(vex::encoder * front_back_encoder, vex::encoder * left_right_encoder, vex::inertial * inertial_sensor, double base_width, double wheel_radius, int thread_sleep);
    
    /**
     * @brief Retrieve the x coordinate.
     */
    double x();

    /**
     * @brief Retrieve the y coordinate.
     */
    double y();

    /**
     * @brief Retrieve the rotation, in radians.
     */
    double rotation();

    /**
     * @brief Set the pose of the robot.
     * 
     * @param x The x position.
     * @param y The y position.
     * @param rotation The rotation, in radians.
     */
    void set_pose(double x, double y, double rotation);
};

#endif // VEXROBOTICS_ODOMETRY_HPP