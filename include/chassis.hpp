#ifndef VEXROBOTICS_CHASSIS_HPP
#define VEXROBOTICS_CHASSIS_HPP

#include <autonomous/odometry.hpp>
#include <autonomous/motor.hpp>
#include <paths/path.hpp>
#include <vector>

/**
 * @brief A chassis class to control the robot. Conforms to the `CompetitionCompatible` concept.
 */
class Chassis {

    /**
     * @brief The odometry object to track the robot's position.
     */
    Odometry odometry;

    /**
     * @brief The width of the base.
     */
    double base_width;

    /**
     * @brief The pursuit distance for the pure pursuit algorithm.
     */
    double pursuit_distance;

    /**
     * @brief The time in milliseconds to sleep between iterations.
     */
    int thread_sleep;

public:

    /**
     * @brief The left motor group.
     */
    MotorGroup * left;

    /**
     * @brief The right motor group.
     */
    MotorGroup * right;

    /**
     * @brief The front-back encoder.
     */
    vex::encoder * front_back_encoder;

    /**
     * @brief The left-right encoder.
     */
    vex::encoder * left_right_encoder;

    /**
     * @brief The inertial sensor.
     */
    vex::inertial * inertial_sensor;

    /**
     * @brief Constructor for the `Chassis` class.
     * 
     * @param left The left motor group's address.
     * @param right The right motor group's address.
     * @param base_width The width of the base.
     * @param wheel_radius The radius of the wheel.
     * @param pursuit_distance The pursuit distance for the pure pursuit algorithm.
     * @param gear_multiplier A constant scaling factor to account for gear ratio effects.
     * @param initial_x The initial x position of the robot.
     * @param initial_y The initial y position of the robot.
     * @param initial_rotation The initial rotation of the robot.
     * @param thread_sleep The time in milliseconds to sleep between iterations.
     */
    Chassis(MotorGroup * left, MotorGroup * right, vex::encoder * front_back_encoder, vex::encoder * left_right_encoder, vex::inertial * inertial_sensor, double base_width, double wheel_radius, double pursuit_distance, double gear_multiplier, double initial_x, double initial_y, double initial_rotation, int thread_sleep);
    
    /**
     * @brief Follow a path defined by a set of points.
     * 
     * @param x The x-coordinates of the points.
     * @param y The y-coordinates of the points.
     * @param tolerance The tolerance for the path following.
     * @param speed_factor The speed factor for the path following. Generally, lower speed factors lead to more stable behavior.
     * @param point_count The number of points to use. If `point_count = -1` then `len(x) * 10` points are used.
     * @throws `std::logic_error` if the number of x-coordinates and y-coordinates do not match.
     */
    void follow_path(std::vector<double> x, std::vector<double> y, double tolerance, double speed_factor = 1.0, int point_count = -1);
    
    /**
     * @brief Follow a path defined by a set of points.
     * 
     * @param points The points that define the path.
     * @param tolerance The tolerance for the path following.
     * @param speed_factor The speed factor for the path following. Generally, lower speed factors lead to more stable behavior.
     * @param point_count The number of points to use. If `point_count = -1` then `len(points) * 10` points are used.
     */
    void follow_path(std::vector<std::pair<double, double>> points, double tolerance, double speed_factor = 1.0, int point_count = -1);
    
    /**
     * @brief Follow a path defined by a `Path` object.
     * 
     * @param path The path to follow.
     * @param tolerance The tolerance for the path following.
     * @param speed_factor The speed factor for the path following. Generally, lower speed factors lead to more stable behavior.
     */
    void follow_path(Path path, double tolerance = 5.0, double speed_factor = 1.0);
    
    /**
     * @brief Turn the robot to a specific heading.
     * 
     * Applies a function which intelligently accounts for coterminal angles.
     * 
     * @param angle The angle to turn to.
     * @param tolerance The tolerance for the turn.
     * @param speed_factor The speed factor for the turn. Generally, lower speed factors lead to more stable behavior.
     */
    void turn_to(double angle, double tolerance = 0.01, double speed_factor = 1.0);

    /**
     * @brief Turn the robot by a specific angle.
     * 
     * Note that this function does NOT account for coterminal angles- so, turning by 720 degrees
     * will result in two full rotations.
     * 
     * @param angle The angle to turn by.
     * @param tolerance The tolerance for the turn.
     * @param speed_factor The speed factor for the turn. Generally, lower speed factors lead to more stable behavior.
     */
    void turn(double angle, double tolerance = 0.01, double speed_factor = 1.0);

    /**
     * @brief Move the robot forward by a specific distance.
     * 
     * @param distance The distance to move forward by.
     * @param tolerance The tolerance for the forward movement.
     * @param speed_factor The speed factor for the forward movement. Generally, lower speed factors lead to more stable behavior.
     */
    void forward(double distance, double tolerance = 5.0, double speed_factor = 1.0);

    /**
     * @brief Retrieve the x coordinate.
     * @return The x coordinate as a double.
     */
    double x();

    /**
     * @brief Retrieve the y coordinate.
     * @return The y coordinate as a double.
     */
    double y();

    /**
     * @brief Retrieve the rotation.
     * @return The rotation as a double, in radians.
     */
    double rotation();

    /**
     * @brief The basic control logic for the chassis. (Move forward, backward, turn, etc.)
     * 
     * @param void_ptr A pointer to the `Chassis` object.
     */
    static void basic_control(void * instance);

};

/**
 * @brief A function to display the coordinates of the robot.
 * 
 * @param b The `Odometry` object from which to read positioning information.
 */
void coordinate_display(void * b);

#endif // VEXROBOTICS_CHASSIS_HPP