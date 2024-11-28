#ifndef VEXROBOTICS_PURE_PURSUIT_HPP
#define VEXROBOTICS_PURE_PURSUIT_HPP

#include <paths/path.hpp>
#include <vector>
#include <utility>

/**
 * @brief A class that implements the pure pursuit algorithm.
 * 
 * The pure pursuit algorithm is a path tracking algorithm that
 * calculates the steering angle of a robot to follow a path.
 * It is based on the concept of a lookahead point, which is
 * a point on the path that the robot is trying to reach, ahead
 * of the robot's current position by a certain distance.
 * 
 * The algorithm calculates the steering angle based on the
 * distance between the robot and the lookahead point, and the
 * curvature of the path at the lookahead point.
 * 
 * By constantly "pursuing" the lookahead point, the robot can
 * follow the path accurately and smoothly.
 */
class Pursuit {

    /**
     * @brief The index of the last found point on the path.
     * 
     * This variable allows the implementation of forward-searching
     * the path without repetition and back-tracing. The advantage
     * of forward-searching is that it allows the program to trace
     * a closed path, whereas back-tracing would not (it would find
     * the end point and return that).
     */
    int last_found_idx;

    /**
     * @brief The lookahead distance.
     * 
     * This is the distance between the robot and the lookahead point.
     */
    double lookahead;

    /**
     * @brief The path to follow.
     */
    Path path;

public:

    /**
     * @brief Construct a new Pursuit object.
     * 
     * @param path The path to follow.
     * @param lookahead_distance The lookahead distance.
     */
    Pursuit(Path path, double lookahead_distance);

    /**
     * @brief Construct a new Pursuit object.
     * 
     * @param x The x-coordinates of the path.
     * @param y The y-coordinates of the path.
     * @param num_points The number of points in the path.
     * @param lookahead_distance The lookahead distance.
     */
    Pursuit(std::vector<double> x, std::vector<double> y, int num_points, double lookahead_distance);

    /**
     * @brief Reference to the lookahead distance.
     * 
     * @return The lookahead distance (modifiable reference) for get and set.
     */
    double & lookahead_distance();

    /**
     * @brief Reference to the path.
     * 
     * @return The path (modifiable reference) for get and set.
     */
    Coordinate2D get_target(double x_bot, double y_bot);

    /**
     * @brief Get the relative steering angle.
     * 
     * This function calculates the relative steering angle
     * based on the robot's current position and orientation,
     * and the width of the robot.
     * 
     * @param x_bot The x-coordinate of the robot.
     * @param y_bot The y-coordinate of the robot.
     * @param theta_bot The orientation of the robot.
     * @param width_bot The width of the robot.
     * @return A pair of doubles representing the left and right steering angles.
     */
    std::pair<double, double> get_relative_steering(double x_bot, double y_bot, double theta_bot, double width_bot);
    
};

#endif // VEXROBOTICS_PURE_PURSUIT_HPP