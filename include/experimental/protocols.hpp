#ifndef VEXROBOTICS_PROTOCOLS_HPP_EXPERIMENTAL
#define VEXROBOTICS_PROTOCOLS_HPP_EXPERIMENTAL

#include <autonomous/motor.hpp>
#include <paths/path.hpp>
#include <vector>
#include <utility>
#include <type_traits>

template<typename U, typename V> struct _is_same : std::false_type {};
template<typename U> struct _is_same<U, U> : std::true_type {};
template<class U, class V> concept same_as_impl = _is_same<U, V>::value;
template<class U, class V> concept same_as = same_as_impl<U, V> and same_as_impl<V, U>;

/**
 * @brief Concept encoding the necessary method requirements for autonomous control:
 * 
 * - `follow_path` and overloads (necessary for path following)
 * 
 * - `turn_to` and `turn` (necessary for turning)
 * 
 * - `forward` (an optional, but often preferred method)
 * 
 * - `x`, `y`, `rotation` (necessary for positioning)
 * 
 * A class `OptionallyCompatible` satisfies the concept `AutonomousCompatible` if
 * it has all of the above methods.
 */
template<class OptionallyCompatible>
concept AutonomousCompatible = requires (OptionallyCompatible instance, 
                                         double tolerance, double speed_factor, int point_count,
                                         std::vector<double> x, std::vector<double> y,
                                         std::vector<std::pair<double, double>> points,
                                         Path path,
                                         double angle, double distance) {
    {
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
        instance.follow_path(x, y, tolerance, speed_factor, point_count) 
    } -> same_as<void>;
    {
        /**
         * @brief Follow a path defined by a set of points.
         * 
         * @param points The points that define the path.
         * @param tolerance The tolerance for the path following.
         * @param speed_factor The speed factor for the path following. Generally, lower speed factors lead to more stable behavior.
         * @param point_count The number of points to use. If `point_count = -1` then `len(points) * 10` points are used.
         */
        instance.follow_path(points, tolerance, speed_factor, point_count) 
    } -> same_as<void>;
    {
        /**
         * @brief Follow a path defined by a `Path` object.
         * 
         * @param path The path to follow.
         * @param tolerance The tolerance for the path following.
         * @param speed_factor The speed factor for the path following. Generally, lower speed factors lead to more stable behavior.
         */
        instance.follow_path(path, tolerance, speed_factor) 
    } -> same_as<void>;
    {
        /**
         * @brief Turn the robot to a specific heading.
         * 
         * Applies a function which intelligently accounts for coterminal angles.
         * 
         * @param angle The angle to turn to.
         * @param tolerance The tolerance for the turn.
         * @param speed_factor The speed factor for the turn. Generally, lower speed factors lead to more stable behavior.
         */
        instance.turn_to(angle, tolerance, speed_factor) 
    } -> same_as<void>;
    {
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
        instance.turn(angle, tolerance, speed_factor) 
    } -> same_as<void>;
    {
        /**
         * @brief Move the robot forward by a specific distance.
         * 
         * @param distance The distance to move forward by.
         * @param tolerance The tolerance for the forward movement.
         * @param speed_factor The speed factor for the forward movement. Generally, lower speed factors lead to more stable behavior.
         */
        instance.forward(distance, tolerance, speed_factor) 
    } -> same_as<void>;
    {
        /**
         * @brief Retrieve the x coordinate.
         * 
         * @return The x coordinate as a double.
         */
        instance.x() 
    } -> same_as<double>;
    {
        /**
         * @brief Retrieve the y coordinate.
         * 
         * @return The y coordinate as a double.
         */
        instance.y() 
    } -> same_as<double>;
    {
        /**
         * @brief Retrieve the rotation.
         * 
         * @return The rotation as a double, in radians.
         */
        instance.rotation() 
    } -> same_as<double>;
};

/**
 * @brief Concept encoding required basic control logic.
 * 
 * To satisfy this concept, the class must have the method `basic_control`
 * with `void *` as a parameter.
 * 
 * The method is intended to provide basic control functionality (movement),
 * which can be threaded in the `main.cpp` `control` function with other
 * functionality (game-specific motors, pneumatics, sensors, etc.).
 */
template<class OptionallyCompatible>
concept ControllerCompatible = requires (OptionallyCompatible instance, void * void_ptr) {
    { instance.left } -> same_as<MotorGroup * &>;
    { instance.right } -> same_as<MotorGroup * &>;
    {
        /**
         * @brief The basic control logic for the chassis. (Move forward, backward, turn, etc.)
         * 
         * @param void_ptr An empty pointer to satisfy VEX's `vex::thread` call signature.
         */
        instance.basic_control(void_ptr)
    } -> same_as<void>;
};

/**
 * @brief A chassis implementation is `CompetitionCompatible` if it is both
 * `AutonomousCompatible` and `ControllerCompatible`; that is, if it supports
 * autonomous functionality and basic controller logic.
 */
template<class OptionallyCompatible>
concept CompetitionCompatible = AutonomousCompatible<OptionallyCompatible> and ControllerCompatible<OptionallyCompatible>;

template<class OptionallyCompatible>
concept OdometryCompatible = requires (OptionallyCompatible instance) {
    {
        /**
         * @brief Retrieve the x coordinate.
         * 
         * @return The x coordinate as a double.
         */
        instance.x() 
    } -> same_as<double>;
    {
        /**
         * @brief Retrieve the y coordinate.
         * 
         * @return The y coordinate as a double.
         */
        instance.y() 
    } -> same_as<double>;
    {
        /**
         * @brief Retrieve the rotation.
         * 
         * @return The rotation as a double, in radians.
         */
        instance.rotation() 
    } -> same_as<double>;
};

#endif // VEXROBOTICS_PROTOCOLS_HPP_EXPERIMENTAL