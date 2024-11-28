#ifndef VEXROBOTICS_PATH_HPP
#define VEXROBOTICS_PATH_HPP

#include <paths/matrix.hpp>
#include <vector>

/**
 * @brief A lightweight structure to represent a 2D coordinate.
 */
struct Coordinate2D {

    /**
     * @brief The x-coordinate of the point.
     */
    double x;
    
    /**
     * @brief The y-coordinate of the point.
     */
    double y;

    /**
     * @brief Default constructor for the `Coordinate2D` class.
     * Initializes the point at the origin `(0, 0)`.
     */
    Coordinate2D();

    /**
     * @brief Constructor for the `Coordinate2D` class.
     * 
     * @param x The x-coordinate of the point.
     * @param y The y-coordinate of the point.
     */
    Coordinate2D(double x, double y);

};

/**
 * @brief A lightweight structure to represent a cubic expression.
 */
struct CubicExpression {

    /**
     * @brief The coefficient of the cubic term.
     */
    double a;

    /**
     * @brief The coefficient of the quadratic term.
     */
    double b;

    /**
     * @brief The coefficient of the linear term.
     */
    double c;

    /**
     * @brief The constant term.
     */
    double d;

    /**
     * @brief Default constructor for the `CubicExpression` class.
     * 
     * Initializes the cubic expression as `0x^3 + 0x^2 + 0x + 0`.
     */
    CubicExpression();

    /**
     * @brief Constructor for the `CubicExpression` class.
     * 
     * Initializes the cubic expression as `ax^3 + bx^2 + cx + d`.
     * 
     * @param a The coefficient of the cubic term.
     * @param b The coefficient of the quadratic term.
     * @param c The coefficient of the linear term.
     * @param d The constant term.
     */
    CubicExpression(double a, double b, double c, double d);

};

/**
 * @brief A class to represent a cubic spline.
 * 
 * A cubic spline is a piecewise-defined function that is continuous and has continuous first and second derivatives.
 * It is defined by a set of cubic expressions, each defined on a separate interval.
 * 
 * The cubic spline is constructed by solving a system of linear equations to find the coefficients of the cubic expressions.
 * The system of equations is constructed by enforcing continuity and smoothness conditions at the boundaries of the intervals.
 * These conditions directly correlate with expressions relating the coefficients of the cubic pieces.
 */
class CubicSpline {

    friend class Path;

    /**
     * @brief The matrix of operations to solve for the coefficients of the cubic spline.
     * 
     * This is a 4n by (4n + 1) matrix, where `n` is the number of intervals and `n+1` is the number of points.
     */
    Matrix operations;

    /**
     * @brief The coefficients of the cubic expressions.
     */
    std::vector<CubicExpression> coefficients;

    /**
     * @brief The x-values of the points. These also define the boundaries for the piecewise cubic expressions.
     */
    std::vector<double> piecewise_boundaries;

    /**
     * @brief linspace helper function to generate a vector of linearly spaced values.
     * 
     * @return A vector of evenly spaced values from `start` to `end` with `point_count` points.
     */
    static std::vector<double> linspace(double start, double end, int point_count);

public:

    /**
     * @brief Get the segment of the cubic spline that an x-coordinate would lie in.
     * 
     * @param of The x-coordinate to check.
     * @return The index of the segment that the x-coordinate lies in.
     */
    int get_segment(double of);

    /**
     * @brief Construct a cubic spline from a set of points.
     * 
     * @param x_values The x-coordinates of the points.
     * @param y_values The y-coordinates of the points.
     */
    CubicSpline(std::vector<double> x_values, std::vector<double> y_values);

    /**
     * @brief Call operator to evaluate the cubic spline at a given list of x-coordinates.
     * 
     * @param values The x-coordinates to evaluate the cubic spline at.
     */
    std::vector<double> operator() (std::vector<double> values);

};

class Path {

    friend class Pursuit;

    friend class Chassis;

    /**
     * @brief The points that define the path.
     */
    std::vector<Coordinate2D> points;

    /**
     * @brief The distance from a point to the end of the path.
     * 
     * This distance is calculated by summing the distances from the point to the closest waypoint
     * with the distances between the waypoints to the end of the path (starting from the closest waypoint).
     * 
     * @param point The point to calculate the distance from.
     * @return The distance from the point to the end of the path.
     */
    double distance_to_end(Coordinate2D point);

    /**
     * @brief The distance from a point to the end of the path.
     * 
     * This distance is calculated by summing the distances from the point to the closest waypoint
     * with the distances between the waypoints to the end of the path (starting from the closest waypoint).
     * 
     * @param x The x-coordinate of the point.
     * @param y The y-coordinate of the point.
     * @return The distance from the point to the end of the path.
     */
    double distance_to_end(double x, double y);

    /**
     * @brief Helper function that both constructors utilize to construct the path.
     * 
     * @param x The x-coordinates of the points.
     * @param y The y-coordinates of the points.
     * @param point_count The number of points to use.
     */
    void construct_path(std::vector<double> x, std::vector<double> y, int point_count);

public:

    /**
     * @brief Construct a path from a set of points.
     * 
     * @param x The x-coordinates of the points.
     * @param y The y-coordinates of the points.
     * @param point_count The number of points to use. If `point_count = -1` then `len(x) * 10` points are used.
     * @throws `std::logic_error` if the number of x-coordinates and y-coordinates do not match.
     */
    Path(std::vector<double> x, std::vector<double> y, int point_count = -1);

    /**
     * @brief Construct a path from a set of points.
     * 
     * @param points The points that define the path.
     * @param point_count The number of points to use. If `point_count = -1` then `len(points) * 10` points are used.
     */
    Path(std::vector<std::pair<double, double>> points, int point_count = -1);
};

#endif // VEXROBOTICS_PATH_HPP