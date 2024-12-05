#include <chassis.hpp>
#include <autonomous/odometry.hpp>
#include <autonomous/pursuit.hpp>
#include <autonomous/motor.hpp>
#include <autonomous/pid.hpp>
#include <paths/path.hpp>
#include <vector>
#include <utility>
#include <cmath>

Chassis::Chassis(MotorGroup * left, MotorGroup * right,
                 vex::rotation * front_back_encoder, vex::rotation * left_right_encoder, vex::inertial * inertial_sensor,
                 double base_width, double wheel_radius,
                 double pursuit_distance,
                 double initial_x, double initial_y, double initial_rotation,
                 int thread_sleep):
                 odometry(front_back_encoder, left_right_encoder, inertial_sensor, base_width, wheel_radius, thread_sleep),
                 base_width(base_width), pursuit_distance(pursuit_distance), 
                 thread_sleep(thread_sleep),
                 left(left), right(right),
                 front_back_encoder(front_back_encoder), left_right_encoder(left_right_encoder), inertial_sensor(inertial_sensor) {

}

void Chassis::follow_path(std::vector<double> x, std::vector<double> y, double tolerance, double speed_factor, int point_count) {
    Path path(x, y, point_count);
    follow_path(path, tolerance, speed_factor);
}

void Chassis::follow_path(std::vector<std::pair<double, double>> points, double tolerance, double speed_factor, int point_count) {
    std::vector<double> x_values;
    std::vector<double> y_values;
    for (auto [x, y] : points) {
        x_values.push_back(x);
        y_values.push_back(y);
    }
    follow_path(x_values, y_values, tolerance, speed_factor, point_count);
}

void Chassis::follow_path(Path path, double tolerance, double speed_factor) {
    const double kp = 1.0;
    const double ki = 0.0;
    const double kd = 0.0;
    const double minimum_output = 0.5;
    const double maximum_output = 100.0;
    const double gamma = 0.999;
    PID linear_pid_controller(kp, ki, kd, minimum_output, maximum_output, gamma);
    Pursuit pursuit_controller(path, pursuit_distance);
    linear_pid_controller.set_target(0);
    double current_distance = path.distance_to_end(odometry.x(), odometry.y());
    while (current_distance > tolerance) {
        auto [steering_left, steering_right] = pursuit_controller.get_relative_steering(odometry.x(), odometry.y(), odometry.rotation(), base_width);
        double pid_factor = linear_pid_controller.calculate(-current_distance);
        double raw_left_power = steering_left * pid_factor;
        double raw_right_power = steering_right * pid_factor;
        double normalization_factor = 100.0 / (fabs(raw_left_power) + fabs(raw_right_power));
        double normalized_left_power = raw_left_power * normalization_factor;
        double normalized_right_power = raw_right_power * normalization_factor;
        left -> spin(vex::fwd, normalized_left_power * speed_factor, vex::pct);
        right -> spin(vex::fwd, normalized_right_power * speed_factor, vex::pct);
        current_distance = path.distance_to_end(odometry.x(), odometry.y());
        vex::this_thread::sleep_for(thread_sleep);
    }
    left -> stop();
    right -> stop();
}

void Chassis::turn_to(double angle, double tolerance, double speed_factor) {
    static auto nearest_coterminal = [] (double rotation, double target) -> double {
        static double pi2 = 3.14159265358979323846 * 2, divpi2 = 1 / pi2;
        return floor((rotation - target + 3.14159265358979323846) * divpi2) * pi2 + target;
    };
    double target_rotation = nearest_coterminal(odometry.rotation(), angle);
    double change = target_rotation - odometry.rotation();
    turn(change, tolerance, speed_factor);
}

void Chassis::turn(double angle, double tolerance, double speed_factor) {
    double target = odometry.rotation() + angle;
    PID pid_controller(45.0, 0.0025, 10.0, 0, 100, 0.9999); // kp, ki, kd, min, max, gamma
    pid_controller.set_target(target);
    while (fabs(odometry.rotation() - target) > tolerance) {
        double output = pid_controller.calculate(odometry.rotation()) * speed_factor;
        left -> spin(vex::fwd, -output, vex::pct);
        right -> spin(vex::fwd, output, vex::pct);
        vex::this_thread::sleep_for(thread_sleep);
    }
    left -> stop();
    right -> stop();
}

void Chassis::forward(double distance, double tolerance, double speed_factor) {
    double x1 = odometry.x();
    double y1 = odometry.y();
    double rotation = odometry.rotation();
    double x2 = x1 + distance * cos(rotation);
    double y2 = y1 + distance * sin(rotation);
    follow_path({x1, x2}, {y1, y2}, tolerance, speed_factor, 4);
}

double Chassis::x() {
    return odometry.x();
}

double Chassis::y() {
    return odometry.y();
}

double Chassis::rotation() {
    return odometry.rotation();
}

void Chassis::basic_control(void * instance) {
    Chassis * chassis_instance = (Chassis *) instance;
    vex::controller controller;
    double linear_speed, turn_speed, left_speed, right_speed;
    while (true) {
        linear_speed = controller.Axis3.position();
        turn_speed = controller.Axis1.position();
        left_speed = linear_speed + turn_speed;
        right_speed = linear_speed - turn_speed;
        if (fabs(left_speed) > 0.5) {
            chassis_instance -> left -> spin(vex::fwd, left_speed, vex::pct);
        } else {
            chassis_instance -> left -> stop(vex::brakeType::brake);
        }
        if (fabs(right_speed) > 0.5) {
            chassis_instance -> right -> spin(vex::fwd, right_speed, vex::pct);
        } else {
            chassis_instance -> right -> stop(vex::brakeType::brake);
        }
        vex::this_thread::sleep_for(10);
    }
}

void coordinate_display(void * b) {
    vex::brain brain;
    Chassis * base = (Chassis *) b;
    while (true) {
        brain.Screen.setCursor(1, 1);
        printf("(%.5f, %.5f, %.5f), %.5f\n", base -> x(), base -> y(), base -> rotation(), base -> inertial_sensor -> rotation(vex::rotationUnits::deg));
        brain.Screen.print("(%.5f, %.5f, %.5f)", base -> x(), base -> y(), base -> rotation());
        // printf("(%.5f, %.5f)\n", base -> odometry.x(), base -> odometry.y());
        vex::this_thread::sleep_for(50);
    }
    return;
}