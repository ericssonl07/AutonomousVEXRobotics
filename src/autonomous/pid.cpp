#include <autonomous/pid.hpp>
#include <iostream>

PID::PID(double kp, double ki, double kd, double minimum_output, double maximum_output, double gamma):
    kp(kp), ki(ki), kd(kd), minimum_output(minimum_output), 
    maximum_output(maximum_output), gamma(gamma) {
    reset();
    if (minimum_output > maximum_output) {
        std::cerr << "Minimum output cannot be greater than maximum output";
        return;
    }
    if (kp < 0.0 || ki < 0.0 || kd < 0.0) {
        std::cerr << "PID constants cannot be negative";
        return;
    }
    if (gamma < 0.0 || gamma > 1.0) {
        std::cerr << "Gamma must be between 0 and 1";
        return;
    }
}

void PID::set_target(double target) {
    this -> target = target;
    this -> integral = 0.0; 
    this -> last_error = 0.0;
}

double PID::calculate(double value) {
    double error = target - value;
    double derivative = error - last_error;
    integral *= gamma;
    integral += error;
    last_error = error;
    double output = kp * error + ki * integral + kd * derivative;
    bool is_negative = false;
    if (output < 0.0) {
        is_negative = true;
        output = -output;
    }
    if (output > maximum_output) output = maximum_output;
    if (output < minimum_output) output = minimum_output;
    if (is_negative) {
        return -output;
    }
    return output;
}

void PID::reset() {
    integral = 0.0;
    last_error = 0.0;
}