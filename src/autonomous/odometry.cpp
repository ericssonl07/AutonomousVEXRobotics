#include <autonomous/odometry.hpp>
#include <vex.h>
#include <vector>
#include <cmath>

#define M_PI 3.14159265358979323846

int track(void* o) {
    Odometry* odometry = (Odometry*) o;
    odometry -> left_right_encoder -> resetRotation();
    odometry -> front_back_encoder -> resetRotation();
    odometry -> inertial_sensor -> resetRotation();
    double last_left_position = 0.0;
    double last_drift = 0.0;
    while (true) {
        double left = odometry -> front_back_encoder -> position(vex::rotationUnits::deg) * M_PI / 180.0 * odometry -> wheel_radius;
        double delta_left = left - last_left_position;
        double rotation = odometry -> inertial_sensor -> rotation(vex::rotationUnits::deg) * M_PI / 180.0;
        double delta_rotation = rotation - odometry -> rotation_value;

        double translation_length;
        if (fabs(delta_rotation) < 1e-7) {
            translation_length = delta_left;
        } else {
            double radius = delta_left / delta_rotation;
            translation_length = (radius + odometry -> base_width * 0.5) * sin(delta_rotation * 0.5) * 2;
        }
        double drift = odometry -> left_right_encoder -> position(vex::rotationUnits::deg) * M_PI / 180.0;
        double delta_drift = drift - last_drift;

        double cosine = cos(odometry -> rotation_value + delta_rotation * 0.5);
        double sine = sin(odometry -> rotation_value + delta_rotation * 0.5);

        double translation_x = cosine * delta_drift - sine * translation_length;
        double translation_y = sine * delta_drift + cosine * translation_length;

        odometry -> x_position += translation_x;
        odometry -> y_position += translation_y;
        odometry -> rotation_value += delta_rotation;

        last_left_position = left;
        last_drift = drift;

        vex::this_thread::sleep_for(std::chrono::microseconds(odometry -> thread_sleep));
    }
}

Odometry::Odometry(vex::encoder * front_back_encoder, vex::encoder * left_right_encoder, vex::inertial * inertial_sensor, double base_width, double wheel_radius, int thread_sleep):
    front_back_encoder(front_back_encoder), left_right_encoder(left_right_encoder), inertial_sensor(inertial_sensor), base_width(base_width), wheel_radius(wheel_radius), thread_sleep(thread_sleep) {
    x_position = 0.0;
    y_position = 0.0;
    rotation_value = 0.0;
    tracking = vex::thread(track, this);
}

double Odometry::x() {
    return Odometry::x_position;
}

double Odometry::y() {
    return Odometry::y_position;
}

double Odometry::rotation() {
    return Odometry::rotation_value;
}

void Odometry::set_pose(double x, double y, double rotation) {
    Odometry::x_position = x;
    Odometry::y_position = y;
    Odometry::rotation_value = rotation;
}