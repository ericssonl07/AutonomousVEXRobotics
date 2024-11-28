#include <autonomous/motor.hpp>
#include <vex.h>
#include <vector>

int MotorGroup::count(void) {
    return motors.size();
}

void MotorGroup::setVelocity(double velocity, vex::velocityUnits units) {
    for (vex::motor* motor : motors) {
        motor -> setVelocity(velocity, units);
    }
}

void MotorGroup::setVelocity(double velocity, vex::percentUnits units) {
    for (vex::motor* motor : motors) {
        motor -> setVelocity(velocity, units);
    }
}

void MotorGroup::setStopping(vex::brakeType mode) {
    for (vex::motor* motor : motors) {
        motor -> setStopping(mode);
    }
}

void MotorGroup::resetPosition(void) {
    for (vex::motor* motor : motors) {
        motor -> resetPosition();
    }
}

void MotorGroup::setPosition(double value, vex::rotationUnits units) {
    for (vex::motor* motor : motors) {
        motor -> setPosition(value, units);
    }
}

void MotorGroup::setTimeout(int32_t time, vex::timeUnits units) {
    for (vex::motor* motor : motors) {
        motor -> setTimeout(time, units);
    }
}

void MotorGroup::spin(vex::directionType dir) {
    for (vex::motor* motor : motors) {
        motor -> spin(dir);
    }
}

void MotorGroup::spin(vex::directionType dir, double velocity, vex::velocityUnits units) {
    for (vex::motor* motor : motors) {
        motor -> spin(dir, velocity, units);
    }
}

void MotorGroup::spin(vex::directionType dir, double velocity, vex::percentUnits units) {
    for (vex::motor* motor : motors) {
        motor -> spin(dir, velocity, units);
    }
}

void MotorGroup::spin(vex::directionType dir, double voltage, vex::voltageUnits units) {
    for (vex::motor* motor : motors) {
        motor -> spin(dir, voltage, units);
    }
}

std::vector<bool> MotorGroup::spinTo(double rotation, vex::rotationUnits units, double velocity, vex::velocityUnits units_v, bool waitForCompletion) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> spinTo(rotation, units, velocity, units_v, waitForCompletion));
    }
    return v;
}

std::vector<bool> MotorGroup::spinToPosition(double rotation, vex::rotationUnits units, double velocity, vex::velocityUnits units_v, bool waitForCompletion) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> spinToPosition(rotation, units, velocity, units_v, waitForCompletion));
    }
    return v;
}

std::vector<bool> MotorGroup::spinTo(double rotation, vex::rotationUnits units, bool waitForCompletion) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> spinTo(rotation, units, waitForCompletion));
    }
    return v;
}

std::vector<bool> MotorGroup::spinToPosition(double rotation, vex::rotationUnits units, bool waitForCompletion) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> spinToPosition(rotation, units, waitForCompletion));
    }
    return v;
}

std::vector<bool> MotorGroup::spinFor(double rotation, vex::rotationUnits units, double velocity, vex::velocityUnits units_v, bool waitForCompletion) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> spinFor(rotation, units, velocity, units_v, waitForCompletion));
    }
    return v;
}

std::vector<bool> MotorGroup::spinFor(vex::directionType dir, double rotation, vex::rotationUnits units, double velocity, vex::velocityUnits units_v, bool waitForCompletion) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> spinFor(dir, rotation, units, velocity, units_v, waitForCompletion));
    }
    return v;
}

std::vector<bool> MotorGroup::spinFor(double rotation, vex::rotationUnits units, bool waitForCompletion) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> spinFor(rotation, units, waitForCompletion));
    }
    return v;
}

std::vector<bool> MotorGroup::spinFor(vex::directionType dir, double rotation, vex::rotationUnits units, bool waitForCompletion) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> spinFor(dir, rotation, units, waitForCompletion));
    }
    return v;
}

void MotorGroup::spinFor(double time, vex::timeUnits units, double velocity, vex::velocityUnits units_v) {
    for (vex::motor* motor : motors) {
        motor -> spinFor(time, units, velocity, units_v);
    }
}

void MotorGroup::spinFor(vex::directionType dir, double time, vex::timeUnits units, double velocity, vex::velocityUnits units_v) {
    for (vex::motor* motor : motors) {
        motor -> spinFor(dir, time, units, velocity, units_v);
    }
}

void MotorGroup::spinFor(double time, vex::timeUnits units) {
    for (vex::motor* motor : motors) {
        motor -> spinFor(time, units);
    }
}

void MotorGroup::spinFor(vex::directionType dir, double time, vex::timeUnits units) {
    for (vex::motor* motor : motors) {
        motor -> spinFor(dir, time, units);
    }
}

std::vector<bool> MotorGroup::isSpinning(void) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> isSpinning());
    }
    return v;
}

std::vector<bool> MotorGroup::isDone(void) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> isDone());
    }
    return v;
}

std::vector<bool> MotorGroup::isSpinningMode(void) {
    std::vector<bool> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> isSpinningMode());
    }
    return v;
}

void MotorGroup::stop(void) {
    for (vex::motor* motor : motors) {
        motor -> stop();
    }
}

void MotorGroup::stop(vex::brakeType mode) {
    for (vex::motor* motor : motors) {
        motor -> stop(mode);
    }
}

void MotorGroup::setMaxTorque(double value, vex::percentUnits units) {
    for (vex::motor* motor : motors) {
        motor -> setMaxTorque(value, units);
    }
}

void MotorGroup::setMaxTorque(double value, vex::torqueUnits units) {
    for (vex::motor* motor : motors) {
        motor -> setMaxTorque(value, units);
    }
}

void MotorGroup::setMaxTorque(double value, vex::currentUnits units) {
    for (vex::motor* motor : motors) {
        motor -> setMaxTorque(value, units);
    }
}

std::vector<vex::directionType> MotorGroup::direction(void) {
    std::vector<vex::directionType> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> direction());
    }
    return v;
}

std::vector<double> MotorGroup::position(vex::rotationUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> position(units));
    }
    return v;
}

std::vector<double> MotorGroup::velocity(vex::velocityUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> velocity(units));
    }
    return v;
}

std::vector<double> MotorGroup::velocity(vex::percentUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> velocity(units));
    }
    return v;
}

std::vector<double> MotorGroup::current(vex::currentUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> current(units));
    }
    return v;
}

std::vector<double> MotorGroup::current(vex::percentUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> current(units));
    }
    return v;
}

std::vector<double> MotorGroup::voltage(vex::voltageUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> voltage(units));
    }
    return v;
}

std::vector<double> MotorGroup::power(vex::powerUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> power(units));
    }
    return v;
}

std::vector<double> MotorGroup::torque(vex::torqueUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> torque(units));
    }
    return v;
}

std::vector<double> MotorGroup::efficiency(vex::percentUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> efficiency(units));
    }
    return v;
}

std::vector<double> MotorGroup::temperature(vex::percentUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> temperature(units));
    }
    return v;
}

std::vector<double> MotorGroup::temperature(vex::temperatureUnits units) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> temperature(units));
    }
    return v;
}

std::vector<double> MotorGroup::convertVelocity(double velocity, vex::velocityUnits units, vex::velocityUnits unitsout) {
    std::vector<double> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> convertVelocity(velocity, units, unitsout));
    }
    return v;
}

std::vector<vex::gearSetting> MotorGroup::getMotorCartridge() {
    std::vector<vex::gearSetting> v;
    for (vex::motor* motor : motors) {
        v.push_back(motor -> getMotorCartridge());
    }
    return v;
}