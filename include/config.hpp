#ifndef VEXROBOTICS_CONFIG_HPP
#define VEXROBOTICS_CONFIG_HPP

#include <vex.h>
#include <autonomous/motor.hpp>
#include <chassis.hpp>

enum Side {
    RED_1 = 1 << 1,
    RED_2 = 1 << 2,
    BLUE_1 = 1 << 3,
    BLUE_2 = 1 << 4
};

vex::brain brain;
vex::motor rm1(vex::PORT13, vex::ratio6_1, false), rm2(vex::PORT12, vex::ratio6_1, false), rm3(vex::PORT11, vex::ratio6_1, false);
vex::motor lm1(vex::PORT3, vex::ratio6_1, true), lm2(vex::PORT2, vex::ratio6_1, true), lm3(vex::PORT1, vex::ratio6_1, true);
MotorGroup right_group(&rm1, &rm2, &rm3), left_group(&lm1, &lm2, &lm3);
vex::inertial inertial_sensor(vex::PORT20, vex::turnType::left);
vex::rotation front_back_encoder(vex::PORT19, true), left_right_encoder(vex::PORT18, false);
Chassis base (
    &left_group, &right_group, /* Motor groups */ 
    &front_back_encoder, &left_right_encoder, &inertial_sensor, /* Sensors */
    31.4 /* Base width */,              4.25 /* Wheel radius */,
    12.5 /* Pursuit distance */,
    0.0, 0.0, 0.0 /* Initial pose */,   1 /* Thread sleep: 1 milliseconds */
);
Side autonomous_configuration;
int presses;
vex::bumper autonomous_selector = vex::bumper(brain.ThreeWirePort.C);

namespace _IMPLEMENTATION {
    void _count() {
        bool last_state = false;
        while (true) {
            bool pressing = autonomous_selector.pressing();
            if (pressing and not last_state) {
                if (presses < 4) {
                    ++presses;
                }
            }
            last_state = pressing;
            vexDelay(10);
        }
    }
}

void selector(int delay_msec = 5000) {
    presses = 0;
    vex::thread t(_IMPLEMENTATION::_count);
    vexDelay(delay_msec);
    if (presses == 0) {
        presses = 1;
    }
    autonomous_configuration = static_cast<Side>(1 << presses);
}

#endif // VEXROBOTICS_CONFIG_HPP