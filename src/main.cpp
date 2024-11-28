/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ericssonlin                                               */
/*    Created:      11/19/2024, 8:03:16 PM                                    */
/*    Description:  General Autonomous/Control Library for VEX Robotics       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------------------------------------------------
 *    To Do:
 *      * Verify Odometry with vex::inertial inertial sensor and dual vex::encoder motor encoder readings
 *      * Test autonomous chassis code (path following, rotation, etc.), tune PID controllers
 *      * Test basic control code, add game-specific logic (color mech, intake, pneumatics, color sensor, etc.)
 *      * Add competition logic (autonomous selector code)
 *      * Add README.md documentation and complete Doxygen comments
 *      * Test experimental code (C++20 concepts, etc.)
/*----------------------------------------------------------------------------------------------------------------------*/

#include <vex.h>
#include <chassis.hpp>

vex::brain brain;
vex::motor rm1(vex::PORT13, vex::ratio6_1, false), rm2(vex::PORT12, vex::ratio6_1, false), rm3(vex::PORT11, vex::ratio6_1, false);
vex::motor lm1(vex::PORT3, vex::ratio6_1, true), lm2(vex::PORT2, vex::ratio6_1, true), lm3(vex::PORT1, vex::ratio6_1, true);
MotorGroup right_group(&rm1, &rm2, &rm3), left_group(&lm1, &lm2, &lm3);
vex::encoder front_back_encoder(brain.ThreeWirePort.A), left_right_encoder(brain.ThreeWirePort.B); // PORTS!!
vex::inertial inertial_sensor(vex::PORT8); // PORTS!!)
Chassis base (
    &left_group, &right_group, /* Motor groups */ 
    &front_back_encoder, &left_right_encoder, &inertial_sensor, /* Sensors */
    31.4 /* Base width */,              4.25 /* Wheel radius */,
    12.5 /* Pursuit distance */,        0.052085391069929 /* Gear multiplier */, /*113/99*0.04563233376923-*/
    0.0, 0.0, 0.0 /* Initial pose */,   1 /* Thread sleep: 1 milliseconds */
);

#ifdef EXPERIMENTAL
    #if __cplusplus > 201703L
        #include <experimental/protocols.hpp>
        #include <concepts>

        template<AutonomousCompatible ChassisInstance>
        void autonomous(ChassisInstance & chassis) {
            chassis.forward(100, 0, 0);
        }

        template<ControllerCompatible ChassisInstance>
        void control(ChassisInstance & chassis) {
            vex::controller controller;
            void * void_ptr;
            vex::thread t(chassis.basic_control);
            while (true) {

            }
        }
        #else // EXPERIMENTAL
        void autonomous_tasks(Chassis & chassis) {
            chassis.forward(100, 0, 0);
        }

        void control_tasks(Chassis & chassis) {
            vex::controller controller;
            vex::thread t(Chassis::basic_control, &chassis);
            while (true) {

            }
        }
    #endif // __cplusplus > 201703L
#else
    void autonomous_tasks(Chassis & chassis) {
        chassis.forward(100, 0, 0);
    }

    void control_tasks(Chassis & chassis) {
        vex::controller controller;
        vex::thread t(Chassis::basic_control, &chassis);
        while (true) {

        }
    }
#endif // EXPERIMENTAL

void autonomous() {
    // selector code here
    autonomous_tasks(base);
}

void control() {
    // selector code here
    control_tasks(base);
}

int main() {
    vexDelay(1); // DO NOT REMOVE!!

    #ifdef COMPETITION
    vex::competition competition;
    competition.autonomous(autonomous);
    competition.drivercontrol(control);
    #else // COMPETITION
    autonomous();
    control();
    #endif // COMPETITION
    
    while (true) { vex::this_thread::sleep_for(100); } // DO NOT REMOVE!!
}