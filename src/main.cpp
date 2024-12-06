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
#include <config.hpp>

void autonomous_tasks(Chassis & chassis, Side autonomous_configuration) {
    chassis.forward(100, 0, 0);
}

void control_tasks(Chassis & chassis) {
    vex::controller controller;
    vex::thread t(Chassis::basic_control, &chassis);
    while (true) {

    }
}

void autonomous() {
    int selector_delay_msec = 5000;
    selector(selector_delay_msec);
    autonomous_tasks(base, autonomous_configuration);
}

void control() {
    control_tasks(base);
}

int main() {
    vexDelay(1); // DO NOT REMOVE!!
    vexDelay(2000); 
    left_right_encoder.resetPosition(); front_back_encoder.resetPosition(); inertial_sensor.resetRotation();
    vex::thread t(coordinate_display, &base);

    #ifdef COMPETITION
    vex::competition competition;
    competition.autonomous(autonomous);
    competition.drivercontrol(control);
    #else // COMPETITION
    // autonomous();
    // control();
    #endif // COMPETITION
    
    while (true) { vex::this_thread::sleep_for(100); } // DO NOT REMOVE!!
}