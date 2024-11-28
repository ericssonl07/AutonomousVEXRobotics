#ifndef VEXROBOTICS_MOTOR_GROUP_HPP
#define VEXROBOTICS_MOTOR_GROUP_HPP

#include <vex.h>
#include <vector>

/**
 * @brief The MotorGroup class implements a motor group, but data are taken 
 * from all motors and returned as a vector (VEX's native `vex::motor_group` class
 * will return the requested reading of the first motor in the motor group only).
*/
class MotorGroup {

    /**
     * @brief Add a motor pointer to the motor group by pushing it to memory.
     * @param m The vex::motor pointer to add to the group.
     * @note This is the specialized add_motor method which addresses the base case (1 param).
     * For additional motors, use the variadic template add_motor method.
    */
    void add_motor(vex::motor* m) {
        motors.push_back(m);
    }

    /**
     * @brief Add motor pointers to the motor group by pushing it to memory.
     * @param m1 The vex::motor pointer to add to the group.
     * @param m The variadic parameter pack of vex::motor pointers to add to the group.
     * @note This is the variadic template add_motor method which addresses the recursive case (n params).
    */
    template<typename... Args>
    void add_motor(vex::motor* m1, Args ... m) {
        motors.push_back(m1);
        add_motor(m...);
    }

public:
    /**
     * @brief The motors std::vector contains all the motor pointers in the group.
    */
    std::vector<vex::motor*> motors;

    /**
     * @brief Construct a new MotorGroup object.
     * @note Default constructor; default-initializes the motors std::vector.
    */
    MotorGroup(): motors() {}

    /**
     * @brief Construct a new MotorGroup object.
     * @param m1 The first vex::motor.
     * @param m The variadic parameter pack of vex::motor pointers to add to the group.
    */
    template<typename... Args>
    MotorGroup(vex::motor* m1, Args ... m): MotorGroup() {
        add_motor(m1, m...);
    }

    /**
     * @brief Retrieve the number of motors in the group.
     * @return The number of motors as an integer.
    */
    int count(void);

    /**
     * @brief Set the velocity of all motors in the group.
     * @param velocity The velocity to set the motors to.
     * @param units The velocity units as vex::velocityUnits.
    */
    void setVelocity(double velocity, vex::velocityUnits units);

    /**
     * @brief Set the velocity of all motors in the group.
     * @param velocity The velocity to set the motors to.
     * @param units The velocity units as vex::percentUnits.
    */
    void setVelocity(double velocity, vex::percentUnits units);

    /**
     * @brief Set the stopping mode of all motors in the group.
     * @param mode The brake type to set the motors to.
    */
    void setStopping(vex::brakeType mode);

    /**
     * @brief Reset the motors' positions.
    */
    void resetPosition(void);

    /**
     * @brief Set the position of all motors in the group.
     * @param value The position to set the motors to.
     * @param units The rotation units as vex::rotationUnits.
    */
    void setPosition(double value, vex::rotationUnits units);

    /**
     * @brief Sets the timeout for the motor. If the motor does not reach its' 
       commanded position prior to the completion of the timeout, the motor will stop.
     * @param time The timeout value.
     * @param units The time units as vex::timeUnits.
    */
    void setTimeout(int32_t time, vex::timeUnits units);

    /**
     * @brief Spin the motors in the group.
     * @param dir The direction as vex::directionType.
    */
    void spin(vex::directionType dir);

    /**
     * @brief Spin the motors in the group.
     * @param dir The direction as vex::directionType.
     * @param velocity The velocity to spin the motors at.
     * @param units The velocity units as vex::velocityUnits.
    */
    void spin(vex::directionType dir, double velocity, vex::velocityUnits units);

    /**
     * @brief Spin the motors in the group.
     * @param dir The direction as vex::directionType.
     * @param velocity The velocity to spin the motors at.
     * @param units The velocity units as vex::percentUnits.
    */
    void spin(vex::directionType dir, double velocity, vex::percentUnits units);

    /**
     * @brief Spin the motors in the group.
     * @param dir The direction as vex::directionType.
     * @param voltage The voltage to put through the circuit.
     * @param units The velocity units as vex::voltageUnits.
    */
    void spin(vex::directionType dir, double voltage, vex::voltageUnits units);

    /**
     * @brief Spin the motors in the group to a position.
     * @param rotation The rotation value to spin to.
     * @param units The units of rotation as vex::rotationUnits.
     * @param velocity The velocity to spin at.
     * @param units_v The velocity units as vex::velocityUnits.
     * @param waitForCompletion Whether to wait for the motors to finish spinning.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> spinTo(double rotation, vex::rotationUnits units, double velocity, vex::velocityUnits units_v, bool waitForCompletion = true);

    /**
     * @brief Spin the motors in the group to a position.
     * @param rotation The rotation value to spin to.
     * @param units The units of rotation as vex::rotationUnits.
     * @param velocity The velocity to spin at.
     * @param units_v The velocity units as vex::velocityUnits.
     * @param waitForCompletion Whether to wait for the motors to finish spinning.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> spinToPosition(double rotation, vex::rotationUnits units, double velocity, vex::velocityUnits units_v, bool waitForCompletion = true);
    
    /**
     * @brief Spin the motors in the group to a position.
     * @param rotation The rotation value to spin to.
     * @param units The units of rotation as vex::rotationUnits.
     * @param waitForCompletion Whether to wait for the motors to finish spinning.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> spinTo(double rotation, vex::rotationUnits units, bool waitForCompletion = true);

    /**
     * @brief Spin the motors in the group to a position.
     * @param rotation The rotation value to spin to.
     * @param units The units of rotation as vex::rotationUnits.
     * @param waitForCompletion Whether to wait for the motors to finish spinning.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> spinToPosition(double rotation, vex::rotationUnits units, bool waitForCompletion = true);
    
    /**
     * @brief Spin the motors in the group to a position.
     * @param rotation The rotation value to spin to.
     * @param units The units of rotation as vex::rotationUnits.
     * @param velocity The velocity to spin at.
     * @param units_v The velocity units as vex::velocityUnits.
     * @param waitForCompletion Whether to wait for the motors to finish spinning.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> spinFor(double rotation, vex::rotationUnits units, double velocity, vex::velocityUnits units_v, bool waitForCompletion = true);
    
    /**
     * @brief Spin the motors in the group to a position.
     * @param dir The direction as vex::directionType.
     * @param rotation The rotation value to spin to.
     * @param units The units of rotation as vex::rotationUnits.
     * @param velocity The velocity to spin at.
     * @param units_v The velocity units as vex::velocityUnits.
     * @param waitForCompletion Whether to wait for the motors to finish spinning.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> spinFor(vex::directionType dir, double rotation, vex::rotationUnits units, double velocity, vex::velocityUnits units_v, bool waitForCompletion = true);
    
    /**
     * @brief Spin the motors in the group to a position.
     * @param rotation The rotation value to spin to.
     * @param units The units of rotation as vex::rotationUnits.
     * @param waitForCompletion Whether to wait for the motors to finish spinning.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> spinFor(double rotation, vex::rotationUnits units, bool waitForCompletion = true);
    
    /**
     * @brief Spin the motors in the group to a position.
     * @param dir The direction as vex::directionType.
     * @param rotation The rotation value to spin to.
     * @param units The units of rotation as vex::rotationUnits.
     * @param waitForCompletion Whether to wait for the motors to finish spinning.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> spinFor(vex::directionType dir, double rotation, vex::rotationUnits units, bool waitForCompletion = true);
    
    /**
     * @brief Spin the motors in the group for a set time.
     * @param time The time to spin for.
     * @param units The time units as vex::timeUnits.
     * @param velocity The velocity to spin at.
     * @param units_v The velocity units as vex::velocityUnits.
    */
    void spinFor(double time, vex::timeUnits units, double velocity, vex::velocityUnits units_v);
    
    /**
     * @brief Spin the motors in the group for a set time.
     * @param dir The direction as vex::directionType.
     * @param time The time to spin for.
     * @param units The time units as vex::timeUnits.
     * @param velocity The velocity to spin at.
     * @param units_v The velocity units as vex::velocityUnits.
    */
    void spinFor(vex::directionType dir, double time, vex::timeUnits units, double velocity, vex::velocityUnits units_v);
    
    /**
     * @brief Spin the motors in the group for a set time.
     * @param time The time to spin for.
     * @param units The time units as vex::timeUnits.
    */
    void spinFor(double time, vex::timeUnits units);
    
    /**
     * @brief Spin the motors in the group for a set time.
     * @param dir The direction as vex::directionType.
     * @param time The time to spin for.
     * @param units The time units as vex::timeUnits.
    */
    void spinFor(vex::directionType dir, double time, vex::timeUnits units);

    /**
     * @brief Retrieve the status of motors spinning.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> isSpinning(void);
    
    /**
     * @brief Retrieve the status of motors stopping.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> isDone(void);

    /**
     * @brief Retrieve the status of motors spinning.
     * @return A vector representing the status for each motor in the group.
    */
    std::vector<bool> isSpinningMode(void);

    /**
     * @brief Stop the motors in the group.
    */
    void stop(void);

    /**
     * @brief Stop the motors in the group.
     * @param mode The brake type to stop the motors with.
    */
    void stop(vex::brakeType mode);

    /**
     * @brief Set the maximum torque of the motors in the group.
     * @param value The maximum torque value to set.
     * @param units The torque units as vex::percentUnits.
     */
    void setMaxTorque(double value, vex::percentUnits units);

    /**
     * @brief Set the maximum torque of the motors in the group.
     * @param value The maximum torque value to set.
     * @param units The torque units as vex::torqueUnits.
     */
    void setMaxTorque(double value, vex::torqueUnits units);

    /**
     * @brief Set the maximum torque of the motors in the group.
     * @param value The maximum torque value to set.
     * @param units The torque units as vex::currentUnits.
     */
    void setMaxTorque(double value, vex::currentUnits units);

    /**
     * @brief Get the direction of the motors in the group.
     * @return A vector of vex::directionType indicating the direction of each motor in the group.
     */
    std::vector<vex::directionType> direction(void);

    /**
     * @brief Get the position of the motors in the group.
     * @param units The rotation units as vex::rotationUnits.
     * @return A vector of double values representing the position of each motor in the group.
     */
    std::vector<double> position(vex::rotationUnits units);

    /**
     * @brief Get the velocity of the motors in the group.
     * @param units The velocity units as vex::velocityUnits.
     * @return A vector of double values representing the velocity of each motor in the group.
     */
    std::vector<double> velocity(vex::velocityUnits units);

    /**
     * @brief Get the velocity of the motors in the group.
     * @param units The percent units as vex::percentUnits.
     * @return A vector of double values representing the velocity of each motor in the group.
     */
    std::vector<double> velocity(vex::percentUnits units);

    /**
     * @brief Get the current of the motors in the group.
     * @param units The current units as vex::currentUnits. Default is vex::currentUnits::amp.
     * @return A vector of double values representing the current of each motor in the group.
     */
    std::vector<double> current(vex::currentUnits units = vex::currentUnits::amp);

    /**
     * @brief Get the current of the motors in the group.
     * @param units The percent units as vex::percentUnits.
     * @return A vector of double values representing the current of each motor in the group.
     */
    std::vector<double> current(vex::percentUnits units);

    /**
     * @brief Get the voltage of the motors in the group.
     * @param units The voltage units as vex::voltageUnits. Default is vex::voltageUnits::volt.
     * @return A vector of double values representing the voltage of each motor in the group.
     */
    std::vector<double> voltage(vex::voltageUnits units = vex::voltageUnits::volt);

    /**
     * @brief Get the power of the motors in the group.
     * @param units The power units as vex::powerUnits. Default is vex::powerUnits::watt.
     * @return A vector of double values representing the power of each motor in the group.
     */
    std::vector<double> power(vex::powerUnits units = vex::powerUnits::watt);

    /**
     * @brief Get the torque of the motors in the group.
     * @param units The torque units as vex::torqueUnits. Default is vex::torqueUnits::Nm.
     * @return A vector of double values representing the torque of each motor in the group.
     */
    std::vector<double> torque(vex::torqueUnits units = vex::torqueUnits::Nm);

    /**
     * @brief Get the efficiency of the motors in the group.
     * @param units The percent units as vex::percentUnits. Default is vex::percentUnits::pct.
     * @return A vector of double values representing the efficiency of each motor in the group.
     */
    std::vector<double> efficiency(vex::percentUnits units = vex::percentUnits::pct);

    /**
     * @brief Get the temperature of the motors in the group.
     * @param units The percent units as vex::percentUnits. Default is vex::percentUnits::pct.
     * @return A vector of double values representing the temperature of each motor in the group.
     */
    std::vector<double> temperature(vex::percentUnits units = vex::percentUnits::pct);

    /**
     * @brief Get the temperature of the motors in the group.
     * @param units The temperature units as vex::temperatureUnits.
     * @return A vector of double values representing the temperature of each motor in the group.
     */
    std::vector<double> temperature(vex::temperatureUnits units);

    /**
     * @brief Convert the velocity of the motors in the group.
     * @param velocity The velocity.
     * @param units The velocity units in vex::velocityUnits.
     * @param unitsout The output units as vex::velocityUnits, default: vex::velocityUnits::rpm.
     * @return A vector of double values representing the converted velocity of each motor in the group.
    */
    std::vector<double> convertVelocity(double velocity, vex::velocityUnits units, vex::velocityUnits unitsout = vex::velocityUnits::rpm);

    /**
     * @brief Get the motor cartridge of the motors in the group.
     * @return A vector of vex::gearSetting values representing the gear setting of each motor in the group.
    */
    std::vector<vex::gearSetting> getMotorCartridge();
    
};

#endif // VEXROBOTICS_MOTOR_GROUP_HPP