#ifndef VEXROBOTICS_PID_HPP
#define VEXROBOTICS_PID_HPP

/**
 * @brief PID controller class
 * 
 * This class implements a general PID controller with output clamping.
 */
class PID {

    /**
     * @brief PID controller parameters
     * 
     * These constants of proportionality, integration, and differentiation are used to calculate the output of the PID controller.
     * They should be fine-tuned as hyperparameters of the program.
     */
    double kp, ki, kd;

    /**
     * @brief The target value for the PID controller to reach
     */
    double target;

    /**
     * @brief The integral of the error over time
     */
    double integral;

    /**
     * @brief The last error value
     */
    double last_error;

    /**
     * @brief The minimum and maximum output values
     * 
     * The output of the PID controller is clamped to these values.
     */
    double minimum_output, maximum_output;

    /**
     * @brief The gamma value for the integral term
     * 
     * This value acts as a "discount" factor for the integral term,
     * which prevents the integral term from accumulating too much error
     * and has the effect of making the integral term "forget" older errors.
     * It should be a value between 0 and 1 (this is not enforced by the class).
     */
    double gamma;

public:

    /**
     * @brief Construct a new PID controller object
     * 
     * @param kp The proportionality constant
     * @param ki The integration constant
     * @param kd The differentiation constant
     * @param minimum_output The minimum output value
     * @param maximum_output The maximum output value
     * @param gamma The gamma value for the integral term
     */
    PID(double kp, double ki, double kd, double minimum_output = 0.5, double maximum_output = 1.0, double gamma = 0.9999);

    /**
     * @brief Set the target value for the PID controller
     * 
     * @param target The target value
     */
    void set_target(double target);

    /**
     * @brief Calculate the output of the PID controller
     * 
     * @param value The current value
     * @return The output of the PID controller
     */
    double calculate(double value);

    /**
     * @brief Reset the PID controller
     * 
     * This function resets the integral term and the last error value.
     */
    void reset();
    
};

#endif // VEXROBOTICS_PID_HPP