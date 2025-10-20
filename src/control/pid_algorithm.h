/**
 * @file pid_algorithm.h
 * @brief PID control algorithm implementation
 *
 * This file defines the PidControlAlgorithm class, which implements a PID
 * (Proportional-Integral-Derivative) control algorithm for the electronic load.
 * It provides precise control of current, voltage, power, and resistance modes.
 */

#ifndef PID_ALGORITHM_H
#define PID_ALGORITHM_H

#include "control_algorithm.h"
#include <PID.h>

/**
 * @brief PID control algorithm implementation
 *
 * This class implements a PID control algorithm for the electronic load system.
 * It can control the load in constant current, voltage, power, or resistance modes
 * by adjusting the gate control voltage based on sensor feedback.
 */
class PidControlAlgorithm final : public ControlAlgorithm {
    public:
        /**
         * @brief Default constructor
         *
         * Creates a PID controller with zero coefficients (no control action).
         * Useful for initialization or when coefficients will be set later.
         */
        PidControlAlgorithm() : PidControlAlgorithm(0, 0, 0) {}

        /**
         * @brief Constructor with PID coefficients
         * @param kp Proportional gain
         * @param ki Integral gain
         * @param kd Derivative gain
         *
         * Creates a PID controller with the specified coefficients.
         * The target is initialized to 0A current mode.
         */
        PidControlAlgorithm(const float kp, const float ki, const float kd)
            : gatePid(kp, ki, kd), target(Current(static_cast<uint32_t>(0)))
        {
            setTarget(target);
        }

        /**
         * @brief Add a new sensor reading to the PID controller
         * @param current Pointer to current measurement
         * @param voltage Pointer to voltage measurement
         * @param micros Timestamp of the reading in microseconds
         *
         * Updates the PID controller with new sensor readings. The method
         * converts the readings to the appropriate input for the current
         * control mode and updates the PID's internal state.
         */
        void addReading(Current* current, Voltage* voltage, unsigned long micros) override;

        /**
         * @brief Set the target value for the PID controller
         * @param value Target value (current, voltage, power, or resistance)
         *
         * Sets the desired target value. If the value type changes, the PID
         * controller is reset to prevent windup from the previous mode.
         */
        void setTarget(Value value) override;

        /**
         * @brief Update the control output based on current state
         * @param output Current gate control voltage output
         * @return New gate control voltage output
         *
         * Calculates the new gate control voltage using the PID algorithm.
         * The output is adjusted based on the control mode (voltage mode
         * uses inverse control logic).
         */
        GateControlVoltage* updateOutput(GateControlVoltage* output) override;

        /**
         * @brief Set PID coefficients
         * @param kp Proportional gain
         * @param ki Integral gain (will be divided by 1000 for time scaling)
         * @param kd Derivative gain (will be multiplied by 1000 for time scaling)
         *
         * Updates the PID coefficients. The integral and derivative gains
         * are scaled to account for the microsecond timing used in the system.
         */
        void setCoefficients(float kp, float ki, float kd);

    private:
        arc::PID<float> gatePid;  // PID controller instance
        Value target;             // Current target value
};
#endif // PID_ALGORITHM_H
