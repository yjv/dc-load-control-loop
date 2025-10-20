/**
 * @file pid_algorithm.cpp
 * @brief PID control algorithm implementation
 *
 * This file implements the PidControlAlgorithm class methods for controlling
 * the electronic load using a PID control algorithm. It handles different
 * control modes (current, voltage, power, resistance) and provides precise
 * regulation of the load's behavior.
 */

#include "pid_algorithm.h"

/**
 * @brief Add a new sensor reading to the PID controller
 * @param current Pointer to current measurement
 * @param voltage Pointer to voltage measurement
 * @param micros Timestamp of the reading in microseconds
 *
 * Updates the PID controller with new sensor readings. The method converts
 * the readings to the appropriate input for the current control mode:
 * - Current mode: uses current reading directly
 * - Voltage mode: uses voltage reading directly
 * - Power mode: converts power target to current target, uses current reading
 * - Resistance mode: converts resistance target to current target, uses current reading
 */
void PidControlAlgorithm::addReading(Current* current, Voltage* voltage, const unsigned long micros)
{
    float input = 0;
    float target = this->target.getValue();

    switch (this->target.getType())
    {
    case ValueType::CURRENT:
        // Direct current control: use current reading as input
        input = current->getValue();
        break;
    case ValueType::VOLTAGE:
        // Direct voltage control: use voltage reading as input
        input = voltage->getValue();
        break;
    case ValueType::POWER:
        // Power control: convert power target to current target
        // P = V * I, so I = P / V
        target = this->target.getValue() / std::max(voltage->getValue(), voltage->getMinValue());
        input = current->getValue();
        break;
    case ValueType::RESISTANCE:
        // Resistance control: convert resistance target to current target
        // R = V / I, so I = V / R
        target = voltage->getValue() / std::max(this->target.getValue(), this->target.getMinValue());
        input = current->getValue();
        break;
    default:
        // Handle unexpected value type
        ESP_LOGE("PidControlAlgorithm", "Unhandled ValueType in PidControlAlgorithm::addReading");
        std::terminate();
    }

    // Update PID controller with new target and input
    gatePid.setTarget(target);
    gatePid.setInput(input, micros);
}

/**
 * @brief Set the target value for the PID controller
 * @param target Target value (current, voltage, power, or resistance)
 *
 * Sets the desired target value. If the value type changes, the PID
 * controller is reset to prevent integral windup from the previous mode.
 * This ensures clean transitions between different control modes.
 */
void PidControlAlgorithm::setTarget(Value target)
{
    // Reset PID controller if value type changes to prevent windup
    if (target.getType() != this->target.getType())
    {
        gatePid = arc::PID<float>(gatePid.getKp(), gatePid.getKi(), gatePid.getKd());
    }

    // Update target and set PID target
    this->target = target;
    gatePid.setTarget(target.getValue());
}

/**
 * @brief Update the control output based on current state
 * @param currentOutput Current gate control voltage output
 * @return New gate control voltage output
 *
 * Calculates the new gate control voltage using the PID algorithm.
 * The output is adjusted based on the control mode:
 * - Current/Power/Resistance modes: add PID output to current voltage
 * - Voltage mode: subtract PID output from current voltage (inverse control)
 */
GateControlVoltage* PidControlAlgorithm::updateOutput(GateControlVoltage* currentOutput)
{
    // Get PID output (error correction)
    const float delta = gatePid.getOutput();
    float control_voltage_value = currentOutput->getValue();

    if (this->target.getType() != ValueType::VOLTAGE)
    {
        // For current, power, and resistance modes: add PID output
        // Higher current/power/resistance requires higher gate voltage
        control_voltage_value += delta;
    }
    else
    {
        // For voltage mode: subtract PID output (inverse control)
        // Higher voltage requires lower gate voltage (less current)
        control_voltage_value -= delta;
    }

    return new GateControlVoltage(control_voltage_value);
}

/**
 * @brief Set PID coefficients
 * @param kp Proportional gain
 * @param ki Integral gain (will be divided by 1000 for time scaling)
 * @param kd Derivative gain (will be multiplied by 1000 for time scaling)
 *
 * Updates the PID coefficients. The integral and derivative gains are scaled
 * to account for the microsecond timing used in the system:
 * - Integral gain is divided by 1000 to convert from milliseconds to microseconds
 * - Derivative gain is multiplied by 1000 to convert from milliseconds to microseconds
 */
void PidControlAlgorithm::setCoefficients(const float kp, const float ki, const float kd)
{
    gatePid.setKp(kp);
    gatePid.setKi(ki / 1000.0f);  // Scale integral gain for microsecond timing
    gatePid.setKd(kd * 1000.0f);  // Scale derivative gain for microsecond timing
}
