/**
 * @file control_algorithm.h
 * @brief Abstract base class for control algorithms
 *
 * This file defines the abstract base class for all control algorithms used
 * in the electronic load system. It provides a common interface for different
 * control strategies (PID, etc.) to manage the load's behavior.
 */

#ifndef CONTROL_ALGORITHM_H
#define CONTROL_ALGORITHM_H

#include <cstdint>
#include <shared/value.h>

/**
 * @brief Abstract base class for control algorithms
 *
 * This class defines the interface that all control algorithms must implement.
 * It provides methods for adding sensor readings, setting targets, and updating
 * the control output. The electronic load system uses this interface to work
 * with different control strategies without being coupled to specific implementations.
 */
class ControlAlgorithm {
    public:
        /**
         * @brief Virtual destructor
         *
         * Virtual destructor ensures proper cleanup when deleting derived objects
         * through base class pointers. This is essential for proper memory management
         * in polymorphic hierarchies.
         */
        virtual ~ControlAlgorithm() = default;

        /**
         * @brief Add a new sensor reading to the control algorithm
         * @param current Pointer to current measurement
         * @param voltage Pointer to voltage measurement
         * @param micros Timestamp of the reading in microseconds
         *
         * This method is called whenever new sensor readings are available.
         * The control algorithm can use these readings to update its internal state
         * and calculate the next control output.
         */
        virtual void addReading(Current* current, Voltage* voltage, unsigned long micros) = 0;

        /**
         * @brief Set the target value for the control algorithm
         * @param value Target value (current, voltage, power, or resistance)
         *
         * Sets the desired target value that the control algorithm should try to achieve.
         * The value type determines the control mode (constant current, voltage, power, or resistance).
         */
        virtual void setTarget(Value value) = 0;

        /**
         * @brief Update the control output based on current state
         * @param output Current gate control voltage output
         * @return New gate control voltage output
         *
         * Calculates and returns the new gate control voltage based on the current
         * sensor readings and target value. The output is used to control the MOSFET
         * gate voltage in the electronic load circuit.
         */
        virtual GateControlVoltage* updateOutput(GateControlVoltage* output) = 0;
};

#endif // CONTROL_ALGORITHM_H