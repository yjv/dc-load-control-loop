/**
 * @file register.h
 * @brief Register-based communication interface
 *
 * This file defines the register system for I2C communication with the
 * electronic load. It provides template classes for read-only and read-write
 * registers, along with specific register implementations for status, mode,
 * and debug data access.
 */

#ifndef REGISTER_H
#define REGISTER_H

#include <shared/communication/data.h>
#include <load.h>

namespace communication
{
    /**
     * @brief Template base class for read-only registers
     * @tparam D Data type that the register handles
     *
     * This template class defines the interface for read-only registers.
     * It provides a virtual method to get data from the load system.
     */
    template<typename D>
    class ReadRegister
    {
    public:
        /**
         * @brief Virtual destructor
         *
         * Virtual destructor ensures proper cleanup when deleting derived objects
         * through base class pointers. This is essential for proper memory management
         * in polymorphic hierarchies.
         */
        virtual ~ReadRegister() = default;

        /**
         * @brief Get data from the load system
         * @param load Pointer to the load system
         * @return Data structure containing the register's data
         *
         * Pure virtual method that must be implemented by derived classes
         * to retrieve the appropriate data from the load system.
         */
        virtual D getData(load::Load* load) const = 0;
    };

    /**
     * @brief Template class for read-write registers
     * @tparam D Data type that the register handles
     *
     * This template class extends ReadRegister to add write capability.
     * It provides methods for both reading and writing data to the load system.
     */
    template<typename D>
    class ReadWriteRegister: public ReadRegister<D>
    {
    public:
        /**
         * @brief Virtual destructor
         *
         * Virtual destructor ensures proper cleanup when deleting derived objects
         * through base class pointers. This is essential for proper memory management
         * in polymorphic hierarchies.
         */
        ~ReadWriteRegister() override = default;

        /**
         * @brief Set data in the load system
         * @param data Data structure to write
         * @param load Pointer to the load system
         *
         * Pure virtual method that must be implemented by derived classes
         * to write data to the load system.
         */
        virtual void setData(D data, load::Load* load) const = 0;
    };

    /**
     * @brief Status register implementation
     *
     * This register provides read-only access to the load's current status,
     * including voltage, current, temperature, load mode, and fault code.
     */
    class StatusRegister final : public ReadRegister<StatusData> {
    public:
        /**
         * @brief Get status data from the load
         * @param load Pointer to the load system
         * @return StatusData containing current load status
         *
         * Retrieves the current status information from the load system,
         * including measurements and operational state.
         */
        StatusData getData(load::Load* load) const override;
    };

    /**
     * @brief Debug register implementation
     *
     * This register provides read-only access to debug information about
     * the load's internal state, including gate control voltage and error counts.
     */
    class DebugRegister final : public ReadRegister<DebugData> {
    public:
        /**
         * @brief Get debug data from the load
         * @param load Pointer to the load system
         * @return DebugData containing debug information
         *
         * Retrieves debug information from the load system, including
         * internal measurements and error statistics.
         */
        DebugData getData(load::Load* load) const override;
    };

    /**
     * @brief Mode register implementation
     *
     * This register provides read-write access to the load's operating mode
     * and setpoint configuration. It allows external controllers to configure
     * the load's behavior.
     */
    class ModeRegister final : public ReadWriteRegister<ModeData> {
    public:
        /**
         * @brief Get mode data from the load
         * @param load Pointer to the load system
         * @return ModeData containing current mode and setpoint
         *
         * Retrieves the current operating mode and setpoint from the load system.
         */
        ModeData getData(load::Load* load) const override;

        /**
         * @brief Set mode data in the load
         * @param data ModeData containing new mode and setpoint
         * @param load Pointer to the load system
         *
         * Updates the load's operating mode and setpoint based on the provided data.
         * If the mode is OFF, the setpoint is automatically set to 0A current.
         */
        void setData(ModeData data, load::Load* load) const override;
    };

    /**
     * @brief Control loop registers container
     *
     * This class provides static instances of all available registers
     * for the control loop communication system. It serves as a central
     * registry for accessing different types of load data.
     */
    class ControlLoopRegisters {
    public:
        static const StatusRegister STATUS;  // Status register instance
        static const DebugRegister DEBUG;    // Debug register instance
        static const ModeRegister MODE;      // Mode register instance
    };

}

#endif //REGISTER_H
