/**
 * @file control_loop.h
 * @brief I2C communication interface for the control loop
 *
 * This file defines the ControlLoop class, which handles I2C communication
 * for the electronic load system. It provides methods for responding to
 * I2C read and write requests, managing register access, and interfacing
 * with the load control system.
 */

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <load.h>
#include <Wire.h>

namespace communication
{
    // Forward declarations for register template classes
    template<typename D>
    class ReadRegister;

    template<typename D>
    class ReadWriteRegister;

    /**
     * @brief I2C communication interface for the control loop
     *
     * This class handles I2C communication for the electronic load system.
     * It manages register-based communication, allowing external controllers
     * to read status information and configure the load's operating mode.
     * The class uses a register-based protocol where each register contains
     * specific data structures for different aspects of the load's state.
     */
    class ControlLoop {
    public:
        /**
         * @brief Constructor
         * @param protocol Pointer to TwoWire (I2C) protocol instance
         * @param load Reference to the load control system
         *
         * Initializes the control loop with the I2C protocol and load system.
         * The control loop acts as an I2C peripheral device that responds to
         * read and write requests from external controllers.
         */
        explicit ControlLoop(TwoWire* protocol, load::Load& load): _protocol(protocol), _load(load) {}

        /**
         * @brief Handle I2C read request
         *
         * Called when an external controller requests data from the load.
         * The method reads the current register and sends the appropriate
         * data structure (StatusData, ModeData, or DebugData) over I2C.
         */
        void onRequest();

        /**
         * @brief Handle I2C write request
         * @param num_bytes Number of bytes received
         *
         * Called when an external controller sends data to the load.
         * The method processes the received data and updates the load's
         * configuration based on the register being written to.
         */
        void onReceive(int num_bytes);

    private:
        TwoWire * _protocol;              // I2C protocol instance
        load::Load& _load;                // Reference to load control system
        uint8_t _control_loop_address{}; // I2C address of this device
        uint8_t _current_register{};      // Currently selected register
        static constexpr uint8_t BUFFER_SIZE = 32;  // I2C buffer size
        uint8_t _buffer[BUFFER_SIZE];     // I2C communication buffer
    };

}

#endif //CONTROL_LOOP_H
