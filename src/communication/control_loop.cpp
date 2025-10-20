/**
 * @file control_loop.cpp
 * @brief I2C communication interface implementation
 *
 * This file implements the ControlLoop class methods for handling I2C
 * communication with external controllers. It provides register-based
 * access to the load's status, configuration, and debug information.
 */

#include "control_loop.h"

#include <shared/communication/data.h>
#include "register.h"

namespace communication
{
    /**
     * @brief Handle I2C read request
     *
     * Called when an external controller requests data from the load.
     * The method reads the current register and sends the appropriate
     * data structure (StatusData, ModeData, or DebugData) over I2C.
     * The data is converted to network byte order before transmission.
     */
    void ControlLoop::onRequest()
    {
        size_t size;

        switch (_current_register)
        {
        case Registers::STATUS.ADDRESS:
            {
                // Read status data (voltage, current, temperature, mode, fault code)
                StatusData status_data = ControlLoopRegisters::STATUS.getData(&_load);
                size = StatusData::SIZE;
                status_data.toNetwork(Buffer(_buffer, size));
            }
            break;
        case Registers::MODE.ADDRESS:
            {
                // Read mode data (load mode and setpoint)
                ModeData mode_data = ControlLoopRegisters::MODE.getData(&_load);
                size = ModeData::SIZE;
                mode_data.toNetwork(Buffer(_buffer, size));
            }
            break;
        case Registers::DEBUG.ADDRESS:
            {
                // Read debug data (gate voltage, error counts, statistics)
                DebugData debug_data = ControlLoopRegisters::DEBUG.getData(&_load);
                size = DebugData::SIZE;
                debug_data.toNetwork(Buffer(_buffer, size));
            }
            break;
        default:
            // Handle unknown register
            ESP_LOGE("ControlLoop", "Unknown register: %d", _current_register);
            abort();
        }

        // Send the data over I2C
        _protocol->write(_buffer, size);
    }

    /**
     * @brief Handle I2C write request
     * @param num_bytes Number of bytes received
     *
     * Called when an external controller sends data to the load.
     * The method processes the received data and updates the load's
     * configuration based on the register being written to.
     * Currently only the MODE register supports write operations.
     */
    void ControlLoop::onReceive(const int num_bytes)
    {
        // Read the received data into buffer
        _protocol->readBytes(_buffer, num_bytes);
        _current_register = _buffer[0];

        // If only register address was sent (1 byte), just store it for next read
        if (num_bytes == 1)
        {
            return;
        }

        // Process the data (excluding the register address)
        const size_t size = num_bytes - 1;

        switch (_current_register)
        {
        case Registers::MODE.ADDRESS:
            {
                // Update load mode and setpoint
                ModeData mode_data;
                mode_data.fromNetwork(Buffer(_buffer + 1, size));
                ControlLoopRegisters::MODE.setData(mode_data, &_load);
            }
            break;
        default:
            // Handle unknown register
            ESP_LOGE("ControlLoop", "Unknown register: %d", _current_register);
            abort();
        }
    }

}
