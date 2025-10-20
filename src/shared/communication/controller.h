/**
 * @file controller.h
 * @brief I2C controller for communication with the control loop
 *
 * This file defines the Controller class, which provides an I2C controller interface
 * for communicating with the electronic load over I2C. It handles reading and
 * writing data to different registers on the load device.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "data.h"
#include <Wire.h>
#include <Arduino.h>

namespace communication
{
    /**
     * @brief I2C controller for communication with the control loop
     *
     * This class provides an I2C controller interface for communicating with the
     * electronic load over I2C. It handles reading and writing data to different
     * registers on the load device, with automatic retry logic for read operations.
     */
    class Controller {
    public:
        /**
         * @brief Constructor
         * @param protocol Pointer to TwoWire (I2C) protocol instance
         * @param control_loop_address I2C address of the control loop device
         *
         * Initializes the controller with the I2C protocol and target device address.
         * The controller acts as an I2C controller device that communicates with the load.
         */
        explicit Controller(TwoWire* protocol, uint8_t control_loop_address): _control_loop_address(control_loop_address), _protocol(protocol)
        {
        }

        /**
         * @brief Read data from a register
         * @tparam D Data type to read
         * @tparam A Register address
         * @param read_register Pointer to register metadata
         * @return Data read from the register
         *
         * Reads data from the specified register on the control loop device.
         * The method includes retry logic to handle communication failures.
         * It will retry until the expected number of bytes is received.
         */
        template<typename D, uint8_t A>
        D read(const ReadRegisterMetadata<D, A> * read_register);

        /**
         * @brief Write data to a register
         * @tparam D Data type to write
         * @tparam A Register address
         * @param read_write_register Pointer to register metadata
         * @param data Data to write
         *
         * Writes data to the specified register on the control loop device.
         * The data is automatically converted to network byte order.
         */
        template<typename D, uint8_t A>
        void write(const ReadWriteRegisterMetadata<D, A> * read_write_register, D data);

    private:
        uint8_t _control_loop_address;        // I2C address of the control loop device
        static constexpr uint8_t BUFFER_SIZE = 32;  // I2C communication buffer size
        uint8_t _buffer[BUFFER_SIZE];         // I2C communication buffer
        TwoWire * _protocol;                  // I2C protocol instance
    };

    /**
     * @brief Read data from a register
     * @tparam D Data type to read
     * @tparam A Register address
     * @param read_register Pointer to register metadata
     * @return Data read from the register
     *
     * Reads data from the specified register on the control loop device.
     * The method includes retry logic to handle communication failures.
     * It will retry until the expected number of bytes is received.
     */
    template<typename D, uint8_t A>
    D Controller::read(const ReadRegisterMetadata<D, A> * read_register)
    {
        uint8_t num_bytes = 0;

        // Retry loop to handle communication failures
        do
        {
            // Send register address to device
            _protocol->beginTransmission(_control_loop_address);
            _protocol->write(read_register->ADDRESS);
            _protocol->endTransmission();
            
            // Request data from device
            num_bytes = _protocol->requestFrom(_control_loop_address, D::SIZE);

            // Small delay between retries
            delay(1);
        }
        while (num_bytes < D::SIZE);

        // Read the data into buffer
        _protocol->readBytes((uint8_t *)_buffer, D::SIZE);

        // Convert from network byte order and return
        D data = D();
        data.fromNetwork(Buffer(_buffer, D::SIZE));
        return data;
    }

    /**
     * @brief Write data to a register
     * @tparam D Data type to write
     * @tparam A Register address
     * @param read_write_register Pointer to register metadata
     * @param data Data to write
     *
     * Writes data to the specified register on the control loop device.
     * The data is automatically converted to network byte order before transmission.
     */
    template<typename D, uint8_t A>
    void Controller::write(const ReadWriteRegisterMetadata<D, A> * read_write_register, D data)
    {
        // Convert data to network byte order
        data.toNetwork(Buffer(_buffer, D::SIZE));
        
        // Send data to device
        _protocol->beginTransmission(_control_loop_address);
        _protocol->write(read_write_register->ADDRESS);
        _protocol->write((uint8_t *)_buffer, D::SIZE);
        _protocol->endTransmission();
    }

}

#endif //CONTROLLER_H
