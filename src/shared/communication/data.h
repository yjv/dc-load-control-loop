/**
 * @file data.h
 * @brief Data structures and communication utilities
 *
 * This file defines the data structures used for I2C communication between
 * the electronic load and external controllers. It includes register metadata
 * classes, data serialization utilities, and specific data structures for
 * status, mode, and debug information.
 */

#ifndef DATA_H
#define DATA_H

#include <stdint.h>
#include "../value.h"

namespace communication
{
    /**
     * @brief Template base class for read-only register metadata
     * @tparam D Data type that the register handles
     * @tparam A Register address
     *
     * This template class provides metadata for read-only registers,
     * including the register address and data type information.
     */
    template<typename D, uint8_t A>
    class ReadRegisterMetadata
    {

    protected:
        /**
         * @brief Virtual destructor
         *
         * Virtual destructor ensures proper cleanup when deleting derived objects
         * through base class pointers. This is essential for proper memory management
         * in polymorphic hierarchies.
         */
        virtual ~ReadRegisterMetadata() = default;
    public:
        static constexpr uint8_t ADDRESS = A;  // Register address
    };

    /**
     * @brief Template class for read-write register metadata
     * @tparam D Data type that the register handles
     * @tparam A Register address
     *
     * This template class extends ReadRegisterMetadata to add write capability.
     * It provides metadata for registers that support both read and write operations.
     */
    template<typename D, uint8_t A>
    class ReadWriteRegisterMetadata: public ReadRegisterMetadata<D, A>
    {
    };

    /**
     * @brief Buffer utility for data serialization
     *
     * This class provides utilities for reading and writing data to/from
     * a buffer in network byte order (big-endian). It handles different
     * data sizes (8, 16, 32 bits) and provides bounds checking.
     */
    class Buffer
    {
    public:
        /**
         * @brief Constructor
         * @param buffer Pointer to the data buffer
         * @param size Size of the buffer in bytes
         *
         * Initializes the buffer with the provided data and size.
         * The buffer is used for serializing data to/from network format.
         */
        explicit Buffer(uint8_t* buffer, const uint8_t size)
            : _buffer(buffer), _index(0), _size(size)
        {
        }

        /**
         * @brief Read an 8-bit value from the buffer
         * @return 8-bit value read from the buffer
         *
         * Reads a single byte from the buffer and advances the index.
         */
        uint8_t read8bit();

        /**
         * @brief Read a 16-bit value from the buffer
         * @return 16-bit value read from the buffer (big-endian)
         *
         * Reads two bytes from the buffer and combines them into a 16-bit value
         * in big-endian format (most significant byte first).
         */
        uint16_t read16bit();

        /**
         * @brief Read a 32-bit value from the buffer
         * @return 32-bit value read from the buffer (big-endian)
         *
         * Reads four bytes from the buffer and combines them into a 32-bit value
         * in big-endian format (most significant byte first).
         */
        uint32_t read32bit();

        /**
         * @brief Write an 8-bit value to the buffer
         * @param value 8-bit value to write
         *
         * Writes a single byte to the buffer and advances the index.
         */
        void write8bit(uint8_t value);

        /**
         * @brief Write a 16-bit value to the buffer
         * @param value 16-bit value to write (big-endian)
         *
         * Writes a 16-bit value to the buffer in big-endian format
         * (most significant byte first).
         */
        void write16bit(uint16_t value);

        /**
         * @brief Write a 32-bit value to the buffer
         * @param value 32-bit value to write (big-endian)
         *
         * Writes a 32-bit value to the buffer in big-endian format
         * (most significant byte first).
         */
        void write32bit(uint32_t value);

    private:
        /**
         * @brief Increment the buffer index with bounds checking
         * @return Previous index value
         *
         * Increments the buffer index, ensuring it doesn't exceed the buffer size.
         * Returns the previous index value for use in read/write operations.
         */
        uint8_t incrementIndex()
        {
            if (_index >= _size)
            {
                return _size - 1;
            }

            return _index++;
        }

        uint8_t * _buffer;  // Pointer to the data buffer
        uint8_t _index;     // Current position in the buffer
        uint8_t _size;      // Size of the buffer in bytes
    };

    /**
     * @brief Template base class for data structures
     * @tparam S Size of the data structure in bytes
     *
     * This template class provides the base interface for all data structures
     * used in I2C communication. It defines methods for converting data to/from
     * network byte order.
     */
    template<uint8_t S>
    class Data
    {
    protected:
        /**
         * @brief Virtual destructor
         *
         * Virtual destructor ensures proper cleanup when deleting derived objects
         * through base class pointers. This is essential for proper memory management
         * in polymorphic hierarchies.
         */
        ~Data() = default;

    public:
        static constexpr uint8_t SIZE = S;  // Size of the data structure in bytes

        /**
         * @brief Convert data to network byte order
         * @param buffer Buffer to write the data to
         *
         * Pure virtual method that must be implemented by derived classes
         * to convert data to network byte order for transmission.
         */
        virtual void toNetwork(Buffer buffer) = 0;

        /**
         * @brief Convert data from network byte order
         * @param buffer Buffer to read the data from
         *
         * Pure virtual method that must be implemented by derived classes
         * to convert data from network byte order after reception.
         */
        virtual void fromNetwork(Buffer buffer) = 0;
    };

    /**
     * @brief Status data structure
     *
     * This class represents the current status of the electronic load,
     * including voltage, current, temperature, load mode, and fault code.
     * It is used for read-only status reporting over I2C.
     */
    class StatusData final: public Data<12> {
    public:
        /**
         * @brief Default constructor
         *
         * Initializes the status data with default values:
         * - Voltage: 0V
         * - Current: 0A
         * - Load mode: OFF
         * - Temperature: 0°C
         * - Fault code: 0 (no fault)
         */
        StatusData() : voltage(Voltage((uint32_t)0)), current(Current((uint32_t)0)), load_mode(LoadMode::OFF), temperature((uint32_t)0), fault_code((uint8_t)0)
        {
        }

        /**
         * @brief Convert status data to network byte order
         * @param buffer Buffer to write the data to
         *
         * Serializes the status data to network byte order for I2C transmission.
         */
        void toNetwork(Buffer buffer) override;

        /**
         * @brief Convert status data from network byte order
         * @param buffer Buffer to read the data from
         *
         * Deserializes the status data from network byte order after I2C reception.
         */
        void fromNetwork(Buffer buffer) override;

        Voltage voltage;        // Current voltage measurement
        Current current;        // Current current measurement
        LoadMode load_mode;     // Current load mode
        uint16_t temperature;   // Current temperature in degrees Celsius
        uint8_t fault_code;     // Current fault code (0 = no fault)
    };

    /**
     * @brief Status register metadata
     *
     * This class provides metadata for the status register, including
     * the register address (0x01) and data type (StatusData).
     */
    class StatusRegisterMetadata final : public ReadRegisterMetadata<StatusData, 0x01>
    {
    };

    /**
     * @brief Mode data structure
     *
     * This class represents the operating mode and setpoint configuration
     * of the electronic load. It is used for both reading and writing
     * the load's configuration over I2C.
     */
    class ModeData final : public Data<5>
    {
    public:
        /**
         * @brief Default constructor
         *
         * Initializes the mode data with default values:
         * - Load mode: OFF
         * - Setpoint: 0
         */
        ModeData() : load_mode(LoadMode::OFF), set_point((uint32_t)0)
        {
        }

        /**
         * @brief Constructor from Value pointer
         * @param set_point Pointer to Value object containing the setpoint
         * @param load_mode Load mode to set
         *
         * Creates mode data from a Value object and load mode.
         * The setpoint is extracted as a raw code value.
         */
        ModeData(Value* set_point, const LoadMode load_mode)
            : load_mode(load_mode), set_point(set_point->getCode())
        {
        }

        /**
         * @brief Constructor from raw values
         * @param set_point Raw setpoint code value
         * @param load_mode Load mode to set
         *
         * Creates mode data from raw setpoint code and load mode.
         */
        ModeData(const uint32_t set_point, const LoadMode load_mode)
            : load_mode(load_mode), set_point(set_point)
        {
        }

        /**
         * @brief Convert mode data to network byte order
         * @param buffer Buffer to write the data to
         *
         * Serializes the mode data to network byte order for I2C transmission.
         */
        void toNetwork(Buffer buffer) override;

        /**
         * @brief Convert mode data from network byte order
         * @param buffer Buffer to read the data from
         *
         * Deserializes the mode data from network byte order after I2C reception.
         */
        void fromNetwork(Buffer buffer) override;

        LoadMode load_mode;  // Load operating mode
        uint32_t set_point;  // Setpoint as raw code value
    };

    /**
     * @brief Mode register metadata
     *
     * This class provides metadata for the mode register, including
     * the register address (0x02) and data type (ModeData).
     */
    class ModeRegisterMetadata final : public ReadWriteRegisterMetadata<ModeData, 0x02>
    {
    };

    /**
     * @brief Debug data structure
     *
     * This class represents debug information about the electronic load's
     * internal state, including gate control voltage and error statistics.
     * It is used for read-only debug reporting over I2C.
     */
    class DebugData final : public Data<20>
    {
    public:
        /**
         * @brief Default constructor
         *
         * Initializes the debug data with default values:
         * - Gate control voltage: 0V
         * - All counts: 0
         */
        DebugData(): gate_control_voltage(GateControlVoltage((uint32_t)0)), invalid_voltage_reading_count(0),
                     invalid_current_reading_count(0), reading_count(0), update_gate_voltage_count(0)
        {
        }

        /**
         * @brief Convert debug data to network byte order
         * @param buffer Buffer to write the data to
         *
         * Serializes the debug data to network byte order for I2C transmission.
         */
        void toNetwork(Buffer buffer) override;

        /**
         * @brief Convert debug data from network byte order
         * @param buffer Buffer to read the data from
         *
         * Deserializes the debug data from network byte order after I2C reception.
         */
        void fromNetwork(Buffer buffer) override;

        GateControlVoltage gate_control_voltage;        // Current gate control voltage
        uint32_t update_gate_voltage_count;             // Number of gate voltage updates
        uint32_t reading_count;                         // Total number of readings taken
        uint32_t invalid_voltage_reading_count;         // Number of invalid voltage readings
        uint32_t invalid_current_reading_count;         // Number of invalid current readings
    };

    /**
     * @brief Debug register metadata
     *
     * This class provides metadata for the debug register, including
     * the register address (0x03) and data type (DebugData).
     */
    class DebugRegisterMetadata final : public ReadRegisterMetadata<DebugData, 0x03>
    {
    };

    /**
     * @brief Register metadata container
     *
     * This class provides static instances of all available register metadata
     * for the communication system. It serves as a central registry for
     * accessing different types of load data.
     */
    class Registers {
    public:
        static const StatusRegisterMetadata STATUS;  // Status register metadata
        static const ModeRegisterMetadata MODE;      // Mode register metadata
        static const DebugRegisterMetadata DEBUG;    // Debug register metadata
    };

}

#endif //DATA_H
