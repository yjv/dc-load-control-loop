#include <stdint.h>

class Registers {
    public:
        static constexpr uint8_t READ = 0x40;
        static constexpr uint8_t WRITE = 0x0;
        
            // Register Address Constants
        static constexpr uint8_t COMMS = 0x00;
        static constexpr uint8_t STATUS = 0x00; // Same as COMMS
        static constexpr uint8_t ADCMODE = 0x01;
        static constexpr uint8_t IFMODE = 0x02;
        static constexpr uint8_t REGCHECK = 0x03;
        static constexpr uint8_t DATA = 0x04;
        static constexpr uint8_t GPIOCON = 0x06;
        static constexpr uint8_t ID = 0x07;
        static constexpr uint8_t CH0 = 0x10;
        static constexpr uint8_t CH1 = 0x11;
        static constexpr uint8_t CH2 = 0x12;
        static constexpr uint8_t CH3 = 0x13;
        static constexpr uint8_t SETUPCON0 = 0x20;
        static constexpr uint8_t SETUPCON1 = 0x21;
        static constexpr uint8_t SETUPCON2 = 0x22;
        static constexpr uint8_t SETUPCON3 = 0x23;
        static constexpr uint8_t FILTCON0 = 0x28;
        static constexpr uint8_t FILTCON1 = 0x29;
        static constexpr uint8_t FILTCON2 = 0x2A;
        static constexpr uint8_t FILTCON3 = 0x2B;
        static constexpr uint8_t OFFSET0 = 0x30;
        static constexpr uint8_t OFFSET1 = 0x31;
        static constexpr uint8_t OFFSET2 = 0x32;
        static constexpr uint8_t OFFSET3 = 0x33;
        static constexpr uint8_t GAIN0 = 0x38;
        static constexpr uint8_t GAIN1 = 0x39;
        static constexpr uint8_t GAIN2 = 0x3A;
        static constexpr uint8_t GAIN3 = 0x3B;
};
