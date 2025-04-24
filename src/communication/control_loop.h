//
// Created by Joseph Deray on 4/10/25.
//

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <load.h>
#include <Wire.h>

template<typename D>
class ReadRegister;

template<typename D>
class ReadWriteRegister;

class ControlLoop {
public:
    explicit ControlLoop(TwoWire* protocol, State& state): _protocol(protocol), _state(state) {}
    void onRequest();
    void onReceive(int num_bytes);

private:
    TwoWire * _protocol;
    State& _state;
    uint8_t _control_loop_address{};
    uint8_t _current_register{};
    static constexpr uint8_t BUFFER_SIZE = 32;
    uint8_t _buffer[BUFFER_SIZE];
};

#endif //CONTROL_LOOP_H
