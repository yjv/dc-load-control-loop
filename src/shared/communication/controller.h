//
// Created by Joseph Deray on 4/10/25.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "data.h"
#include <Wire.h>
#include <Arduino.h>

class Controller {
public:
    explicit Controller(TwoWire* protocol, uint8_t controller_address, uint8_t control_loop_address): _controller_address(controller_address), _control_loop_address(control_loop_address), _protocol(protocol)
    {
    }

    template<typename D, uint8_t A>
    D read(const ReadRegisterMetadata<D, A> * read_register);

    template<typename D, uint8_t A>
    void write(const ReadWriteRegisterMetadata<D, A> * read_write_register, D data);
private:
    uint8_t _controller_address;
    uint8_t _control_loop_address;
    static constexpr uint8_t BUFFER_SIZE = 32;
    uint8_t _buffer[BUFFER_SIZE];
    TwoWire * _protocol;
};

template<typename D, uint8_t A>
D Controller::read(const ReadRegisterMetadata<D, A> * read_register)
{
    uint8_t num_bytes = 0;

    do
    {
        _protocol->beginTransmission(_control_loop_address);
        _protocol->write(read_register->ADDRESS);
        _protocol->endTransmission();
        num_bytes = _protocol->requestFrom(_control_loop_address, D::SIZE);

        delay(1);
    }
    while (num_bytes < D::SIZE);

    _protocol->readBytes((uint8_t *)_buffer, D::SIZE);

    D data = D();
    data.fromNetwork(Buffer(_buffer, D::SIZE));
    return data;
}

template<typename D, uint8_t A>
void Controller::write(const ReadWriteRegisterMetadata<D, A> * read_write_register, D data)
{
    data.toNetwork(Buffer(_buffer, D::SIZE));
    _protocol->beginTransmission(_control_loop_address);
    _protocol->write(read_write_register->ADDRESS);
    _protocol->write((uint8_t *)_buffer, D::SIZE);
    _protocol->endTransmission();
}

#endif //CONTROLLER_H
