#include "mux.h"

#include <Arduino.h>

// Creates an object to interface with a 16-channel multiplexer
Mux::Mux(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t signal)
    : _s0(s0), _s1(s1), _s2(s2), _s3(s3), _signal(signal) {}

// Initialises the pins connected to the multiplexer
void Mux::init() const {
    pinMode(_s0, OUTPUT);
    pinMode(_s1, OUTPUT);
    pinMode(_s2, OUTPUT);
    pinMode(_s3, OUTPUT);
    pinMode(_signal, INPUT);
}

// Selects the channel on the multiplexer
void Mux::selectChannel(uint8_t channel) const {
    digitalWrite(_s0, channel & 0b0001);
    digitalWrite(_s1, channel & 0b0010);
    digitalWrite(_s2, channel & 0b0100);
    digitalWrite(_s3, channel & 0b1000);

}

// Selects the channel on the multiplexer and reads the value from the channel,
// with a delay between channel selection and the read in microseconds if
// provided
uint16_t Mux::readChannel(uint16_t channel, uint32_t delay) const {
    selectChannel(channel);
    if (delay > 0) delayMicroseconds(delay);
    return analogRead(_signal);
}