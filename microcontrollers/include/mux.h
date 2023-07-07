#ifndef MUX_H
#define MUX_H

#include <Arduino.h>

class Mux {
  public:
    Mux(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t signal);
    void init() const;
    void selectChannel(uint8_t channel) const;
    uint16_t readChannel(uint16_t channel, uint32_t delay = 0) const;

  private:
    uint8_t _s0;
    uint8_t _s1;
    uint8_t _s2;
    uint8_t _s3;
    uint8_t _signal;
};

#endif // MUX_H
