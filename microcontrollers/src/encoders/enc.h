#ifndef ENC_H
#define ENC_H

#include <Arduino.h>
#include <Encoder.h>

// Encoder definitions
#define ENCODER_PPR 13  // Pulses Per Revolution (PPR)
#define TIMER_INTERVAL 100 // RPM update interval (milliseconds)


extern const uint8_t encoderPins[4][2];
extern Encoder encoders[4];
void encoderISR(uint8_t index);
void encoder1ISR();
void encoder2ISR();
void encoder3ISR();
void encoder4ISR();
float getRPM(uint8_t index);
void updateRPM();


#endif