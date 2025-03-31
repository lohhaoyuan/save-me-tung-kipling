#include "enc.h"
#include "main.h"
#include "stm32f4.h"

// put function declarations here:
//PWM and DIR pin assignments
#define M1_PWM PA6
#define M1_DIR PA7

#define M2_PWM PB6
#define M2_DIR PB5

#define M3_PWM PB8
#define M3_DIR PB9

#define M4_PWM PA5
#define M4_DIR PA4


// Encoder pin assignments
const uint8_t encoderPins[4][2] = {
    {PB13, PB12},   // Encoder 1
    {PA9, PA8}, // Encoder 2
    {PA10, PA11}, // Encoder 3
    {PB1, PB0}  // Encoder 4
};

// Encoder objects
Encoder encoders[4] = {
    Encoder(encoderPins[0][0], encoderPins[0][1]),
    Encoder(encoderPins[1][0], encoderPins[1][1]),
    Encoder(encoderPins[2][0], encoderPins[2][1]),
    Encoder(encoderPins[3][0], encoderPins[3][1])
};

// Variables for tracking encoder positions and time
volatile long lastPosition[4] = {0};
volatile long lastTime[4] = {0};
volatile float rpm[4] = {0};

// Interrupt Service Routine (ISR) for all encoders
void encoderISR(uint8_t index) {
  lastPosition[index] = encoders[index].read();
  lastTime[index] = micros(); // Use micros() instead of millis() for better accuracy
}
void updateRPM() {
  static long prevPosition[4] = {0}; 
  static long prevTime[4] = {0};

  for (uint8_t i = 0; i < 4; i++) {
      long currentPosition = lastPosition[i];
      long currentTime = micros();

      long positionChange = currentPosition - prevPosition[i];
      long timeChange = currentTime - prevTime[i];

      if (timeChange > 0) {
          rpm[i] = (positionChange / (float)ENCODER_PPR) * (60000000.0 / timeChange);
      }

      prevPosition[i] = currentPosition;
      prevTime[i] = currentTime;

  }
  TeensySerial.print("RPM: ");
      for (uint8_t i = 0; i < 4; i++) {
        TeensySerial.print(rpm[i], 2);  // Print with 2 decimal places
        TeensySerial.print(i < 4 - 1 ? ", " : "\n");
      }
}
// Wrapper functions for each encoder interrupt
void encoder1ISR() { encoderISR(0); }
void encoder2ISR() { encoderISR(1); }
void encoder3ISR() { encoderISR(2); }
void encoder4ISR() { encoderISR(3); }

// Function to get RPM of each encoder
float getRPM(uint8_t index) {
  return rpm[index];
}
