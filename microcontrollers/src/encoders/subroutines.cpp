#include "main.h"

int16_t speeds[4];

void TeensyPacketHandler(const uint8_t *buf, size_t size) {
        if (size != sizeof(EncoderTxPayload)) {
               return;
        }
        
        EncoderTxPayloadUnion packet;
        memcpy(packet.bytes, buf, size);

        speeds[0] = packet.data.motorSpeed[0];
        speeds[1] = packet.data.motorSpeed[1];
        speeds[2] = packet.data.motorSpeed[2];
        speeds[3] = packet.data.motorSpeed[3];
}

void drive(int16_t FL_SPEED,
            int16_t FR_SPEED,
            int16_t BL_SPEED,
            int16_t BR_SPEED) {

    // Constrain motor speed with "hardware-imposed" limits
    auto constrainSpeed = [](int16_t speed) {
        // If the speed is below the stall speed, don't bother moving
        if (abs(speed) < DRIVE_STALL_SPEED) return 0;
        return min(abs(speed), DRIVE_MAX_SPEED);
    };
        // Set the motor directions and speeds
    digitalWrite(FL_DIR,
            FL_SPEED > 0 ? MOTOR_FL_REVERSED : !MOTOR_FL_REVERSED);
    digitalWrite(FR_DIR,
            FR_SPEED > 0 ? MOTOR_FR_REVERSED : !MOTOR_FR_REVERSED);
    digitalWrite(BL_DIR,
            BL_SPEED > 0 ? MOTOR_BL_REVERSED : !MOTOR_BL_REVERSED);
    digitalWrite(BR_DIR,
            BR_SPEED > 0 ? MOTOR_BR_REVERSED : !MOTOR_BR_REVERSED);
    analogWrite(FL_PWM,
    constrainSpeed(FL_SPEED) * MOTOR_FL_MULTIPLIER);
    analogWrite(FR_PWM,
    constrainSpeed(FR_SPEED) * MOTOR_FR_MULTIPLIER);
    analogWrite(BL_PWM,
    constrainSpeed(BL_SPEED) * MOTOR_BL_MULTIPLIER);
    analogWrite(BR_PWM,
    constrainSpeed(BR_SPEED) * MOTOR_BR_MULTIPLIER);

}