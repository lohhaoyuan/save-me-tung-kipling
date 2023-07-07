#include "main.h"
#include "misc.cpp"
// #include "calibrate.cpp"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29); // For purple

float deg2rad(float angle) { return angle * PI / 180; }

struct Movement {
    int16_t angle = 0; // -179(.)99º to +180(.)00º
    int16_t speed = 40;
    int16_t angularVelocity = 0; // ~±140 to ~±1024
} movement;

struct balldata {
    int angle = 0;
    int strength = 0;
} balldata;

struct linedata {
    int moveAngle = 0;
    bool onLine = false;
} linedata;

// IMU Stuff

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData) {
    DEBUG.print("Accelerometer: ");
    DEBUG.print(calibData.accel_offset_x);
    DEBUG.print(" ");
    DEBUG.print(calibData.accel_offset_y);
    DEBUG.print(" ");
    DEBUG.print(calibData.accel_offset_z);
    DEBUG.print(" ");

    DEBUG.print("\nGyro: ");
    DEBUG.print(calibData.gyro_offset_x);
    DEBUG.print(" ");
    DEBUG.print(calibData.gyro_offset_y);
    DEBUG.print(" ");
    DEBUG.print(calibData.gyro_offset_z);
    DEBUG.print(" ");

    DEBUG.print("\nMag: ");
    DEBUG.print(calibData.mag_offset_x);
    DEBUG.print(" ");
    DEBUG.print(calibData.mag_offset_y);
    DEBUG.print(" ");
    DEBUG.print(calibData.mag_offset_z);
    DEBUG.print(" ");

    DEBUG.print("\nAccel Radius: ");
    DEBUG.print(calibData.accel_radius);

    DEBUG.print("\nMag Radius: ");
    DEBUG.print(calibData.mag_radius);
    delay(BNO055_SAMPLERATE_DELAY_MS);
}

void displayCalStatus(void) {
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system) { Serial.print("! "); }

    /* Display the individual values */
    DEBUG.print("Sys:");
    DEBUG.print(system, DEC);
    DEBUG.print(" G:");
    DEBUG.print(gyro, DEC);
    DEBUG.print(" A:");
    DEBUG.print(accel, DEC);
    DEBUG.print(" M:");
    DEBUG.println(mag, DEC);
}

void calculateRobotAngle() {
    sensors_event_t eulerAngles;
    bno.getEvent(&eulerAngles, Adafruit_BNO055::VECTOR_EULER);
    int robotBearing = round(eulerAngles.orientation.x);
    int epoopoo = robotBearing <= 180 ? robotBearing : robotBearing - 360;
    int error = -epoopoo;
    correctionKP = error * IMUKP;
    correctionKI = (error + correctionKI) * IMUKI;
    correctionKD = (error - lastError) * IMUKD;
    correction = correctionKP + correctionKI + correctionKD;
    lastError = error;
    // DEBUG.println(robotBearing);
}

void drive() {
    // Convert polar to cartesian
    const auto x = sin(movement.angle * DEG_TO_RAD);
    const auto y = cos(movement.angle * DEG_TO_RAD);

    // Compute the speeds of the individual motors
    const auto transformSpeed = [](float speed, float angularComponent) {
        return (int16_t)roundf(speed * movement.speed + angularComponent);
    };
    // Compute speeds
    const int16_t FLSpeed = transformSpeed(
        x * COS45 + y * SIN45,
        movement.angularVelocity); // Positive Angular is clockwise
    const int16_t FRSpeed =
        transformSpeed(x * -COS45 + y * SIN45, -movement.angularVelocity);
    const int16_t BLSpeed =
        transformSpeed(x * -COS45 + y * SIN45, movement.angularVelocity);
    const int16_t BRSpeed =
        transformSpeed(x * COS45 + y * SIN45, -movement.angularVelocity);

    // Constrain motor speed
    // auto constrainSpeed = [](int16_t speed)
    // {
    //   // If the speed is below the stall speed, don't bother moving
    //   if (abs(speed) < DRIVE_STALL_SPEED)
    //     return 0;
    //   return min(abs(speed), DRIVE_MAX_SPEED);
    // };

    // Set the motor directions and speeds
    digitalWriteFast(FL_DIR, FLSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(FR_DIR, FRSpeed > 0 ? HIGH : LOW);
    digitalWriteFast(BL_DIR, BLSpeed > 0 ? LOW : HIGH);
    digitalWriteFast(BR_DIR, BRSpeed > 0 ? HIGH : LOW);

    analogWrite(FL_PWM,
                constrain(abs(FLSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
    analogWrite(FR_PWM,
                constrain(abs(FRSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
    analogWrite(BL_PWM,
                constrain(abs(BLSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
    analogWrite(BR_PWM,
                constrain(abs(BRSpeed), DRIVE_STALL_SPEED, DRIVE_MAX_SPEED));
}

void getIRData() {
    // Loop through the IR_SERIAL buffer to find the sync byte
    while (IR_SERIAL.available() >= 9U) {
        if (IR_SERIAL.read() == SYNC_BYTE) {
            // Subtract the buffer by -1 to remove the SYNC_BYTE before reading
            // the rest of the buffer
            byte buf[8U];
            IR_SERIAL.readBytes(buf, 8U);

            // Copy the last 8 buffer bytes into the ballAngle and ballStrength
            // variables
            memcpy(&balldata.angle, buf, 4U);
            memcpy(&balldata.strength, buf + 4U, 4U);

            //     // // Print the buffer to serial with printf
            //     // for (int i = 0; i < 8; ++i)
            //     //   DEBUG.printf("%02x", buf[i]);
            //     // DEBUG.print("\n ");
            //   }
            // }
        }
    }
    // while (IR_SERIAL.available() > 0)
    // {
    //   DEBUG.print(char(IR_SERIAL.read()));
    // }
}

void getLightData() {
    // Loop through the IR_SERIAL buffer to find the sync byte
    while (LAYER1_SERIAL.available() >= 7U) {
        if (LAYER1_SERIAL.read() == SYNC_BYTE) {
            // Subtract the buffer by -1 to remove the SYNC_BYTE before reading
            // the rest of the buffer
            byte buf[6U];
            LAYER1_SERIAL.readBytes(buf, 6U);

            // Copy the last 6 buffer bytes into the onLine status which is a
            // bool and moveAngle which is an int
            memcpy(&linedata.onLine, buf, 1U);
            memcpy(&linedata.moveAngle, buf + 1U, 4U);
        }
    }
    // while (LAYER1_SERIAL.available() > 0)
    // {
    //   DEBUG.print(char(LAYER1_SERIAL.read()));
    // }
}

void getCameraData() {
    // while (CAMERA_SERIAL.available() >= 25)
    // {
    //   if (LAYER1_SERIAL.read() == SYNC_BYTE)
    //   {
    //     // Subtract the buffer by -1 to remove the SYNC_BYTE before reading
    //     the rest of the buffer byte buf[24U]; LAYER1_SERIAL.readBytes(buf,
    //     6U);

    //     // Copy the last 6 buffer bytes into the onLine status which is a
    //     bool and moveAngle which is an int memcpy(&linedata.onLine, buf, 1U);
    //     memcpy(&linedata.moveAngle, buf + 1U, 4U);
    //   }
    // }
    // while (CAMERA_SERIAL.available() > 0)
    // {
    //   DEBUG.print(char(CAMERA_SERIAL.read()));
    // }
}

void stop() {
    analogWrite(FL_PWM, 0);
    analogWrite(FR_PWM, 0);
    analogWrite(BL_PWM, 0);
    analogWrite(BR_PWM, 0);
}

void calculateOrbit() {
    float ballOffset, ballMult;
    balldata.angle <= 180 ? ballOffset = fmin(balldata.angle * 1.6, 90)
                          : ballOffset = max((balldata.angle - 360) * 1.6, -90);
    // float factor = 1 - ((balldata.strength) / 20) / 27;
    ballMult = 1;
    robotAngle = balldata.angle + ballOffset * ballMult;
}

void setup() {
    // Serial Defintions
    DEBUG.begin(115200);
    IR_SERIAL.begin(115200);
    CAMERA_SERIAL.begin(115200);
    LAYER1_SERIAL.begin(115200);

    Wire.begin();
    bno.begin();
    delay(1000);
    // bno.setMode(OPERATION_MODE_IMUPLUS);

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    // Get BNO calibration data from EEPROM
    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id) {
        DEBUG.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    } else {
        DEBUG.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        DEBUG.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        DEBUG.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    bno.setExtCrystalUse(false);

    // Motor Pins
    pinMode(FL_PWM, OUTPUT);
    pinMode(FL_DIR, OUTPUT);
    pinMode(FR_PWM, OUTPUT);
    pinMode(FR_DIR, OUTPUT);
    pinMode(BR_PWM, OUTPUT);
    pinMode(BR_DIR, OUTPUT);
    pinMode(BL_PWM, OUTPUT);
    pinMode(BL_DIR, OUTPUT);

    // Built in LED
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);
}

void loop() {

    // if (linedata.onLine)
    // {
    //   movement.angle = linedata.moveAngle;
    //   movement.speed = 50;
    //   movement.angularVelocity = 0;
    //   drive();
    // }
    getIRData();
    if (balldata.strength != 400) {
        // move to ball
        calculateOrbit();
        movement.angularVelocity = 0;
        movement.angle = robotAngle;
        movement.speed = 50;
        drive();
    }
    // else
    // {
    //   // localise
    // }
    //     // if (analogRead(BALL_DETECT) > 400)
    //     // {
    //     //   movement.angle = 0;
    //     //   movement.speed = 50;
    //     //   movement.angularVelocity = 0;
    //     //   drive();
    //     // }
    //   }
    // else
    // {
    //   movement.angle = 0;
    //   movement.speed = 0;
    //   movement.angularVelocity = 0;
    //   drive();
    // }
}

// }

// DEBUG.println(balldata.strength);
// DEBUG.print(linedata.onLine);
// DEBUG.print(" ");
// if (linedata.onLine)
// {
//   movement.angle = linedata.moveAngle;
//   movement.speed = 40;
//   movement.angularVelocity = 0;
//   drive();
// }
// else
// {
// }

// drive();
// getLightData();
// else
// {
//   getIRData();
//   if (balldata.strength != 400)
//   {
//     // move to ball
//     calculateOrbit();
//     drive();
//   }
// }
// getIRData();
// if (balldata.strength != 400)
// {
//   // move to ball
//   calculateOrbit();
//   drive();
// }

// read all sensors
// getLightData();
// check inbounds
// if light.In false
// moveout
// getIRData();
// // check if orientation is correct
// calculateRobotAngle();
// if (abs(correction) > 5)
// {
//   // keep looping until robot is facing forwards
//   while (abs(correction) > 5)
//   {
//     calculateRobotAngle();
//     faceForwards();
//   }
// }
// else
// {
//   // check if ball exist
//   if (balldata.strength != 400)
//   {
//     // move to ball
//     calculateOrbit();
//     drive();
//   }
// if (balldata.strength != 400)
// {
//   // move to ball
//   calculateOrbit();
//   drive();
// }
// else
// {
//   // center to field via camera
//   // calculateRobotAngle();
//   // faceForwards();
//   // chaseGoal(); // while ball still caputred chase goal else search ball
//   stop();
// }
// movement.angle = balldata.angle;
// movement.speed = 50;
// drive();
// check if ball exist ? move to ball : center to field via camera
// if ball.stregnth != 400
// movement.angle = ball.angle
// if ball caught analogread
// keep straight
// calculateRobotAngle();
// faceForwards();
// chaseGoal(); // while ball still caputred chase goal else search ball

// DEBUG.print("Ball Angle: ");
// DEBUG.print(movement.angle);
// DEBUG.print(" | ");
// DEBUG.print("Ball Strength: ");
// DEBUG.println(balldata.strength);
// getLightData();
//  printAllIMUData();
// readRobotAngle();