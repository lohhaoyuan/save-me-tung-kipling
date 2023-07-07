// #include "main.h"

// void printAllIMUData()
// {
//     sensors_event_t eul, gyr, lac, mag, acc, gra;
//     bno.getEvent(&eul, Adafruit_BNO055::VECTOR_EULER);
//     bno.getEvent(&gyr, Adafruit_BNO055::VECTOR_GYROSCOPE);
//     bno.getEvent(&acc, Adafruit_BNO055::VECTOR_ACCELEROMETER);
//     bno.getEvent(&lac, Adafruit_BNO055::VECTOR_LINEARACCEL);
//     bno.getEvent(&gra, Adafruit_BNO055::VECTOR_GRAVITY);
//     bno.getEvent(&mag, Adafruit_BNO055::VECTOR_MAGNETOMETER);

//     // Get calibration states
//     uint8_t systemCalib, gyroCalib, accCalib, magCalib;
//     bno.getCalibration(&systemCalib, &gyroCalib, &accCalib, &magCalib);

//     // Print everything to serial
//     const auto printVector = [](const char *name, const sensors_vec_t
//     &vector)
//     {
//         DEBUG.printf(
//             "%s: x = %4d.%02d y = %4d.%02d z = %4d.%02d\n", name,
//             (int16_t)vector.x, abs((int32_t)(vector.x * 100) % 100),
//             (int16_t)vector.y, abs((int32_t)(vector.y * 100) % 100),
//             (int16_t)vector.z, abs((int32_t)(vector.z * 100) % 100));
//     };
//     printVector("Euler Angle (º)            ", eul.orientation);
//     printVector("Angular Velocity (rad s⁻¹) ", gyr.gyro);
//     printVector("Acceleration (m s⁻²)       ", acc.acceleration);
//     printVector("Linear Acceleration (m s⁻²)", lac.acceleration);
//     printVector("Gravity (m s⁻²)            ", gra.acceleration);
//     printVector("Magnetic Field (μT)        ", mag.magnetic);

//     DEBUG.printf(
//         "Calibration: System = %d Gyroscope = %d Accelerometer = %d "
//         "Magnetometer = %d\n\n",
//         systemCalib, gyroCalib, accCalib, magCalib);
// }