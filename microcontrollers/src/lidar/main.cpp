#include "main.h"

HardwareSerial MySerial0(0);
PacketSerial L3LIDARSerial;

TFLI2C front, right, back, left;
TFLI2C tflI2C[4] = {front, right, back, left};

int16_t tfDist[4];
int tfAddress[4] = {0x35, 0x22, 0x33, 0x44}; // front, right, back, left

lidardata esp32lidardata;

void setup() {
    Serial.begin(115200);
    MySerial0.begin(115200, SERIAL_8N1, -1, -1);
    L3LIDARSerial.begin(&MySerial0);
    Wire.begin();

    setupLidar();
}

void loop() {
    readLidar();
    sendLidarData();
    delay(20);
}

// Initialize LIDAR settings
void setupLidar() {
    for (int i = 0; i < 4; i++) { tflI2C[i].Save_Settings(tfAddress[i]); }
    // Uncomment if you need to reset I2C address
    // tflI2C[3].Set_I2C_Addr(0x44, 0x22);
    // tflI2C[3].Soft_Reset(0x44);
}

// Read LIDAR sensor data
void readLidar() {
    for (int i = 0; i < 4; i++) {
        if (tflI2C[i].getData(tfDist[i], tfAddress[i])) {
#ifdef DEBUGTOF
            Serial.print(" Dist: ");
            Serial.print(tfDist[i]);
            if (i == 3) Serial.println();
#endif
            esp32lidardata.distance[i] = tfDist[i];
        } else {
            esp32lidardata.distance[i] = 0;
            tflI2C[i].printStatus();
        }
    }
}

// Send LIDAR data over serial
void sendLidarData() {
    byte buf[sizeof(lidarTxPayload)];
    memcpy(buf, &esp32lidardata, sizeof(esp32lidardata));
    L3LIDARSerial.send(buf, sizeof(buf));
}