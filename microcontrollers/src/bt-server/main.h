#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "cobs.h"
#include "vector.h"


#define BT_SERVER


#define SERVICE_UUID "ca438d8d-b932-46e8-84e9-39737270ba9b"
#define RX_CHARACTERISTIC_UUID "52cd094a-605b-4cc1-b45f-473a5e8c41c9" // Client → Server
#define TX_CHARACTERISTIC_UUID "e2ad2562-d291-49a8-acfd-81f7179746cd"  // Server → Client

struct ScoutToGoalieBTPacket {
    Vector ball;
    bool ScoutActive;
};


#endif