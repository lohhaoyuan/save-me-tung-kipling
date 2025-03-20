#include "main.h"

#ifdef BT_SERVER
BLEServer *server = nullptr;
BLECharacteristic *rxCharacteristic = nullptr;

// Callback to handle received data
class RXCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override {
        std::string receivedData = characteristic->getValue();
        Serial.print("Received from Client: ");
        Serial.println(receivedData.c_str());  // Print received message
    }
};

void setup() {
    Serial.begin(115200);
    BLEDevice::init("ESP32C3_Server");

    server = BLEDevice::createServer();

    BLEService *service = server->createService(SERVICE_UUID);

    // RX Characteristic - writable (Client → Server)
    rxCharacteristic = service->createCharacteristic(
        RX_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    rxCharacteristic->setCallbacks(new RXCallbacks());

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();

    Serial.println("BLE Server Ready, Waiting for Client...");
}
void loop(){
    delay(1000);
}
#else // BT_CLIENT
BLEClient *client;
BLEAdvertisedDevice *serverDevice;
BLERemoteCharacteristic *rxCharacteristic;
bool deviceFound = false;

// Callback to detect the BLE Server
class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
            Serial.println("Found BLE Server!");
            serverDevice = new BLEAdvertisedDevice(advertisedDevice);
            deviceFound = true;
        }
    }
};

void setup() {
    Serial.begin(115200);
    BLEDevice::init("ESP32C3_Client");

    BLEScan *scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
    scan->setActiveScan(true);
    scan->start(5, false);

    if (!deviceFound) {
        Serial.println("No server found. Retrying...");
        return;
    }

    client = BLEDevice::createClient();
    client->connect(serverDevice);
    Serial.println("Connected to BLE Server!");

    BLERemoteService *service = client->getService(BLEUUID(SERVICE_UUID));
    if (service) {
        rxCharacteristic = service->getCharacteristic(BLEUUID(RX_CHARACTERISTIC_UUID));

        if (rxCharacteristic && rxCharacteristic->canWrite()) {
            Serial.println("Sending data to Server...");
            rxCharacteristic->writeValue("Hello from Client!");
        }
    }
}

void loop() {
    delay(5000);
    if (rxCharacteristic && rxCharacteristic->canWrite()) {
        Serial.println("Sending message...");
        rxCharacteristic->writeValue("Ping from Client!");
    }
}
#endif
