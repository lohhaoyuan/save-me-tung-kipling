#include <Arduino.h>
#include "stm32f103c_variant_generic.h"
#include "main.h"
void setup() {
    TeensySerial.begin(115200);

    analogReadResolution(12);
    for (auto mux : muxs) mux.init();
}

void loop(){
    printLightRingCalibration();
}