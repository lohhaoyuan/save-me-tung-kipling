#include "main.h"
#include "movement.h"

extern Movement seggs;
void LightPacketHandler(const uint8_t *buf, size_t size) {
    if (size != sizeof(LightTxPayloadUnion)) return;
    LightTxPayloadUnion payload;
    memcpy(payload.bytes, buf, sizeof(LightTxPayloadUnion));
    sensors.line = payload.data.line;
    sensors.frontLG = payload.data.frontLG;
    sensors.backLG = payload.data.backLG;
} // Ensure proper termination of the function

void TopPacketHandler(const uint8_t *buf, size_t size){
    if (size != sizeof(SesbianLexPayloadUnion)) return;
    SesbianLexPayloadUnion payload;
    memcpy(payload.bytes, buf, sizeof(SesbianLexPayloadUnion));
    sensors.ball = payload.data.ball;
    sensors.blue = payload.data.blue;
    sensors.yellow = payload.data.yellow;
    sensors.lidarDist[0] = payload.data.lidarDist[0];
    sensors.lidarDist[1] = payload.data.lidarDist[1];
    sensors.lidarDist[2] = payload.data.lidarDist[2];
    sensors.lidarDist[3] = payload.data.lidarDist[3];
    sensors.yaw = payload.data.yaw;
}

void avoidLine(){
    if (sensors.line.exists()) {
        if (sensors.line.depth > WALL_AVOIDANCE_THRESHOLD) {
            // We're too far into the line, move away quickly
            seggs.setDirection(
                (Direction::Constant){sensors.line.angleBisector});
            seggs.setVelocity((Velocity::Constant){
                sensors.line.depth * WALL_AVOIDANCE_SPEED_MULTIPLIER});
        }
    }  
}
