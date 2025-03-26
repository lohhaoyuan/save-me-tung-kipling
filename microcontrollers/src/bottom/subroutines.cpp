#include "main.h"

void setupDribblers(){
    analogWriteFrequency(FrontESCpin, 1000);
    analogWriteFrequency(BackESCpin, 1000);
    analogWrite(FrontESCpin, DRIBBLER_LOWER_LIMIT);
    analogWrite(BackESCpin, DRIBBLER_LOWER_LIMIT);
    delay(3000);
    analogWrite(FrontESCpin, DRIBBLER_UPPER_LIMIT);
    analogWrite(BackESCpin, DRIBBLER_UPPER_LIMIT);
    delay(100);
}

void runDribblers(int speed){
    analogWrite(FrontESCpin, speed);
    analogWrite(BackESCpin, speed);
}