#include "main.h"
#include "stm32f4.h"

void setup(){
    pinMode(LED_PIN, OUTPUT);

    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(M3_DIR, OUTPUT);
    pinMode(M4_DIR, OUTPUT);

    digitalWrite(M1_DIR,HIGH);
    digitalWrite(M2_DIR,HIGH);
    digitalWrite(M3_DIR, HIGH);
    digitalWrite(M4_DIR, HIGH);

    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M3_PWM, OUTPUT);
    pinMode(PIN_A5, OUTPUT);

    analogWrite(M1_PWM, 200);
    analogWrite(M2_PWM, 200);
    analogWrite(M3_PWM, 200);
    analogWrite(M4_PWM, 200);
}

void loop(){
    for (int i = 100; i<200; i++){
        digitalWrite(M1_DIR,HIGH);
        digitalWrite(M2_DIR,HIGH);
        digitalWrite(M3_DIR, HIGH);
        digitalWrite(M4_DIR, HIGH);
        analogWrite(M1_PWM, i);
        analogWrite(M2_PWM, i);
        analogWrite(M3_PWM, i);
        analogWrite(M4_PWM, i);
        delay(5);

    }
    delay(5);
    for (int i = 100; i<200; i++){
        digitalWrite(M1_DIR,LOW);
        digitalWrite(M2_DIR,LOW);
        digitalWrite(M3_DIR, LOW);
        digitalWrite(M4_DIR, LOW);
        analogWrite(M1_PWM, i);
        analogWrite(M2_PWM, i);
        analogWrite(M3_PWM, i);
        analogWrite(M4_PWM, i);
        delay(5);

    }
}