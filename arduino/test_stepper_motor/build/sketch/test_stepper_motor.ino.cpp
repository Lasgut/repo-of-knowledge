#include <Arduino.h>
#line 1 "C:\\Users\\lmgut\\OneDrive\\Programmering\\Arduino\\test_stepper_motor\\test_stepper_motor.ino"
#define STEPPER_PIN_1 9
#define STEPPER_PIN_2 10
#define STEPPER_PIN_3 11
#define STEPPER_PIN_4 12

int step_number = 0;

#line 8 "C:\\Users\\lmgut\\OneDrive\\Programmering\\Arduino\\test_stepper_motor\\test_stepper_motor.ino"
void setup();
#line 15 "C:\\Users\\lmgut\\OneDrive\\Programmering\\Arduino\\test_stepper_motor\\test_stepper_motor.ino"
void loop();
#line 20 "C:\\Users\\lmgut\\OneDrive\\Programmering\\Arduino\\test_stepper_motor\\test_stepper_motor.ino"
void OneStep(bool dir);
#line 8 "C:\\Users\\lmgut\\OneDrive\\Programmering\\Arduino\\test_stepper_motor\\test_stepper_motor.ino"
void setup() {
    pinMode(STEPPER_PIN_1, OUTPUT);
    pinMode(STEPPER_PIN_2, OUTPUT);
    pinMode(STEPPER_PIN_3, OUTPUT);
    pinMode(STEPPER_PIN_4, OUTPUT);
}

void loop() {
    OneStep(true);
    delay(10);
}

void OneStep(bool dir){
    if(dir){
        switch(step_number){
            case 0:
            digitalWrite(STEPPER_PIN_1, HIGH);
            digitalWrite(STEPPER_PIN_2, LOW);
            digitalWrite(STEPPER_PIN_3, LOW);
            digitalWrite(STEPPER_PIN_4, LOW);
            break;
            case 1:
            digitalWrite(STEPPER_PIN_1, LOW);
            digitalWrite(STEPPER_PIN_2, HIGH);
            digitalWrite(STEPPER_PIN_3, LOW);
            digitalWrite(STEPPER_PIN_4, LOW);
            break;
            case 2:
            digitalWrite(STEPPER_PIN_1, LOW);
            digitalWrite(STEPPER_PIN_2, LOW);
            digitalWrite(STEPPER_PIN_3, HIGH);
            digitalWrite(STEPPER_PIN_4, LOW);
            break;
            case 3:
            digitalWrite(STEPPER_PIN_1, LOW);
            digitalWrite(STEPPER_PIN_2, LOW);
            digitalWrite(STEPPER_PIN_3, LOW);
            digitalWrite(STEPPER_PIN_4, HIGH);
            break;
        }
    }
    else{
        switch(step_number){
            case 0:
            digitalWrite(STEPPER_PIN_1, LOW);
            digitalWrite(STEPPER_PIN_2, LOW);
            digitalWrite(STEPPER_PIN_3, LOW);
            digitalWrite(STEPPER_PIN_4, HIGH);
            break;
            case 1:
            digitalWrite(STEPPER_PIN_1, LOW);
            digitalWrite(STEPPER_PIN_2, LOW);
            digitalWrite(STEPPER_PIN_3, HIGH);
            digitalWrite(STEPPER_PIN_4, LOW);
            break;
            case 2:
            digitalWrite(STEPPER_PIN_1, LOW);
            digitalWrite(STEPPER_PIN_2, HIGH);
            digitalWrite(STEPPER_PIN_3, LOW);
            digitalWrite(STEPPER_PIN_4, LOW);
            break;
            case 3:
            digitalWrite(STEPPER_PIN_1, HIGH);
            digitalWrite(STEPPER_PIN_2, LOW);
            digitalWrite(STEPPER_PIN_3, LOW);
            digitalWrite(STEPPER_PIN_4, LOW);
            break;
        }        
    }
    step_number++;
    if(step_number > 3){
        step_number = 0;
    }
}
