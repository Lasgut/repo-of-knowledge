# 1 "C:\\Users\\lmgut\\OneDrive\\Programmering\\Arduino\\test_stepper_motor\\test_stepper_motor.ino"





int step_number = 0;

void setup() {
    pinMode(9, 0x1);
    pinMode(10, 0x1);
    pinMode(11, 0x1);
    pinMode(12, 0x1);
}

void loop() {
    OneStep(true);
    delay(10);
}

void OneStep(bool dir){
    if(dir){
        switch(step_number){
            case 0:
            digitalWrite(9, 0x1);
            digitalWrite(10, 0x0);
            digitalWrite(11, 0x0);
            digitalWrite(12, 0x0);
            break;
            case 1:
            digitalWrite(9, 0x0);
            digitalWrite(10, 0x1);
            digitalWrite(11, 0x0);
            digitalWrite(12, 0x0);
            break;
            case 2:
            digitalWrite(9, 0x0);
            digitalWrite(10, 0x0);
            digitalWrite(11, 0x1);
            digitalWrite(12, 0x0);
            break;
            case 3:
            digitalWrite(9, 0x0);
            digitalWrite(10, 0x0);
            digitalWrite(11, 0x0);
            digitalWrite(12, 0x1);
            break;
        }
    }
    else{
        switch(step_number){
            case 0:
            digitalWrite(9, 0x0);
            digitalWrite(10, 0x0);
            digitalWrite(11, 0x0);
            digitalWrite(12, 0x1);
            break;
            case 1:
            digitalWrite(9, 0x0);
            digitalWrite(10, 0x0);
            digitalWrite(11, 0x1);
            digitalWrite(12, 0x0);
            break;
            case 2:
            digitalWrite(9, 0x0);
            digitalWrite(10, 0x1);
            digitalWrite(11, 0x0);
            digitalWrite(12, 0x0);
            break;
            case 3:
            digitalWrite(9, 0x1);
            digitalWrite(10, 0x0);
            digitalWrite(11, 0x0);
            digitalWrite(12, 0x0);
            break;
        }
    }
    step_number++;
    if(step_number > 3){
        step_number = 0;
    }
}
