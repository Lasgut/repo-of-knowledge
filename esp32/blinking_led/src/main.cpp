#include <Arduino.h>

// Most ESP32 development boards use GPIO 2 for the onboard LED
const int LED_PIN = 2;

void setup()
{
    pinMode(LED_PIN, OUTPUT);
}

void loop()
{
    digitalWrite(LED_PIN, HIGH); // LED on
    delay(500);

    digitalWrite(LED_PIN, LOW);  // LED off
    delay(500);
}