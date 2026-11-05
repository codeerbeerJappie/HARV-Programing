/*
Set board to 'Generic STM32F4 series'
Board part number to 'Generic F401RETx'
Upload method to 'STM32CubeProgrammer (DFU)'
USB support to 'CDC (generic 'Serial' supersede U(S)ART)'
U(S)ART support to 'Disabled (no Serial support)'
USB speed to 'Low/Full Speed'
*/

#include <Feelix.h>

Feelix feelix = Feelix();

const float start_position = 0.0;   // target in radians

unsigned long prevTime = 0;

void setup() {
    delay(250);
    Serial.begin(115200);

    feelix.init();
    delay(100);
    feelix.setVelocityLimit(10.0);  // move back gently
}

void loop() {
    feelix.run();   // must run every loop

    // drive the motor to 0
    feelix.move_feelix(start_position, MotionControlType::angle);

    // print angle every 100 ms
    if (millis() - prevTime >= 100) {
        prevTime = millis();
        Serial.println((String) "angleDeg: " + feelix.angleDeg);
        Serial.print("sensor_direction: ");
            Serial.print((int) feelix.bldc->sensor_direction);
            Serial.print("\tzero_electric_angle: ");
            Serial.println(feelix.bldc->zero_electric_angle, 10);
    }
}