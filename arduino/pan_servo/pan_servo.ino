// Pan servo driven by body-detector (main.py --follow --port ...).
// Protocol: one angle per line over serial, e.g. "97\n". Range 0-180.
//
// Wiring: servo signal -> pin 9, servo power -> 5V, servo ground -> GND.
// A bigger servo (e.g. MG996R) should get its own 5-6V supply, with grounds shared.
//
// Boards: Arduino Nano, Uno, and Seeeduino (Grove) boards — all ATmega328P.
// On a Seeeduino, upload with Board = "Arduino Uno". If the servo is on a Grove
// connector, set SERVO_PIN to that connector's number (Grove "D5" -> 5).

#include <Servo.h>

const int SERVO_PIN = 9;  // change to match your Grove D port

Servo pan;
String line;

void setup() {
  Serial.begin(115200);
  pan.attach(SERVO_PIN);
  pan.write(90);  // start centered, matches PanServo's starting angle
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (line.length() > 0) {
        pan.write(constrain(line.toInt(), 0, 180));
      }
      line = "";
    } else if (isDigit(c)) {
      line += c;  // ignore anything that isn't a digit (e.g. '\r')
    }
  }
}
