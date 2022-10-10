#include <Arduino.h>
#include "AsyncStepper.hpp"
#include "AsyncSerial.hpp"

AsyncStepper MA(22,24,26,28,30);
AsyncStepper MB(23,25,27,29,31);
AsyncSerial serial(0);

void setup() {
  serial.begin(250000);
  // put your setup code here, to run once:
  MA.start(1, 30);
  MB.start(1, 2);
}

void loop() {
  // put your main code here, to run repeatedly:
  MA.loop();
  MB.loop();
  serial.loop();
}