#include "HX711.h"

// Define pins
  // Scale pins
#define DOUT 21
#define CLK 20

// Set up constants
const float calibration_factor = -21.9;

// For printing results
unsigned long currTime = 0; // Current time (ms)
unsigned long lastPrintTime = 0; // Initialize the last time that results were printed (ms)
const float printDelay = 500; // How long to delay between printing results (ms)
float force = 0;

// Configure scale
HX711 scale;

void tare(){
  // Tare the scale. This can be necessary after running a run_test
  scale.tare(20);
  // delay(1000); // "Standard practice" to wait for a bit after taring. Not necessary??
}

void setup() {
  Serial.begin(115200);

  // Set up scale
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor);
  scale.tare(20);  // Was scale.tare(20)
}

void loop() {
  currTime = millis();

  if (currTime - lastPrintTime >= printDelay){
    force = scale.get_units() * 9.81 / 1000; // Default is g, so convert to N

    Serial.println(force);
    lastPrintTime = currTime; // Update the last print time
  }
}
