#include <SPI.h>
#include <SCL3300.h>

SCL3300 inclinometer;

const int CSB_PIN = 10; // Chip Select pin

void setup() {
  // Start Serial communication for debugging and plotting
  Serial.begin(9600);

  // Initialize the SCL3300
  while (inclinometer.begin(CSB_PIN) == false) {
    Serial.println("Murata SCL3300 inclinometer not connected. Resetting...");
    
    inclinometer.reset();
    delay(500);
  }

  Serial.println("SCL3300 Initialized!");
}

void loop() {
  // Read angle data from the inclinometer
  if (inclinometer.available()){
    // Retrieve the angle data (assuming the library provides these methods)
    float angleY = convertTo180Range(inclinometer.getCalculatedAngleY());
    float angleZ = convertTo180Range(inclinometer.getCalculatedAngleZ());

    // Print angle data in a format suitable for the Serial Plotter
    // Serial.print("X:"); Serial.print(angleX); Serial.print(" ");
    // Serial.print("Y:"); Serial.print(angleY); Serial.print(" ");
    // Serial.print("Z:"); Serial.println(angleZ);

    // Calculate tilt about the robot's x and y axes
    float tiltX = - angleZ;
    float tiltY = - angleY;
    
    // Print angle data in a format suitable for the Serial Plotter
    Serial.print("X:"); Serial.print(tiltX); Serial.print(" ");
    Serial.print("Y:"); Serial.println(tiltY);

  } else {inclinometer.reset();}
}

// Function to convert angle from 0-360 to -180 to +180
float convertTo180Range(float angle) {
  if (angle > 180.0) {
    angle -= 360.0;
  }
  return angle;
}
