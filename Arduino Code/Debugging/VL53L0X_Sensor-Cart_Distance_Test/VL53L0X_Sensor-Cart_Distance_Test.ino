#include <Wire.h>          // Required for I2C communication
#include <VL53L0X.h>       // Include the VL53L0X library header

VL53L0X sensor;            // Create an object for the VL53L0X sensor

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Initializing VL53L0X...");
  if (!sensor.init()) {
    Serial.println("Failed to initialize VL53L0X sensor!");
    while (1); // Halt
  }
  sensor.setTimeout(500);
  Serial.println("VL53L0X ready!");
}

void loop() {
  int distance = sensor.readRangeSingleMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.println("Timeout!");
  } else {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
  }
  delay(100);
}
