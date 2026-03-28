// https://learn.adafruit.com/adafruit-vl53l1x/arduino
#include "Adafruit_VL53L1X.h"
#include <avr/wdt.h>  // ADD THIS: Include the Watchdog library

#define IRQ_PIN 2
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();
  Wire.setWireTimeout(3000, true); // I2C timeout safety
  
  vl53.begin(0x29, &Wire);
  vl53.setTimingBudget(50);
  vl53.startRanging();

  // ADD THIS: Enable the watchdog timer set to 1 second
  wdt_enable(WDTO_1S); 
}

void loop() {
  // ADD THIS: "Pet the dog". Tell the timer we are still alive. 
  // If the Arduino freezes and misses petting the dog for 1 second, it reboots automatically.
  wdt_reset(); 

  if (vl53.dataReady()) {
    int16_t distance = vl53.distance();
    if (distance >= 0) {
      Serial.println(distance);
    }
    vl53.clearInterrupt();
  }
}