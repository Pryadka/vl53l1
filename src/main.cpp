#include <Wire.h>
#include <VL53L1X.h>


const uint8_t sensorCount = 1;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = {33};

VL53L1X sensors[sensorCount];

void setup()
{
  Serial.begin(115200);

  Serial.println("Start");
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    int res = sensors[i].init();
    if (res != 0)
    {
      Serial.print("Failed to detect and initialize sensor. res=");Serial.println(res, HEX);
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x2A + i);

    sensors[i].startContinuous(2);
  }
}

void loop()
{
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    const auto distance = sensors[i].read();
    if (sensors[i].timeoutOccurred()) { 
      Serial.println("TIMEOUT"); 
    }
    if (distance < 500) {
      Serial.print("BUH=");Serial.print(i);
      Serial.print(" Distance: ");Serial.print(distance);Serial.println(" mm");
    }
  }
}