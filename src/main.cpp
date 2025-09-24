#include <Wire.h>
#include <VL53L1X.h>

template <typename... T>
void log(const char *fmt, T&&... args)
{
  Serial.printf(fmt, args...);
  Serial.println();
}

struct SensorData {
  VL53L1X sensor;
  uint8_t xshutPin {};
  uint8_t interruptPin {};
  void (*interruptHandler)() {};
  volatile bool dataReady {};

  // For detection
  std::array<uint16_t, 3> recentReadings {};
  uint8_t readingIndex {};
  uint16_t currentDistance {};
  uint32_t lastValidReading {};
  bool hasValidReading {};
};

void gpio0_isr();
void gpio1_isr();
void gpio2_isr();
void gpio3_isr();

SensorData sensorsData[] = {
  {
    {},
    33, // XSHUT pin
    2,  // interrupt pin
    gpio0_isr // interrupt handler
  },
};

// We can't do any calculations or whatever in interrupt handlers.
void gpio0_isr() { sensorsData[0].dataReady = true; }
// void gpio1_isr() { sensorsData[1].dataReady = true; }
// void gpio2_isr() { sensorsData[2].dataReady = true; }
// void gpio3_isr() { sensorsData[3].dataReady = true; }

struct DetectionCandidate
{
  uint16_t distance {};
  uint32_t firstSeen {};
  uint8_t confirmedSensors {};
  bool active {};
};

// Just some bullshit for a playground.
const uint16_t MAX_DETECTION_RANGE = 4000;
const uint16_t MIN_DETECTION_RANGE = 10;
const uint16_t NOISE_THRESHOLD = 10;
const uint16_t DETECTION_WINDOW_MS = 100;
const uint16_t MAX_READING_AGE_MS = 60;
const uint16_t ALERT_COOLDOWN_MS = 20;
const uint16_t MIN_CONFIRMING_SENSORS = (std::size(sensorsData) + 1) / 2;

DetectionCandidate currentCandidate;
uint16_t lastDetectionAlert = 0;

// Median shit
uint16_t getFilteredReading(const SensorData &s)
{
  auto readings = s.recentReadings;
  std::sort(std::begin(readings), std::end(readings));
  return readings[readings.size() / 2];
}

bool isValidTarget(uint16_t distance)
{
  return (distance >= MIN_DETECTION_RANGE && distance <= MAX_DETECTION_RANGE);
}

void updateDetection(uint32_t currentTime)
{
  uint8_t sensorsWithTargets = 0;
  uint16_t averageDistance = 0;
  uint32_t distanceSum = 0;

  // Count how many sensors currently see valid targets
  for (size_t i = 0; i < std::size(sensorsData); ++i)
  {
    if (sensorsData[i].hasValidReading &&
        (currentTime - sensorsData[i].lastValidReading) < MAX_READING_AGE_MS &&
        isValidTarget(sensorsData[i].currentDistance))
    {
      ++sensorsWithTargets;
      distanceSum += sensorsData[i].currentDistance;
    }
  }

  if (sensorsWithTargets >= MIN_CONFIRMING_SENSORS)
  {
    averageDistance = distanceSum / sensorsWithTargets;

    if (!currentCandidate.active)
    {
      // New detection candidate
      currentCandidate.distance = averageDistance;
      currentCandidate.firstSeen = currentTime;
      currentCandidate.confirmedSensors = sensorsWithTargets;
      currentCandidate.active = true;
    }
    else
    {
      // Update existing candidate
      currentCandidate.distance = (currentCandidate.distance + averageDistance) / 2;
      currentCandidate.confirmedSensors = std::max(currentCandidate.confirmedSensors, sensorsWithTargets);
    }
  }
  else
  {
    // Reset candidate if not enough confirming sensors
    if (currentCandidate.active && (currentTime - currentCandidate.firstSeen) > DETECTION_WINDOW_MS)
    {
      currentCandidate.active = false;
    }
  }

  if (currentCandidate.active &&
    currentCandidate.confirmedSensors >= MIN_CONFIRMING_SENSORS &&
    (currentTime - lastDetectionAlert) > ALERT_COOLDOWN_MS)
  {

    log("Some shit detected. Distance: %u mm, Confirmed by %d/%d sensors,",
        currentCandidate.distance,
        currentCandidate.confirmedSensors,
        std::size(sensorsData));

    lastDetectionAlert = currentTime;

    // digitalWrite(BLOW_THE_SHIT_PIN, HIGH);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Start");

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  for (auto &sd : sensorsData)
  {
    pinMode(sd.xshutPin, OUTPUT);
    digitalWrite(sd.xshutPin, LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (size_t i = 0; i < std::size(sensorsData); ++i)
  {
    auto &sd = sensorsData[i];
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(sd.xshutPin, INPUT);
    delay(10);

    sd.sensor.setTimeout(500);
    
    if (!sd.sensor.init())
    {
      log("Failed to detect and initialize sensor %d", i);
      while (1); // Why?
    }

    pinMode(sd.interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(sd.interruptPin), sd.interruptHandler, FALLING);

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sd.sensor.setAddress(0x2A + i);

    // How will this impact the measurements?
    sd.sensor.setDistanceMode(VL53L1X::DistanceMode::Long);

    // The minimum and maximum timing budgets are [20ms, 1000ms], according to sensor spec
    sd.sensor.setMeasurementTimingBudget(20*1000); // <- microseconds
    // The minimum inter-measurement period is 20ms, according to sensor spec
    sd.sensor.startContinuous(20);
  }
}

void loop()
{
  const uint32_t now = ::millis();

  for (size_t i = 0; i < std::size(sensorsData); ++i)
  {
    auto &sd = sensorsData[i];
    if (!sd.dataReady) continue;
    sd.dataReady = false;

    // Is it really beneficial to have interrupt check + non-blocking read?
    // I mean, it seems to be there is a lot of garbage data to filter out in this case.
    sd.sensor.read(false);
    if (sd.sensor.timeoutOccurred())
    {
      // Most likely sensor is dead. How to hard-reset it?
      log("Sensor %d timeout", i);
      continue;
    }

    const auto &rangingData = sd.sensor.ranging_data;
    if (rangingData.range_status != VL53L1X::RangeStatus::RangeValid) continue;

    sd.recentReadings[sd.readingIndex] = rangingData.range_mm;
    sd.readingIndex = (sd.readingIndex + 1) % sd.recentReadings.size();

    sd.currentDistance = getFilteredReading(sd);
    sd.lastValidReading = now;
    sd.hasValidReading = true;
  }

  updateDetection(now);
}
