#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <ESP32Servo.h>

// BLE tag MAC of your cat's collar
const String knownTagAddress = "ea:49:ed:ef:a8:02";

// RSSI thresholds
const int rssiThresholdOpen = -60;
const int rssiThresholdClose = rssiThresholdOpen;

// BLE scan settings
const int scanTime = 1; // seconds

// Servo and lid settings
const int servoPin = 12;
const int openAngle = 87;
const int closedAngle = 0;
const int slowMoveStep = 3;
const int slowMoveDelay = 30; // ms
const int fastMoveStep = 10;
const int fastMoveDelay = 10; // ms

// Safety settings
const int maxMissingCount = 3;
const unsigned long minLidOpenTime = 10000; // 10 seconds

// State variables
int latestRSSI = -999;
bool tagNearby = false;
bool lidOpen = false;
bool servoAttached = false;
int missingTagCounter = 0;
unsigned long lidOpenedAt = 0;
int currentLidAngle = 0; // Track last known position

BLEScan* pBLEScan;
Servo lidServo;

// BLE device detection callback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getAddress().toString() == knownTagAddress) {
      latestRSSI = advertisedDevice.getRSSI();
      tagNearby = true;
      Serial.printf("Detected tag: %s | RSSI: %d\n", knownTagAddress.c_str(), latestRSSI);
    }
  }
};

void setup() {
  Serial.begin(115200);

  attachServo();
  lidServo.write(closedAngle);
  currentLidAngle = closedAngle;
  detachServo();

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false);
}

void loop() {
  tagNearby = false;
  latestRSSI = -999;

  pBLEScan->start(scanTime, false);
  pBLEScan->clearResults();

  Serial.printf("BLE: %s | RSSI: %d\n",
                tagNearby ? "yes" : "no",
                latestRSSI);

  if (!lidOpen && tagNearby && latestRSSI > rssiThresholdOpen) {
    Serial.println("Opening lid fast...");
    attachServo();
    fastMove(openAngle);
    delay(500); // Let the servo finish moving
    detachServo();
    lidOpen = true;
    lidOpenedAt = millis();
    missingTagCounter = 0;
  }

  if (lidOpen) {
    if (!tagNearby || latestRSSI < rssiThresholdClose) {
      missingTagCounter++;
      Serial.printf("Missing count: %d\n", missingTagCounter);
    } else {
      missingTagCounter = 0;
    }

    if (millis() - lidOpenedAt > minLidOpenTime && missingTagCounter >= maxMissingCount) {
      Serial.println("Closing lid slowly...");
      attachServo();
      slowMove(closedAngle);
      delay(500); // Let the servo finish moving
      detachServo();
      lidOpen = false;
      missingTagCounter = 0;
    }
  }

  delay(1000);
}

void slowMove(int targetAngle) {
  moveLid(targetAngle, slowMoveStep, slowMoveDelay);
}

void fastMove(int targetAngle) {
  moveLid(targetAngle, fastMoveStep, fastMoveDelay);
}

void moveLid(int targetAngle, int stepSize, int stepDelay) {
  if (currentLidAngle == targetAngle) return;

  int step = (targetAngle > currentLidAngle) ? abs(stepSize) : -abs(stepSize);

  int angle = currentLidAngle;
  while ((step > 0 && angle < targetAngle) || (step < 0 && angle > targetAngle)) {
    lidServo.write(angle);
    delay(stepDelay);
    angle += step;
  }

  lidServo.write(targetAngle); // Final exact position
  currentLidAngle = targetAngle;
}

void attachServo() {
  if (!servoAttached) {
    lidServo.attach(servoPin);
    servoAttached = true;
    Serial.println("Servo attached.");
  }
}

void detachServo() {
  if (servoAttached) {
    lidServo.detach();
    servoAttached = false;
    Serial.println("Servo detached.");
    delay(50); // Safe detach
  }
}
