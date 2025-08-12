# INO-s

## Experiment 1: Automated Climate Control System
### Creating an Intelligent Temperature Management System
### Programming the Climate Control System

```cpp
#include "thingProperties.h"
#include <DHT.h>
#include <ESP32Servo.h>

#define DHT_PIN 4
#define DHT_TYPE DHT22
#define SERVO_PIN 18
#define LED_PIN 2

DHT dht(DHT_PIN, DHT_TYPE);
Servo fanServo;

unsigned long lastSensorRead = 0;
unsigned long lastLEDBlink = 0;
bool ledState = false;
int currentFanSpeed = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  dht.begin();
  fanServo.attach(SERVO_PIN);
  
  // Initialize cloud properties
  initProperties();
  
  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop() {
  ArduinoCloud.update();
  
  // Read sensors every 2 seconds
  if (millis() - lastSensorRead > 2000) {
    readSensors();
    lastSensorRead = millis();
  }
  
  // Update LED blinking based on temperature
  updateStatusLED();
  
  // Control fan based on temperature (unless override is active)
  if (!systemOverride) {
    autoControlFan();
  }
}

void readSensors() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  
  if (!isnan(temp) && !isnan(hum)) {
    temperature = temp;
    humidity = hum;
    Serial.printf("Temperature: %.1f°C, Humidity: %.1f%%\n", temp, hum);
  }
}

void autoControlFan() {
  int targetSpeed = 0;
  
  if (temperature < 25.0) {
    targetSpeed = 0;    // Fan off
    coolingActive = false;
  } else if (temperature <= 30.0) {
    targetSpeed = 90;   // Medium speed
    coolingActive = true;
  } else {
    targetSpeed = 180;  // Maximum speed
    coolingActive = true;
  }
  
  if (targetSpeed != currentFanSpeed) {
    currentFanSpeed = targetSpeed;
    fanSpeed = targetSpeed;
    fanServo.write(targetSpeed);
    Serial.printf("Fan speed set to: %d degrees\n", targetSpeed);
  }
}

void updateStatusLED() {
  unsigned long blinkInterval;
  
  if (temperature < 25.0) {
    // Steady on for comfortable temperature
    digitalWrite(LED_PIN, HIGH);
    return;
  } else if (temperature <= 30.0) {
    blinkInterval = 1000;  // Slow blink for moderate temperature
  } else {
    blinkInterval = 200;   // Fast blink for critical temperature
  }
  
  if (millis() - lastLEDBlink > blinkInterval) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastLEDBlink = millis();
  }
}

// Cloud variable callbacks
void onSystemOverrideChange() {
  if (systemOverride) {
    Serial.println("Manual override activated");
  } else {
    Serial.println("Automatic control resumed");
  }
}

void onFanSpeedChange() {
  if (systemOverride) {
    fanServo.write(fanSpeed);
    currentFanSpeed = fanSpeed;
    Serial.printf("Manual fan speed set to: %d\n", fanSpeed);
  }
}

---

## Experiment 2: Smart Security System  
### Building an Intelligent Motion Detection System


### Detection Algorithm
```cpp
// Motion detection based on distance changes
bool detectMotion() {
  int currentDistance = getDistance();
  
  if (abs(currentDistance - baselineDistance) > MOTION_THRESHOLD) {
    return true;
  }
  return false;
}

### Arduino IoT Cloud Configuration

1. **Create New Thing**: "Smart_Security_System"

2. **Cloud Variables**:
   ```
   - systemArmed (bool, read/write) - Security system status
   - motionDetected (bool, read-only) - Current motion status  
   - detectionCount (int, read-only) - Total detections today
   - lastDetectionTime (string, read-only) - Timestamp of last event
   - alarmActive (bool, read-only) - Current alarm status
   - sensitivity (int, read/write) - Detection sensitivity (1-10)
   ```

3. **Dashboard Elements**:
   - **Switch Widget**: Arm/Disarm security system
   - **LED Indicator**: Motion detection status
   - **Counter Widget**: Daily detection count
   - **Text Widget**: Last detection timestamp
   - **Slider Widget**: Sensitivity adjustment
   - **Chart Widget**: Detection frequency over time

### Programming the Security System

```cpp
#include "thingProperties.h"

#define TRIG_PIN 5
#define ECHO_PIN 19
#define LED_PIN 2
#define BUZZER_PIN 21

#define MOTION_THRESHOLD 10  // cm difference for motion detection
#define ALARM_DURATION 5000  // 5 seconds alarm

unsigned long lastDistanceRead = 0;
unsigned long alarmStartTime = 0;
int baselineDistance = 0;
bool alarmTriggered = false;
int dailyDetectionCount = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize cloud properties
  initProperties();
  
  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
  
  // Set baseline distance
  delay(2000);
  baselineDistance = getDistance();
  Serial.printf("Baseline distance set to: %d cm\n", baselineDistance);
}

void loop() {
  ArduinoCloud.update();
  
  // Check for motion every 100ms
  if (millis() - lastDistanceRead > 100) {
    checkForMotion();
    lastDistanceRead = millis();
  }
  
  // Handle active alarm
  if (alarmTriggered) {
    handleAlarm();
  }
}

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  
  return distance;
}

void checkForMotion() {
  int currentDistance = getDistance();
  
  // Apply sensitivity factor
  int threshold = MOTION_THRESHOLD - (sensitivity - 5);  // Adjust based on sensitivity
  
  if (abs(currentDistance - baselineDistance) > threshold) {
    if (!motionDetected) {
      motionDetected = true;
      recordDetection();
      
      if (systemArmed) {
        triggerAlarm();
      }
    }
  } else {
    motionDetected = false;
  }
}

void recordDetection() {
  detectionCount++;
  dailyDetectionCount++;
  
  // Update timestamp (simplified - in practice use RTC or NTP)
  lastDetectionTime = "Detection at " + String(millis()/1000) + "s";
  
  Serial.printf("Motion detected! Count: %d\n", dailyDetectionCount);
}

void triggerAlarm() {
  if (!alarmTriggered) {
    alarmTriggered = true;
    alarmActive = true;
    alarmStartTime = millis();
    
    Serial.println("SECURITY ALERT: Motion detected while armed!");
    
    // Start alarm sequence
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  }
}

void handleAlarm() {
  // Stop alarm after duration
  if (millis() - alarmStartTime > ALARM_DURATION) {
    alarmTriggered = false;
    alarmActive = false;
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Alarm deactivated");
  } else {
    // Flashing LED during alarm
    digitalWrite(LED_PIN, (millis() / 200) % 2);
  }
}

// Cloud variable callbacks
void onSystemArmedChange() {
  if (systemArmed) {
    Serial.println("Security system ARMED");
    // Flash LED 3 times to confirm arming
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  } else {
    Serial.println("Security system DISARMED");
    // Turn off any active alarms
    alarmTriggered = false;
    alarmActive = false;
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void onSensitivityChange() {
  Serial.printf("Sensitivity adjusted to: %d\n", sensitivity);
  // Recalibrate baseline if needed
  baselineDistance = getDistance();
}
```

---
```

---
