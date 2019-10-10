/**
 * Light LED when motion is detected in the dark.
 * Uses a PIR sensor for motion detection and photoresistor to detect darkness. 
 *
 * Shigeru Sasao
**/

#include "esp32-hal-cpu.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_wifi.h"

#define timeMotionSeconds 10
#define timeLightSensSeconds 3
#define lightIgnoreVal 4095
#define darkVal 20
#define sleepWhenLight 30000000
#define reducedClockSpeed 80

// Set GPIOs for photo resister, LED, PIR Motion Sensor
const int photoResistor = 27;
const int led = 33;
const int motionSensor = 15;

// Motion timer variables
unsigned long now = millis();
unsigned long lastMotionTrigger = 0;
boolean startMotionTimer = false;

// Photo resistor variables
int curLightVal;
int lightVal;
int lightValCount;
unsigned long lastLightTime;
boolean isDark = false;

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
  Serial.println("MOTION DETECTED!!!");
  if (isDark) {
    digitalWrite(led, HIGH);
  }
  startMotionTimer = true;
  lastMotionTrigger = millis();
}

void setup() {

  // Disable bluetooth and wifi
  esp_bt_controller_disable();
  esp_bluedroid_disable();  
  esp_wifi_disconnect();
  esp_wifi_deinit();

  // Serial port for debugging purposes
  Serial.begin(115200);

  // Reduce clock speed
  setCpuFrequencyMhz(reducedClockSpeed);  
  
  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT_PULLUP);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  // Set LED to LOW
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // set darkness sensing variables
  lastLightTime = millis();
  lightVal = 0;
  lightValCount = 0;

  esp_sleep_enable_timer_wakeup(sleepWhenLight);  
}

void loop() {
  
  // Current time
  delay(1000);
  now = millis();

  // Capture photo resistor reading.
  // Ignore lightIgnoreVal, which happens sporadically whether dark or light.
  // Get the total during the reading period so it can be averaged
  curLightVal = analogRead(photoResistor);
  if (curLightVal != lightIgnoreVal) {
    lightVal += analogRead(photoResistor);
    lightValCount += 1;
  }

  // Take the average reading for specified time to determine if it is dark.
  if (now - lastLightTime > (timeLightSensSeconds*1000)) {
    lightVal = lightVal / lightValCount;
    Serial.println("Light sensor read: ");
    Serial.println(lightVal);
    if (lightVal < darkVal) {
      isDark = true;
      Serial.println("It is dark...");
    } else {
      isDark = false;
      Serial.println("It is light...");
      esp_deep_sleep_start();
    }
    lightVal = 0;
    lightValCount = 0;
    lastLightTime = millis();
  }
    
  // Turn off the LED after defined time.
  if(startMotionTimer && (now - lastMotionTrigger > (timeMotionSeconds*1000))) {
    Serial.println("Turn off LED...");
    digitalWrite(led, LOW);
    startMotionTimer = false;
  }
}
