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
#define darkVal 10
#define sleepAtWakeup 2
#define reducedClockSpeed 80

// Set GPIOs for photo resister, LED, PIR Motion Sensor
const int photoResistor = 27;
const int led = 33;
const int motionSensor = 15;

// Motion timer variables
unsigned long now;

// Photo resistor variables
int curLightVal;
int lightVal;
int lightValCount;
unsigned long lastLightTime;

void setup() {

  // Disable bluetooth and wifi
  esp_bt_controller_disable();
  esp_bluedroid_disable();  
  esp_wifi_disconnect();
  esp_wifi_deinit();

  // Serial port for debugging purposes
  //Serial.begin(115200);

  // Reduce clock speed
  setCpuFrequencyMhz(reducedClockSpeed);  
  
  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT_PULLUP);
  
  // Set LED to LOW
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // set darkness sensing variables  
  lightVal = 0;
  lightValCount = 0;

  gpio_wakeup_enable(GPIO_NUM_15, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
}

void loop() {
  
  // Wait a bit after wakeup
  delay(sleepAtWakeup);  

  // Capture photo resistor reading.
  // Ignore lightIgnoreVal, which happens sporadically whether dark or light.
  // Get the total during the reading period so it can be averaged
  now = millis();
  lightVal = 0;
  lightValCount = 0;
  lastLightTime = millis();
  while (lastLightTime - now < (timeLightSensSeconds*1000)) {
    curLightVal = analogRead(photoResistor);
    if (curLightVal != lightIgnoreVal) {
      lightVal += analogRead(photoResistor);
      lightValCount += 1;
    }
    lastLightTime = millis();
  }

  // Take the average reading for specified time to determine if it is dark.  
  lightVal = lightVal / lightValCount;
  //Serial.println("Light sensor read: ");
  //Serial.println(lightVal);
  if (lightVal < darkVal) {
    //Serial.println("It is dark...");
    digitalWrite(led, HIGH);
    delay(timeMotionSeconds*1000);
    digitalWrite(led, LOW);
  } else {      
    //Serial.println("It is light..."); 
  }
    
  // enter deep sleep  
  esp_light_sleep_start();
}
