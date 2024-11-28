#ifdef ENABLE_DEBUG
  #define DEBUG_ESP_PORT Serial
  #define NODEBUG_WEBSOCKETS
  #define NDEBUG
#endif 

#include <Arduino.h>
#include <WiFi.h>
#include "./apiKey.hpp"

#include "SinricPro.h"
#include "SinricProSwitch.h"
#include "ESP32Servo.h"

#define BUTTON_PIN 23   // GPIO for BUTTON (inverted: LOW = pressed, HIGH = released)
#define LED_PIN   2   // GPIO for LED (inverted)
#define servoPin 16   // GPIO for LED (inverted)
#define servoOnDergree 124
#define servoOffDergree 175

bool myPowerState = true;
unsigned long lastBtnPress = 0;
unsigned long lastMoveTime = 0;
bool attached = false;

Servo servo1;

void checkAttach(){
  if(attached && ((unsigned long)(millis() - lastMoveTime) > 500)){
    servo1.detach();
    attached = false;
  }
}

void controlServo(const bool &state) {
  if(state != myPowerState){
    servo1.attach(servoPin);
    servo1.write(state ? servoOnDergree : servoOffDergree);
    lastMoveTime = millis();
    attached = true;
  }
}

bool onPowerState(const String &deviceId, bool &state) {
  Serial.printf("Device %s turned %s (via SinricPro) \r\n", deviceId.c_str(), state?"on":"off");
  myPowerState = state;
  digitalWrite(LED_PIN, myPowerState?LOW:HIGH);
  controlServo(state);
  return true; // request handled properly
}

void handleButtonPress() {
  static bool buttonPressed = false; // 紀錄按鈕是否已經按下

  unsigned long actualMillis = millis(); // 取得當前時間
  if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed) { // 按鈕剛剛被按下
    buttonPressed = true; // 設置按鈕已被按下
    if ((unsigned long)(actualMillis - lastBtnPress) > 1000) { // 防抖檢測（1秒）
      myPowerState = !myPowerState; // 翻轉電燈狀態
      digitalWrite(LED_PIN, myPowerState ? LOW : HIGH); // 更新 LED 狀態
      controlServo(myPowerState);

      // 向 SinricPro 發送事件
      SinricProSwitch& mySwitch = SinricPro[SWITCH_ID];
      bool success = mySwitch.sendPowerStateEvent(myPowerState);
      if (!success) {
        Serial.printf("Something went wrong...could not send Event to server!\r\n");
      }

      Serial.printf("Device %s turned %s (manually via flashbutton)\r\n", 
                    mySwitch.getDeviceId().c_str(), 
                    myPowerState ? "on" : "off");

      lastBtnPress = actualMillis; // 更新最後按鈕按下的時間
    }
  } 

  if (digitalRead(BUTTON_PIN) == HIGH && buttonPressed) { // 按鈕釋放
    buttonPressed = false; // 重置按鈕狀態
  }
}


void setupWiFi() {
  Serial.printf("\r\n[Wifi]: Connecting");

  #if defined(ESP8266)
    WiFi.setSleepMode(WIFI_NONE_SLEEP); 
    WiFi.setAutoReconnect(true);
  #elif defined(ESP32)
    WiFi.setSleep(false); 
    WiFi.setAutoReconnect(true);
  #endif

  WiFi.begin(WIFI_SSID, WIFI_PASS); 

  while (WiFi.status() != WL_CONNECTED) {
    Serial.printf(".");
    delay(250);
  }
  Serial.printf("connected!\r\n[WiFi]: IP-Address is %s\r\n", WiFi.localIP().toString().c_str());
}


void setupSinricPro() {
  // add device to SinricPro
  SinricProSwitch& mySwitch = SinricPro[SWITCH_ID];

  // set callback function to device
  mySwitch.onPowerState(onPowerState);

  // setup SinricPro
  SinricPro.onConnected([](){ Serial.printf("Connected to SinricPro\r\n"); }); 
  SinricPro.onDisconnected([](){ Serial.printf("Disconnected from SinricPro\r\n"); });
  //SinricPro.restoreDeviceStates(true); // Uncomment to restore the last known state from the server.
  SinricPro.begin(APP_KEY, APP_SECRET);
}


void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP); // GPIO 0 as input, pulled high
  pinMode(LED_PIN, OUTPUT); // define LED GPIO as output
  digitalWrite(LED_PIN, HIGH); // turn off LED on bootup

  Serial.begin(BAUD_RATE); Serial.printf("\r\n\r\n");
  setupWiFi();
  setupSinricPro();
  controlServo(true);
}

void loop() {
  handleButtonPress();
  SinricPro.handle();
  checkAttach();
}