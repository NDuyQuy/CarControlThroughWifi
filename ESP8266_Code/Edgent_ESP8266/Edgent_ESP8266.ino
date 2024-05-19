/*************************************************************
  Blynk is a platform with iOS and Android apps to control
  ESP32, Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build mobile and web interfaces for any
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: https://www.blynk.io
    Sketch generator:           https://examples.blynk.cc
    Blynk community:            https://community.blynk.cc
    Follow us:                  https://www.fb.com/blynkapp
                                https://twitter.com/blynk_app

  Blynk library is licensed under MIT license
 *************************************************************
  Blynk.Edgent implements:
  - Blynk.Inject - Dynamic WiFi credentials provisioning
  - Blynk.Air    - Over The Air firmware updates
  - Device state indication using a physical LED
  - Credentials reset using a physical Button
 *************************************************************/

/* Fill in information from your Blynk Template here */
/* Read more: https://bit.ly/BlynkInject */
//#define BLYNK_TEMPLATE_ID           "TMPxxxxxx"
//#define BLYNK_TEMPLATE_NAME         "Device"
#define BLYNK_TEMPLATE_ID "TMPL6WWi0FveN"
#define BLYNK_TEMPLATE_NAME "ESP8266MyCar"
#define BLYNK_AUTH_TOKEN    "cr5f6Of0KJ8O-FTYavdNslYS7fQX-JJC"
#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h
//#define USE_SPARKFUN_BLYNK_BOARD
//#define USE_NODE_MCU_BOARD
//#define USE_WITTY_CLOUD_BOARD
//#define USE_WEMOS_D1_MINI

//#include "BlynkEdgent.h"
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>
const char ssid[] = "Oc Soc";
const char pass[] = "01234554321";

//DEFINE OUTPUT PIN TO CONTROL MOTOR 
typedef enum{
  STOP,
  FORWARD,
  BACKWARD,
  TURNLEFT,
  TURNRIGHT,
  SETSPEED
}trig_mode_enum;
void sendMessage(trig_mode_enum mode, uint8_t val)
{
  char msg[20];
  sprintf(msg, "*%d-%d#", mode, val);
  Serial.write(msg);
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  //BlynkEdgent.begin();
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  Blynk.run();
  //BlynkEdgent.run();
  // handle_trig();
}

BLYNK_WRITE(V0){//forward both two engine
  int p = param.asInt();
  p?sendMessage(FORWARD,p):sendMessage(STOP,0);
}

BLYNK_WRITE(V1){//backward both two engine 
  int p = param.asInt();
  p?sendMessage(BACKWARD,p):sendMessage(STOP,0);
}
BLYNK_WRITE(V2){
  //forward engine in the left
  int p = param.asInt();
  p?sendMessage(TURNLEFT,p):sendMessage(STOP,0);
}

BLYNK_WRITE(V3){//forward engine in the right
  int p = param.asInt();
  p?sendMessage(TURNRIGHT,p):sendMessage(STOP,0);
}
void do_nothing(){}
BLYNK_WRITE(V4){//forward engine in the right
  int p = param.asInt();
  p?sendMessage(SETSPEED,p):do_nothing();
}
