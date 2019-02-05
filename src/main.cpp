#include <Arduino.h>
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <NMEA2000_CAN.h>
#include <NMEA2000_mcp.h>

void setup() {
  WiFiManager wifiManager;
}

void loop() {
  // put your main code here, to run repeatedly:
}