#include <Arduino.h>
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <N2kMsg.h>
#include <NMEA2000_CAN.h>
#include <NMEA2000.h>

#include <N2kMsg.h>
#include <NMEA2000.h>
#include <SPI.h>
#include <mcp_can.h> // https://github.com/ttlappalainen/CAN_BUS_Shield
#include <NMEA2000_mcp.h>

#define N2k_CAN_INT_PIN 21 // Pin, where interrupt line has been connected
#define N2k_SPI_CS_PIN 53  // Pin for SPI Can Select

void setup() {
  WiFiManager wifiManager;
}

void loop() {
  // put your main code here, to run repeatedly:
}