// Demo: NMEA2000 library. Send battery status to the bus.
//
//
//      In this example are shown ways to minimize the size and RAM usage using two techniques:
//        1) Moving data strictures to PROGMEM vs. using inline constantans when calling a function
//        2) Reducing the size of NMEA CAN buffer to the min needed.  Use caution with this, as some functions
//           (specifically fast packet Messages) require bigger buffer. 
//


#include <Arduino.h>

//#define N2k_CAN_INT_PIN 21          // Comment out to disable interrupt.
#define N2k_SPI_CS_PIN 5
#define USE_N2K_CAN USE_N2K_MCP_CAN   // Force to use NMEA2000_mcp
#define USE_MCP_CAN_CLOCK_SET 8       // Set bus speed to 8 MHz

#include "NMEA2000_CAN.h"       // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "N2kMessages.h"

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={127506L,127508L,127513L,0};

// ---  Example of using PROGMEM to hold Product ID.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const tNMEA2000::tProductInformation BatteryMonitorProductInformation PROGMEM={
                                       1300,                        // N2kVersion
                                       1001,                        // Manufacturer's product code
                                       "Fuel Flow Monitor",         // Manufacturer's Model ID
                                       "0.1.0",                     // Manufacturer's Software version code
                                       "0.1.0",                     // Manufacturer's Model version
                                       "00000001",                  // Manufacturer's Model serial code
                                       0,                           // SertificationLevel
                                       1                            // LoadEquivalency
                                      };                                      

// ---  Example of using PROGMEM to hold Configuration information.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const char BatteryMonitorManufacturerInformation [] PROGMEM = "André Biseth, andre@biseth.net"; 
const char BatteryMonitorInstallationDescription1 [] PROGMEM = "Fuel Flow Monitor"; 
const char BatteryMonitorInstallationDescription2 [] PROGMEM = "Monitoring fuel flow for diesel engine with return fuel line."; 

volatile int frequency; //measuring the rising edges of the signal
int interruptPin = 25;    //The pin location of the sensor

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleInterrupt()     //This is the function that the interupt calls 
{ 
  portENTER_CRITICAL_ISR(&mux);
  frequency++;  //This function measures the rising and falling edge of the hall effect sensors signal
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  // Set Product information
  NMEA2000.SetProductInformation(&BatteryMonitorProductInformation );
  // Set Configuration information
  NMEA2000.SetProgmemConfigurationInformation(BatteryMonitorManufacturerInformation,BatteryMonitorInstallationDescription1,BatteryMonitorInstallationDescription2);
  // Set device information
  NMEA2000.SetDeviceInformation(1,      // Unique number. Use e.g. Serial number.
                                160,    // Device function. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75,     // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                6765    // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );
                               
                               
  // Uncomment 3 rows below to see, what device will send to bus                           
   Serial.begin(115200);
   NMEA2000.SetForwardStream(&Serial);
   NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);     // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode,22);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);     // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  // NMEA2000.EnableForward(false);                      // Disable all msg forwarding to USB (=Serial)
  
//  NMEA2000.SetN2kCANMsgBufSize(2);                    // For this simple example, limit buffer size to 2, since we are only sending data
  NMEA2000.Open();

  pinMode(interruptPin, INPUT_PULLUP); //initializes digital pin 2 as an input
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING); //and the interrupt is attached
}

unsigned long previousMillis = 0;
unsigned long interval = 1000;
unsigned long pulses = 0;
float pulsesPerLiter = 300.0; // Pulses per liter.

void loop() {
  if (millis() - previousMillis > interval) {
    portENTER_CRITICAL_ISR(&mux);
    float calc = (((((float)frequency * (pulsesPerLiter / 1000.0)) * 60.0) * 60.0) / 1000.0);
    pulses += frequency;
    frequency = 0;                  //Set NbTops to 0 ready for calculations
    portEXIT_CRITICAL_ISR(&mux);
    previousMillis = millis();

    SendN2kEngineData(calc);

    Serial.print (pulses, DEC); //Prints the number calculated above
    Serial.println (" pulses"); //Prints "L/hour" and returns a  new line
    Serial.print (calc, DEC); //Prints the number calculated above
    Serial.println (" L/hour"); //Prints "L/hour" and returns a  new line
  }

  NMEA2000.ParseMessages();
}

#define dataUpdatePeriod 1000

void SendN2kEngineData(double fuelRate) {
  tN2kMsg N2kMsg;

  SetN2kEngineDynamicParam(N2kMsg, 1, 0, 0, 0, 0, fuelRate, 0);
  NMEA2000.SendMsg(N2kMsg);
  Serial.println("Sent fuel rate.");
  // SetN2kDCBatStatus(N2kMsg,1,13.87,5.12,35.12,1);
  // NMEA2000.SendMsg(N2kMsg);
  // SetN2kDCStatus(N2kMsg,1,1,N2kDCt_Battery,56,92,38500,0.012);
  // NMEA2000.SendMsg(N2kMsg);
  // SetN2kBatConf(N2kMsg,1,N2kDCbt_Gel,N2kDCES_Yes,N2kDCbnv_12v,N2kDCbc_LeadAcid,AhToCoulomb(420),53,1.251,75);
  // NMEA2000.SendMsg(N2kMsg);
  // Serial.print(millis()); Serial.println(", Battery send ready");
}
