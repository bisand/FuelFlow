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
#define USE_N2K_CAN USE_N2K_MCP_CAN // Force to use NMEA2000_mcp
#define USE_MCP_CAN_CLOCK_SET 8     // Set bus speed to 8 MHz

#include "NMEA2000_CAN.h" // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "N2kMessages.h"

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM = {127506L, 127508L, 127513L, 0};

// ---  Example of using PROGMEM to hold Product ID.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const tNMEA2000::tProductInformation BatteryMonitorProductInformation PROGMEM = {
    1300,                // N2kVersion
    1001,                // Manufacturer's product code
    "Fuel Flow Monitor", // Manufacturer's Model ID
    "0.1.1",             // Manufacturer's Software version code
    "0.1.1",             // Manufacturer's Model version
    "00000001",          // Manufacturer's Model serial code
    0,                   // SertificationLevel
    1                    // LoadEquivalency
};

// ---  Example of using PROGMEM to hold Configuration information.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const char BatteryMonitorManufacturerInformation[] PROGMEM = "André Biseth, andre@biseth.net";
const char BatteryMonitorInstallationDescription1[] PROGMEM = "Fuel Flow Monitor";
const char BatteryMonitorInstallationDescription2[] PROGMEM = "Monitoring fuel flow for diesel engine with return fuel line.";

#define MAX_ELAPSED_MS 60000

int flowInPin = 25;  //The pin location of the input sensor
int flowOutPin = 26; //The pin location of the output sensor

volatile unsigned int pulsesIn = 0;  //Pulses from incoming fuel flow.
volatile unsigned int pulsesOut = 0; //Pulses from outgoing fuel flow.

volatile unsigned long msLastIn = 0;  // Last registered milliseconds from inbound interrupt.
volatile unsigned long msLastOut = 0; // Last registered milliseconds from outbound interrupt.
volatile unsigned long msElapsedIn = MAX_ELAPSED_MS;   // Milliseconds elapsed since last inbound interrupt.
volatile unsigned long msElapsedOut = MAX_ELAPSED_MS;  // Milliseconds elapsed since last outbound interrupt.

// Mutex used by interrupt 
portMUX_TYPE muxIn = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxOut = portMUX_INITIALIZER_UNLOCKED;

/*
  Interrupt for the incoming fuel flow.
  This function is triggered on the falling edge of the hall effect sensors signal
*/
void IRAM_ATTR flowInInterrupt()
{
  portENTER_CRITICAL_ISR(&muxIn);
  pulsesIn++; 
  unsigned long ms = millis();
  msElapsedIn = ms - msLastIn;
  msLastIn = ms;
  portEXIT_CRITICAL_ISR(&muxIn);
}

/*
  Interrupt for the outgoing fuel flow.
  This function is triggered on the falling edge of the hall effect sensors signal
*/
void IRAM_ATTR flowOutInterrupt()
{
  portENTER_CRITICAL_ISR(&muxOut);
  pulsesOut++;
  unsigned long ms = millis();
  msElapsedOut = ms - msLastOut;
  msLastOut = ms;
  portEXIT_CRITICAL_ISR(&muxOut);
}

/*
  The setup
  initializes libraries and values.
*/
void setup()
{
  // Set Product information
  NMEA2000.SetProductInformation(&BatteryMonitorProductInformation);
  // Set Configuration information
  NMEA2000.SetProgmemConfigurationInformation(BatteryMonitorManufacturerInformation, BatteryMonitorInstallationDescription1, BatteryMonitorInstallationDescription2);
  // Set device information
  NMEA2000.SetDeviceInformation(1,   // Unique number. Use e.g. Serial number.
                                160, // Device function. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75,  // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                6765 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  // Uncomment 3 rows below to see, what device will send to bus
  Serial.begin(115200);
  //NMEA2000.SetForwardStream(&Serial);
  //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);     // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 22);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_ClearText);     // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  // NMEA2000.EnableForward(false);                      // Disable all msg forwarding to USB (=Serial)

  //  NMEA2000.SetN2kCANMsgBufSize(2);                    // For this simple example, limit buffer size to 2, since we are only sending data
  NMEA2000.Open();

  pinMode(flowInPin, INPUT);                                                     //initializes digital pin 2 as an input
  attachInterrupt(digitalPinToInterrupt(flowInPin), flowInInterrupt, FALLING);   //and the interrupt is attached
  pinMode(flowOutPin, INPUT);                                                    //initializes digital pin 2 as an input
  attachInterrupt(digitalPinToInterrupt(flowOutPin), flowOutInterrupt, FALLING); //and the interrupt is attached
}

unsigned long prevMillis = 0;
unsigned long interval = 1000;

float pulsesPerLiterIn = 169.0;             // Pulses per liter In (old=153)
float pulsesPerLiterOut = 169.0;            // Pulses per liter Out (old=185)
float mlppIn = 1000.0 / pulsesPerLiterIn;   // Milliliters per pulse = 5.9172
float mlppOut = 1000.0 / pulsesPerLiterOut; // Milliliters per pulse = 5.9172

/*
  Send NMEA 2000 engine data out on the bus.
*/
void SendN2kEngineData(double fuelRate)
{
  tN2kMsg N2kMsg;

  SetN2kEngineDynamicParam(N2kMsg, 0, 0, 0, 0, 0, fuelRate, 0);
  NMEA2000.SendMsg(N2kMsg);
  Serial.println("Sent fuel rate.");
}

unsigned long tmpPulsesIn = 0;
unsigned long tmpPulsesOut = 0;

unsigned long tmpMsElapsedIn = 0;
unsigned long tmpMsElapsedOut = 0;

#define CALC_FREQ false

/*
  The loop
  The continuously running loop
*/
void loop()
{
  if (millis() - prevMillis > interval)
  {
    prevMillis = millis();

    portENTER_CRITICAL_ISR(&muxIn);
    tmpPulsesIn = pulsesIn;
    pulsesIn = 0;
    if(msElapsedIn > MAX_ELAPSED_MS)
      msElapsedIn = MAX_ELAPSED_MS;
    tmpMsElapsedIn = msElapsedIn;
    portEXIT_CRITICAL_ISR(&muxIn);

    portENTER_CRITICAL_ISR(&muxOut);
    tmpPulsesOut = pulsesOut;
    pulsesOut = 0;
    if(msElapsedOut > MAX_ELAPSED_MS)
      msElapsedOut = MAX_ELAPSED_MS;
    tmpMsElapsedOut = msElapsedOut;
    portEXIT_CRITICAL_ISR(&muxOut);

    // Prevent division by zero.
    if (tmpMsElapsedIn == 0)
      tmpMsElapsedIn = MAX_ELAPSED_MS;

    // Prevent division by zero.
    if (tmpMsElapsedOut == 0)
      tmpMsElapsedOut = MAX_ELAPSED_MS;

    float calcIn = 0.0;
    float calcOut = 0.0;

    if (CALC_FREQ || tmpPulsesIn > 5)
    {
      // Calculates flow by frequency. Works better for higher flow rates.
      calcIn = (((mlppIn * tmpPulsesIn) * 60.0) * 60.0);    // mL/hr
      calcIn = calcIn / 1000.0;                           // L/hr
      calcOut = (((mlppOut * tmpPulsesOut) * 60.0) * 60.0); // mL/hr
      calcOut = calcOut / 1000.0;                         // L/hr
    }
    else
    {
      // Calculates flow by elapsed milliseconds. Works better on lower flow rates.
      calcIn = (mlppIn / ((float)tmpMsElapsedIn / 1000.0));    // mL/s
      calcIn = ((calcIn * 60.0) * 60.0) / 1000.0;              // L/hr
      calcOut = (mlppOut / ((float)tmpMsElapsedOut / 1000.0)); // mL/s
      calcOut = ((calcOut * 60.0) * 60.0) / 1000.0;            // L/hr
    }

    if (tmpMsElapsedIn > (MAX_ELAPSED_MS / 4))
      calcIn = calcIn - (calcIn / 4);
    if (tmpMsElapsedIn > (MAX_ELAPSED_MS / 2))
      calcIn = calcIn - (calcIn / 2);
    if (tmpMsElapsedIn == MAX_ELAPSED_MS)
      calcIn = 0.0;

    if (tmpMsElapsedOut > (MAX_ELAPSED_MS / 4))
      calcOut = calcOut - (calcOut / 4);
    if (tmpMsElapsedOut > (MAX_ELAPSED_MS / 2))
      calcOut = calcOut - (calcOut / 2);
    if (tmpMsElapsedOut == MAX_ELAPSED_MS)
      calcOut = 0.0;

    float calc = calcIn - calcOut;

    SendN2kEngineData(calc);

    Serial.print(mlppIn, 2);            //Prints the number calculated above
    Serial.println(" ml/P in");         //Prints "L/hour" and returns a  new line
    Serial.print(tmpMsElapsedIn, DEC);  //Prints the number calculated above
    Serial.println(" ms elapsed in");   //Prints "L/hour" and returns a  new line
    Serial.print(mlppOut, 2);           //Prints the number calculated above
    Serial.println(" ml/P out");        //Prints "L/hour" and returns a  new line
    Serial.print(tmpMsElapsedOut, DEC); //Prints the number calculated above
    Serial.println(" ms elapsed out");  //Prints "L/hour" and returns a  new line
    Serial.print(calc, 2);              //Prints the number calculated above
    Serial.println(" L/hour");          //Prints "L/hour" and returns a  new line
  }

  NMEA2000.ParseMessages();
}
