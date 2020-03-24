#include <Arduino.h>
#include "alttemp.h"
#include "RunningAverage.h"
#include "DHTesp.h"
#include "NMEA2000_CAN.h" // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "N2kMessages.h"
#include "AdminPortal.h"

// ---  Example of using PROGMEM to hold Product ID.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const tNMEA2000::tProductInformation ProductInformation PROGMEM = {
    1300,             // N2kVersion
    1001,             // Manufacturer's product code
    "Engine Monitor", // Manufacturer's Model ID
    "1.0.0",          // Manufacturer's Software version code
    "1.0.0",          // Manufacturer's Model version
    "00000001",       // Manufacturer's Model serial code
    0,                // SertificationLevel
    1                 // LoadEquivalency
};

// ---  Example of using PROGMEM to hold Configuration information.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const char ManufacturerInformation[] PROGMEM = "André Biseth, andre@biseth.net";
const char InstallationDescription1[] PROGMEM = "Engine Monitor";
const char InstallationDescription2[] PROGMEM = "Monitoring engine parameters.";

#define IS_DEBUG true
#define MAX_ELAPSED_MS 30000
#define MAX_ELAPSED_HALF_MS (MAX_ELAPSED_MS / 2)
#define MOVING_AVERAGE_COUNT 32

int flowInPin = 25;  //The pin location of the input sensor
int flowOutPin = 26; //The pin location of the output sensor
int rpmPin = 27;
int coolTempPin = 14;
int dhtPin = 13;

volatile unsigned int pulsesIn = 0;  //Pulse count to calibrate fuel flow.
volatile unsigned int pulsesOut = 0; //Pulse count to calibrate fuel flow.
volatile unsigned int rpmPulses = 0;
volatile unsigned long msLastIn = 0;                  // Last registered milliseconds from inbound interrupt.
volatile unsigned long msLastOut = 0;                 // Last registered milliseconds from outbound interrupt.
volatile unsigned long msElapsedIn = MAX_ELAPSED_MS;  // Milliseconds elapsed since last inbound interrupt.
volatile unsigned long msElapsedOut = MAX_ELAPSED_MS; // Milliseconds elapsed since last outbound interrupt.

unsigned int rpmDivisor = 75;
AltTemp altTemp;
DHTesp dht;
RunningAverage raTot(MOVING_AVERAGE_COUNT);
AdminPortal *adminPortal;

// Mutex used by interrupt
portMUX_TYPE muxRpm = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxIn = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE muxOut = portMUX_INITIALIZER_UNLOCKED;

/*
  Interrupt for the incoming fuel flow.
  This function is triggered on the falling edge of the hall effect sensors signal
*/
void IRAM_ATTR flowInInterrupt()
{
  unsigned long ms = millis();
  if ((ms - msLastIn) < 50)
    return;
  portENTER_CRITICAL_ISR(&muxIn);
  pulsesIn++;
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
  unsigned long ms = millis();
  if ((ms - msLastOut) < 50)
    return;
  portENTER_CRITICAL_ISR(&muxOut);
  pulsesOut++;
  msElapsedOut = ms - msLastOut;
  msLastOut = ms;
  portEXIT_CRITICAL_ISR(&muxOut);
}

/*
  Interrupt for the RPM signal.
  This function is triggered on the falling edge of the hall effect sensors signal
*/
void IRAM_ATTR rpmInterrupt()
{
  portENTER_CRITICAL_ISR(&muxRpm);
  rpmPulses++;
  portEXIT_CRITICAL_ISR(&muxRpm);
}

/*
  Send NMEA 2000 engine data out on the bus.
*/
void SendSlowN2kEngineData(double fuelRate, double coolTemp = 0)
{
  tN2kMsg N2kMsg;
  SetN2kEngineDynamicParam(N2kMsg, 0, 0, 0, coolTemp, 0, fuelRate, 0);
  NMEA2000.SendMsg(N2kMsg);
}

/*
  Send NMEA 2000 engine data out on the bus.
*/
void SendFastN2kEngineData(double rpm)
{
  tN2kMsg N2kMsg;
  SetN2kEngineParamRapid(N2kMsg, 0, rpm);
  NMEA2000.SendMsg(N2kMsg);
}

/*
  Send NMEA 2000 engine data out on the bus.
*/
void SendN2kTemperatureData(double temperature)
{
  tN2kMsg N2kMsg;
  SetN2kTemperature(N2kMsg, 1, 1, N2kts_EngineRoomTemperature, CToKelvin(temperature));
  NMEA2000.SendMsg(N2kMsg);
}

/*
  Calculates the flow in L/hr
*/
float calculateFlow(float mlpp, unsigned long elapsed)
{
  float calc = 0.0;
  calc = mlpp / (static_cast<float>(elapsed) / 1000.0); // mL/s
  calc = ((calc * 60.0) * 60.0) / 1000.0;               // L/hr
  return calc;
}

/*
  Adjusts the calculation if the duration between the pulses gets too high.
*/
float adjustCalculation(float calc, float mlpp, unsigned long elapsed)
{
  if (elapsed >= MAX_ELAPSED_MS)
    return 0.0;

  if (elapsed > MAX_ELAPSED_HALF_MS)
    return calculateFlow(mlpp, elapsed);

  return calc;
}

/*
  Reads the internal temperature sensor.
*/
bool getTemperature(double &temperature)
{
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0)
  {
    return false;
  }
  temperature = static_cast<double>(newValues.temperature);
  return true;
}

void logData(float tmp, unsigned long pulses, unsigned long elpsIn, unsigned long elpsOut, float flow)
{
  if (!IS_DEBUG)
    return;

  unsigned int ram = ESP.getFreeHeap();
  char buf[256];
  sprintf(buf, "%ld | %ld p | %ld ms in | %ld ms out | %f L/hr | %ld bytes", millis(), pulses, elpsIn, elpsOut, flow, ram);
  adminPortal->log("log_event", buf);

  sprintf(buf, "%ld", pulses);
  adminPortal->log("pulses", buf);
  sprintf(buf, "%ld", elpsIn);
  adminPortal->log("elapsedIn", buf);
  sprintf(buf, "%ld", elpsOut);
  adminPortal->log("elapsedOut", buf);
  sprintf(buf, "%f", flow);
  adminPortal->log("flow", buf);
  sprintf(buf, "%d", ram);
  adminPortal->log("ram", buf);
  sprintf(buf, "%ld", millis());
  adminPortal->log("millis", buf);
}

/*
  The setup
  initializes libraries and values.
*/
void setup()
{
  Serial.begin(115200);

  uint64_t chipid = ESP.getEfuseMac();
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String chipId = mac.substring(6);
  String clientId = "EM-" + chipId;
  WiFi.mode(WIFI_AP);
  
  IPAddress Ip(192, 168, 4, 1);
  IPAddress NMask(255, 255, 255, 0);
  WiFi.softAPConfig(Ip, Ip, NMask);
  WiFi.softAP(clientId.c_str(), mac.c_str());

  //Serial.println("Chip Id:      " + chipid);
  Serial.println("Access Point: " + clientId);
  Serial.println("Password:     " + mac);
  Serial.println("-----------------------------------------");
  Serial.println("IP address:   " + WiFi.softAPIP().toString());
  Serial.println("MAC:          " + mac);

  /*use mdns for host name resolution*/
  if (!MDNS.begin(clientId.c_str()))
  {
    Serial.println("Error setting up MDNS responder!");
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");

  adminPortal = new AdminPortal();
  adminPortal->setup();
  // if(adminPortal->formatSPIFFS())
  // {
  //   Serial.println("Successfully formatted SPIFFS.");
  // }

  // Initialize temperature sensors
  dht.setup(dhtPin, DHTesp::DHT11);
  altTemp.setup(coolTempPin);

  // Set Product information
  NMEA2000.SetProductInformation(&ProductInformation);
  // Set Configuration information
  NMEA2000.SetProgmemConfigurationInformation(ManufacturerInformation, InstallationDescription1, InstallationDescription2);
  // Set device information
  NMEA2000.SetDeviceInformation(10001, // Unique number. Use e.g. Serial number.
                                160,   // Device function. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                50,    // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                6765   // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  NMEA2000.SetDeviceInformation(10002, // Unique number. Use e.g. Serial number.
                                160,   // Device function. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75,    // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                6765   // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  NMEA2000.SetDeviceInformation(10003, // Unique number. Use e.g. Serial number.
                                130,   // Device function. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                90,    // Device class=Electrical Generation. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                6766   // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
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

  pinMode(flowInPin, INPUT);                                                     //initializes digital pin as an input
  attachInterrupt(digitalPinToInterrupt(flowInPin), flowInInterrupt, FALLING);   //and the interrupt is attached
  pinMode(flowOutPin, INPUT);                                                    //initializes digital pin as an input
  attachInterrupt(digitalPinToInterrupt(flowOutPin), flowOutInterrupt, FALLING); //and the interrupt is attached
  pinMode(rpmPin, INPUT);                                                        //initializes digital pin as an input
  attachInterrupt(digitalPinToInterrupt(rpmPin), rpmInterrupt, FALLING);         //and the interrupt is attached

  raTot.fillValue(0.0, MOVING_AVERAGE_COUNT);
}

unsigned long currMillis = 0;
unsigned long interval = 1000;
unsigned long currMillisTemp = 0;
unsigned long intervalTemp = 2500;
unsigned long currMillisRpm = 0;
unsigned long intervalRpm = 250;

float pulsesPerLiterIn = 190.0;             // Pulses per liter In
float pulsesPerLiterOut = 160.0;            // Pulses per liter Out
float mlppIn = 1000.0 / pulsesPerLiterIn;   // Milliliters per pulse = 5.2632
float mlppOut = 1000.0 / pulsesPerLiterOut; // Milliliters per pulse = 6,25

unsigned long tmpMsElapsedIn = MAX_ELAPSED_MS;
unsigned long tmpMsElapsedOut = MAX_ELAPSED_MS;
unsigned long lastPulsesIn = 0;
unsigned long lastPulsesOut = 0;

double temperature = 0;

unsigned long loopElapsedIn = 0;
unsigned long loopElapsedOut = 0;

/*
  The loop
  The continuously running loop
*/
void loop()
{
  // Do calculation if time has changed since last calculation.
  if (pulsesIn > lastPulsesIn || pulsesOut > lastPulsesOut || (millis() - currMillis > interval))
  {
    currMillis = millis();

    loopElapsedIn = 0;
    loopElapsedOut = 0;

    portENTER_CRITICAL(&muxIn);
    loopElapsedIn = currMillis - msLastIn;
    if (loopElapsedIn > MAX_ELAPSED_MS)
      msElapsedIn = MAX_ELAPSED_MS;
    tmpMsElapsedIn = msElapsedIn;
    lastPulsesIn = pulsesIn;
    portEXIT_CRITICAL(&muxIn);

    portENTER_CRITICAL(&muxOut);
    loopElapsedOut = currMillis - msLastOut;
    if (loopElapsedOut > MAX_ELAPSED_MS)
      msElapsedOut = MAX_ELAPSED_MS;
    tmpMsElapsedOut = msElapsedOut;
    lastPulsesOut = pulsesOut;
    portEXIT_CRITICAL(&muxOut);

    // Prevent division by zero.
    if (tmpMsElapsedIn < 1)
      tmpMsElapsedIn = 1;
    if (tmpMsElapsedOut < 1)
      tmpMsElapsedOut = 1;

    float calcIn = 0.0;
    float calcOut = 0.0;

    // Calculates flow by elapsed milliseconds. Works better on lower flow rates.
    calcIn = calculateFlow(mlppIn, tmpMsElapsedIn);
    calcIn = adjustCalculation(calcIn, mlppIn, loopElapsedIn);

    calcOut = calculateFlow(mlppOut, tmpMsElapsedOut);
    calcOut = adjustCalculation(calcOut, mlppOut, loopElapsedOut);

    // Calculating fuel flow based on input and output flow.
    float fuelFlow = calcIn - calcOut;
    raTot.addValue(fuelFlow);
    fuelFlow = raTot.getFastAverage();
    if (fuelFlow < 0)
      fuelFlow = 0;

    // Reads coolant temperature.
    float coolTemp = altTemp.getTemperature();

    SendSlowN2kEngineData(fuelFlow, coolTemp);

    logData(temperature, pulsesIn, tmpMsElapsedIn, tmpMsElapsedOut, fuelFlow);
  }

  if (millis() - currMillisTemp > intervalTemp)
  {
    currMillisTemp = millis();

    if (getTemperature(temperature))
    {
      SendN2kTemperatureData(temperature);
    }
  }

  if (millis() - currMillisRpm > intervalRpm)
  {
    if (rpmPulses < 1)
      return;

    currMillisRpm = millis();

    unsigned int tmpRpmPulses;
    portENTER_CRITICAL(&muxRpm);
    tmpRpmPulses = rpmPulses;
    rpmPulses = 0;
    portEXIT_CRITICAL(&muxRpm);
    double rpm = static_cast<double>((tmpRpmPulses * (1000 / intervalRpm) / rpmDivisor) * 60);
    SendFastN2kEngineData(rpm);
    if (IS_DEBUG && (millis() - currMillis > interval))
    {
      Serial.print(rpm, DEC);
      Serial.println(" RPM");
    }
  }

  adminPortal->loop();

  NMEA2000.ParseMessages();
}
