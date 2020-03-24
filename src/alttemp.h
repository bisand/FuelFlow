#include <Arduino.h>

struct TempData
{
    float Vin = 3.3;   // [V]
    float Rt = 10000;  // Resistor t [ohm]
    float R0 = 10000;  // value of rct in T0 [ohm]
    float T0 = 298.15; // use T0 in Kelvin [K]
    float Vout = 0.0;  // Vout in A0
    float Rout = 0.0;  // Rout in A0
    // use the datasheet to get this data.
    float T1 = 273.15; // [K] in datasheet 0º C
    float T2 = 343.15; // [K] in datasheet 70° C
    //float T2 = 373.15; // [K] in datasheet 100° C
    float RT1 = 26000; // [ohms]  resistence in T1
    float RT2 = 2245; // [ohms]   resistence in T2
    //float RT2 = 549.4; // [ohms]   resistence in T2
    float beta = 0.0;  // initial parameters [K]
    float Rinf = 0.0;  // initial parameters [ohm]
    float TempK = 0.0; // variable output
    float TempC = 0.0; // variable output
};

class AltTemp
{
  private:
    TempData _tempData;
    int _pin;

  public:
    AltTemp();
    ~AltTemp();

    void setup(int analogPin);
    float getTemperature();
};
