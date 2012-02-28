#include <PID_v1.h>
#define RelayPin 13


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);


int WindowSize = 5000;
unsigned long windowStartTime;



double thermistor_read(int RawADC) {
 double Temp;
 Temp = log(((10240000/RawADC) - 10000));
 Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
 Temp = Temp - 273.15;            // Convert Kelvin to Celcius
 Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
 return Temp;
}

void setup()
{
  Setpoint = 130;

  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC); 
 
  Serial.begin(9600);
}

void loop()
{
  Input = thermistor_read(analogRead(0));
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if(now - windowStartTime > WindowSize) { // time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  
  if(Output > now - windowStartTime) 
    digitalWrite(RelayPin,HIGH);
  else
    digitalWrite(RelayPin,LOW);
  
  Serial.println(Input);
  analogWrite(3,Output);
}

