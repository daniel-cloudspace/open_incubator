/* FamiLAB incubator V0.2: Now with cooling capability!
This project arose out of the need for a 37˚C incubator for E.coli culturing (Biohacking).  
Additionally a peltier was used so that organisms that grow at lower temperature (20˚), could be cultured as well.
A L6203 heavyduty H-bridge was used to achieve the PWM switching, and also the switch the peltier from heating mode to cooling mode

PID code taken, with some modification, from "PID without a PhD" by Tim Wescott http://igor.chudov.com/manuals/Servo-Tuning/PID-without-a-PhD.pdf

TODO:
7-segment display output!
Verification of thermristor temp readings?

*/


//Define our variables
const int internalThermistor = 0; //Analog input attatched to the thermistor inside the incubator
const int externalThermistor = 1; //Analog input attatched to the thermistor sampling the ambient air temperature outside the box
const int inOne = 9; //Goes to logic input pin #1 for the L6203 H-bridge, if this is High and the inTwo is low, Out1 will source current and Out2 will sink it
const int inTwo = 10; //Goes to logic input pin #2 for the L6203 H-bridge, if this is High and the inOne is low, Out2 will source current and Out1 will sink it
const int enablePin = 11; //Goes to the enable pin, if this isn't brought high the chip won't do anything
double setPoint = 37; //Temperature setpoint, in degrees celcius
double error = 0;
double feedback = 0;
double tempReading = 0;
double tempReadingExternal = 0;
int sampleInverval = 1000; //Sampling interval in milliseconds


struct SPid {
double dState; // Last position input double 
double iState; // Integrator state 
double iMax; 
double iMin;   // Maximum and minimum allowable integrator state
double pGain;  // proportional gain
double iGain;  // integral gain
double dGain;  // derivative gain 

};

struct SPid thePID;

double UpdatePID(struct SPid *pid, double error, double position) {
double pTerm, dTerm, iTerm, feedback;
pTerm = pid->pGain * error;
// calculate the proportional term
// calculate the integral state with appropriate limiting
pid->iState += error;
if (pid->iState > pid->iMax) pid->iState = pid->iMax; else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
iTerm = pid->iGain * pid->iState * double(sampleInverval/1000); // calculate the integral term 
dTerm = pid->dGain * (position - pid->dState); pid->dState = position;

feedback = pTerm + iTerm - dTerm;
//Our analog write function only takes 0-255, so if the PID output is greater than 255 cap it.
if (feedback > 255) feedback = 255; if (feedback < -255) feedback = -255;

return feedback;

}

double thermistor_read(int RawADC) {
 double Temp;
 Temp = log(((10240000/RawADC) - 10000));
 Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
 Temp = Temp - 273.15;            // Convert Kelvin to Celcius
 return Temp;
}



void setup() {                
 //Begin serial and initialize our data values for thePID struct.
 Serial.begin(9600);
 thePID.pGain = 15;
 thePID.iGain = 1.5;
 thePID.dGain = 1;
 thePID.iMax = 160;
 thePID.iMin = -160;
 pinMode(inOne,OUTPUT);
 pinMode(inTwo,OUTPUT);
 pinMode(enablePin,OUTPUT);
 digitalWrite(enablePin,HIGH);
 digitalWrite(inOne,LOW);
 digitalWrite(inTwo,LOW);
 
 
    
}

void loop() {
  //Internal Thermistor 
  tempReading = thermistor_read(analogRead(0));
  tempReadingExternal = thermistor_read(analogRead(5));
  error = setPoint - tempReading;
  feedback = UpdatePID(&thePID,error,tempReading);
  //Serial.print("Interior Temp: ");
  Serial.println(tempReading);
  //Serial.print("Exterior Temp: ");
  //Serial.println(tempReadingExternal);
  //Serial.print("Feedback: ");
  //Serial.println(feedback);
  
  //If feedback is >= 0, then heat the box by PWM switching the peltier
  if (feedback >= 0)
  {
   analogWrite(inOne,feedback);
   analogWrite(inTwo,0);
  }
  else
  //If feedback is < 0, flip the current through the peltier so that its now cooling and PWM it the same way.
  {
   analogWrite(inOne,0);
   analogWrite(inTwo,-feedback); 
  }
  
  
  delay(sampleInverval);
        
}
