#include <FreqCounter.h>

void setup() {
  Serial.begin(57600);                    // connect to the serial port
  Serial.println("Frequency Counter");
}

long int frq;
float R1 = 4680;
float R2 = 99500;

float CapAir = 0.0000000000031; //A = 0.0007m2-calculated; 
float CapAir_msrd = 0.000000000037; //10x more than the calculated value
float CapWater = 0.000000000860; //max capacitance; part of the calibration process
float distancePlates = 0.002;
float base = 0.01;

float permFreeSp = 0.000000000008854;
float Er = CapWater/CapAir; //at 100% submerged-part of the calibration process

//Sensor Length
float SensorLength(float capWater, float capAir){

 float lengthSensor = (capWater*distancePlates)/(permFreeSp*Er*base);
 return lengthSensor;
}
//Sensor capacitance
float SensorCapacitance(long int freq,float r1,float r2){
  float cap = 1.44/(freq*(r1 + 2*r2));
  return cap;
  }
  
void loop() {
 float level;
 float point_calibration = 0.000000000028;  //from a 470pf reference capacitor
 float max_level = 0.000000000860;          //fine tune this parameter
 float zero_level = 0.000000000063;         //65pf at 0% water level (air)
 float range = 0.000000000797;              //full-efective-capacitance range (submerged)
 float lengthcorrection = 0.5; 
 
 float probelength;
 float CapCalibrated;
 CapCalibrated = SensorCapacitance(frq, R1, R2) + (point_calibration);
 
 FreqCounter::f_comp= 8;                    // Set compensation to 12
 FreqCounter::start(1000);                  // Start counting with gatetime of 100ms
 
 while (FreqCounter::f_ready == 0)          // wait until counter ready
 frq=FreqCounter::f_freq;                   // read result
 
 probelength = SensorLength(CapCalibrated,CapAir)*100 - lengthcorrection;
  
 
 level = ((((SensorCapacitance(frq, R1, R2) + point_calibration) - zero_level)/range))*100;
 float SensorCap = SensorCapacitance(frq, R1, R2);
 float SensorCap_cal = SensorCapacitance(frq, R1, R2) + point_calibration;
 
 Serial.print(" Frequency (Hz): ");
 Serial.print(frq);                         // print frequency
 Serial.print(" Capacitance raw (F): ");
 Serial.print(SensorCap,12);
 Serial.print(" Capacitance calibrated (F): ");
 Serial.print(SensorCap_cal,12);
 Serial.print(" Water Level (%): ");
 Serial.print(level,12);
 
 Serial.print(" Relative Permitivity: ");
 Serial.print(Er,3);
 Serial.print(" Probe length submerged (cm): ");
 Serial.print(probelength,12);
 
 Serial.println();
 delay(20);
}
