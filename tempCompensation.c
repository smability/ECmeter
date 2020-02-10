#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 5 //Temperature input

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float Celcius = 0;   //Water temperature in ºC
float a = 0.019;     //Compensation factor uS/cm/ºC

//Electrode parameters:
int distance = 2;      //in cm
int area = 2.5;        //in cm2

//Input Frequency
//int Frequency = 100;  //in Hz

//Water Resistivity Function
float RawWaterRes(float Vin_avg,float avgVoltWater,float Rtest)
{
  /* Voltage divider
   *  
  */
  float RawResWater = Rtest*avgVoltWater/(Vin_avg-avgVoltWater);
  return RawResWater;
  }

//Raw conductivity function
float RawConduc(float WaterRes){
  /* Cell constant
  *  
  * K = d/a 
  * K = cell constant (cm-1)
  * 
  * a = effective area of the electrodes (cm2)
  * d = distance between the electrodes (cm)
  * 
  * Conductivity
  * 
  * κ = G*K
  * κ = conductivity (S/cm)
  * G = conductance (S), where G = 1/R
  * K = cell constant (cm-1)
  */
  float G;
  float Raw_k;
  int CellConstant; 
  CellConstant = distance/area; //cm/cm2=cm-1
  G = 1/WaterRes; //1/Ohm
  Raw_k = G*CellConstant; //ohm-1*cm-1
  return Raw_k; //uS/cm
  }

//Temperature compensation conductivity function  
float compConduc(float KT, float T){
  /* Temperatuere Compensated Conductivity
  * T is temperature in Celcius
  * a is the compensation coeficient 
  * KT is the raw conductivity (actual conductivity)
  * K25 is the compensated conducitivty
  * K25 = KT /(1 + a(T - 25))
  */
  float K25; //uS/cm
  K25 = KT /(1 + a*(T - 25)); //uS/cm
  return K25; ////uS/cm
}

void setup(void)
{
 
  Serial.begin(9600);
  sensors.begin();
}

void loop(void)
{
  /*Voltage inputs and system resitance*/
  float Vin_avg = 3.0; //Input Average Voltage
  float avgVoltWater = 0.273; //Input voltage from the water sample
  float Rtest = 10000.0;   //System Resistor = 10000 Ohm
  /*Water Resistivity, Water Conductivity and Water Compensated Conductivity*/
  
  float ResWater;
  float ConducWater;
  float Kcomp25;
  
  ResWater = RawWaterRes(Vin_avg,avgVoltWater,Rtest);
  ConducWater = RawConduc(ResWater);
  sensors.requestTemperatures();
  Celcius=sensors.getTempCByIndex(0);
  Kcomp25 = compConduc(ConducWater,Celcius);
  
  Serial.print(" ºC: ");
  Serial.print(Celcius);
  Serial.print(" - Raw ");
  Serial.print(" G (us/cm): ");
  Serial.print(ResWater);
  Serial.print(" - Compensated ");
  Serial.print(" G (us/cm): ");
  Serial.print(Kcomp25);
  Serial.print("\n");
  delay(1000);
}
