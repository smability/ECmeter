#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 5 //Temperature input

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup(void)
{
  Serial.begin(9600);
  sensors.begin();
}

float Celcius = 0;   //Water temperature in ºC
float a = 0.019;     //Compensation factor in uS/cm/ºC

//Electrode parameters:
float distance = 1.2;      //in cm
float area = 1.02;        //in cm2

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
float RawConduc(float WaterRes, float d, float a){
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
  float CellConstant;
   
  CellConstant = d/a; //in 1/cm
  
  G = 1/(WaterRes); //in 1/Ohm.cm
  
  Raw_k = G*(CellConstant); //ohm.cm-1 = S/cm
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
  float K25; 
  K25 = KT /(1 + a*(T - 25.0)); 
  return K25; // in S/cm
}

float differential(){
  
  int ValueA0 = analogRead (A0);  // read A0
  int ValueA1 = analogRead (A1);  // read A1

  float voltageV1 = ValueA0 * (5.0 / 1023.0); // V1 voltage
  float voltageV2 = ValueA1 * (5.0 / 1023.0); // V2 voltage

  float diffV = (voltageV1 - voltageV2);
  
  return diffV;
  
  }

float dt = 1; //sampling time = 1ms, integration time = samplingtime*20 = 20ms (integration-total-time)
int n = 200; //number of samples
 
float RMS(float ds, int n){
  //n = 20, since dt = 1ms, period T = 20ms 
  
  int i = 1;
  float sum = 0; // reset sum every 20ms (20 samples)
      
    while (i<=n){
       
      sum = sum + sq(differential()*ds); //integrate as as fast as the loop time, then divide by 20 samples then wait for 1ms
      
      //delay(ds); //dt=ds=delay=sampling time
      
      i++; 
    }
    
  float avgSig = (sum/n);//n=20, dt=1ms hece T = n*dt = 20ms
  
  float RMS_sig = sqrt(avgSig);
  
  return RMS_sig;
  
  }

void loop(void)
{
  /*Voltage inputs and system resitance*/
  //float Vin_avg = 3.0 - 0.03; //Input Average Voltage from the H-bridge, 0.03v voltage loss due to cable resitance
  float Vin_avg = 3.444;
  //float avgVoltWater = 0.664; //Input voltage from the water sample (probablly the driff is because small-electrolysis)
  float avgVoltWater = RMS(dt,n); // RMS (dt,n), where dt is sampling time and n is number of samples
  float Rtest = 10000.0;   //System Resistor = 10000 Ohm
  /*Water Resistivity, Water Conductivity and Water Compensated Conductivity*/
  
  float ResWater;
  float ConducWater;
  float Kcomp25;
  
  ResWater = RawWaterRes(Vin_avg,avgVoltWater,Rtest);
  
  ConducWater = RawConduc(ResWater,distance,area);
  
  sensors.requestTemperatures();
  Celcius=sensors.getTempCByIndex(0);
  
  Kcomp25 = compConduc(ConducWater,Celcius);

  
  Serial.print(" RMS V: ");
  Serial.print(avgVoltWater,3);
  Serial.print(" ºC: ");
  Serial.print(Celcius);
  Serial.print(" - Resistivity (ohm.cm): ");
  Serial.print(ResWater);
  Serial.print(" - Raw Conductivity (s/cm): ");
  Serial.print(ConducWater,8);
  Serial.print(" - Conductivity at 25ºC (s/cm): ");
  Serial.print(Kcomp25,8);
  Serial.println();
  delay(1000);
}
