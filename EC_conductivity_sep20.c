#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 5 //Temperature input
// Control Signal A
#define CONTROLA (11)
// Control Signal B
#define CONTROLB (13)

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup(void)
{
  Serial.begin(9600);
  sensors.begin();
  pins_init();
}

//port definition
void pins_init()
{
  pinMode(CONTROLA, OUTPUT);
  pinMode(CONTROLB, OUTPUT);

}

int analogPin0 = A0;
int analogPin1 = A1;
int analogPin3 = A3;

float Celcius = 0.0;        //Water temperature in ºC
float a = 0.019;           //Compensation factor in uS/cm/ºC

//Electrode parameters:
float distance = 1.2;     //in cm
float area = 0.9;         //in cm2 (total area of the two electrodes?)

//Input Frequency
//int Frequency = 100;    //in Hz
float Vdrop_Water;
float WaterResistance;
float WaterConductivity;

//Water Resistance 
float RawWaterImp(float Vin_rms_avg,float avg_rms_rc_Water,float Rtest)
{
  /* Voltage divider*/
  float RawImpWaterAvg;
  
  //float RawImpWater = (Rtest*avg_rms_rc_Water)/(Vin_rms_avg-avg_rms_rc_Water);
  //Rc=(Vdrop*R1)/(Vin-Vdrop);

  float sum = 0.0;
  int samples = 1;
  float RawImpWater[samples];
 
  for (int i = 0; i<samples; i++){

    RawImpWater[i] = (Rtest * (avg_rms_rc_Water))/(Vin_rms_avg - (avg_rms_rc_Water));
    sum = sum + RawImpWater[i];
  }

  RawImpWaterAvg = sum/samples;

  return RawImpWaterAvg;
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
  float CellConstant;
   
  CellConstant = (1/2.95); //in 1/cm
  
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
  * 
  */
  float K25; 
  K25 = KT /(1 + a*(T - 25.0)); 
  return K25; // in S/cm
}


float differential(int analog_A, int analog_B){
  
  int ValueA0 = analogRead (analog_A);  // read A0
  int ValueA1 = analogRead (analog_B);  // read A1
  float offset = 0.08;
  float diffV;

  float voltageV1 = (((ValueA0) * (5.0)) / 1024.0); // V1 voltage
  float voltageV2 = (((ValueA1) * (5.0)) / 1024.0); // V2 voltage
  
  //Vdrop= (((Vin)*(raw))/1024.0);

  diffV = (voltageV1) - (voltageV2); //power supply gives 4.92v
  
  return diffV;
  
  }

float temperature(){
  sensors.requestTemperatures();
  Celcius=sensors.getTempCByIndex(0);
  return Celcius;
}

/* H-bridge Control*/
// Control A "ON"
void ControlAON()
  {
  digitalWrite (CONTROLA, HIGH);  // turn CONTROL A ON
  //delay(1); // give time to power up (mosfet)
  }

// Control A "OFF"
void ControlAOFF()
  {
  digitalWrite (CONTROLA, LOW);  // turn CONTROL A OFF
  //delay(1); // give time to power down (mosfet)
  }

// Control B "ON"
void ControlBON()
  {
  digitalWrite (CONTROLB, HIGH);  // turn CONTROL B ON
  //delay(1); // give time to power up (mosfet)
  }

// Control B "OFF"
void ControlBOFF()
  {
  digitalWrite (CONTROLB, LOW);  // turn CONTROL B ON
  //delay(1); // give time to power down (mosfet)
  }

/* Main loop*/

void loop(void)
{
  //Injecting charge into water
  
  ControlAON();
  ControlBOFF();
  
  Vdrop_Water = differential(analogPin0, analogPin1);
  //Vdrop_Water = differential(analogPin0, analogPin1);
  
  //OFF (2.5 seconds)
  ControlAOFF();
  ControlBOFF();
  delay(2500);

  //Remove charge form the electrode and avoid water/electrode polarization
  
  ControlAOFF();
  ControlBON();
  //OFF (2.5 seconds)
  ControlAOFF();
  ControlBOFF();
  delay(2500);

  WaterResistance = RawWaterImp((5.0 - 0.08),Vdrop_Water,1000.0);
  //WaterResistance = RawWaterImp((5.0 - 0.08),Vdrop_Water,1000.0);
  
  if (WaterResistance < 300.0) //Low conductivity solutions
  {
     WaterResistance = WaterResistance*0.5;
     WaterConductivity = compConduc(RawConduc(WaterResistance), temperature());
    }
  else{ // Average conductivity solutions
    WaterResistance = WaterResistance-0.0;
    WaterConductivity = compConduc(RawConduc(WaterResistance), temperature());
  }
  
  //Report values
  Serial.print("           ");
  Serial.print( temperature(),2);
  Serial.print(" ºC           ");
  Serial.print(WaterResistance);
  Serial.print(" Ω            ");
  Serial.print(RawConduc(WaterResistance)*1000000,2);
  Serial.print(" µs/cm        ");
  Serial.print((WaterConductivity)*1000000,2);
  Serial.print(" µs/cm        ");
  Serial.println();
  
}

/* RMS value

float dt = 1; //sampling time = 1ms, integration time = samplingtime*20 = 20ms (integration-total-time)
int n = 1000; //number of samples
 
float RMS(float ds, int n, int analogA, int analogB){
  //n = 20, since dt = 1ms, period T = 20ms 
  
  int i = 1;
  float sum = 0; // reset sum every 20ms (20 samples)
      
    while (i<=n){
       
      sum = sum + sq(differential(analogA,analogB)*ds); //integrate as as fast as the loop time, then divide by 20 samples then wait for 1ms  
      i++; 
      //delay(ds); //dt=ds=delay=sampling; we are sampling 10 times the period of 20ms
    }
    
  float avgSig = (sum/n);//n=20 or 200, dt=1ms hece T = n*dt = 20ms or 200*1m=200ms "10 signals"
  
  float RMS_sig = sqrt(avgSig);
  
  return RMS_sig + 0.060;

  //return RMS_sig;
  
  }
*/

/* Signal RMS average; N samples
float AverageRMS(int analogA, int analogB){
  
  float RMSAvg;
  float sum;
  int samples = 10 ;
  float RMS_Vac[samples];
  
  sum = 0.0;
 
  for (int i = 0; i<samples; i++){

    RMS_Vac[i] = RMS(dt,n,analogA,analogB);
    sum = sum + RMS_Vac[i];
  }
  
  RMSAvg = sum/samples;
  return RMSAvg;
 }
*/

