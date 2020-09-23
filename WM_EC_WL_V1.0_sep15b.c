#include <FreqCounter.h>
#include <math.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//PORT Definitions
#define ONE_WIRE_BUS 4 //Temperature input
// Control Signal A
#define CONTROLA (11)
// Control Signal B
#define CONTROLB (13)

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Setup; Run only once
void setup() {
  Serial.begin(57600);                     // connect to the serial port
  //Serial.begin(115200);                     // connect to the serial port
  sensors.begin();
  pins_init();
  Serial.println(" WaterMappers V1.0. Sep 20");
}

//port definition function
void pins_init()
{
  pinMode(CONTROLA, OUTPUT);
  pinMode(CONTROLB, OUTPUT);

}
/**** Initial STATE ******/
int STATE = 2;            //Start with the latest calibration dry-probe: zero_level = 0.000000000669 F and wet-probe: max_level = (from calibration function);
/******EC Sensor variables*******/
//analog readings H-Bridge
int analogPin0 = A0;
int analogPin1 = A1;
int analogPin3 = A3;

//Temperature and EC compensation
float Celcius = 0.0;      //Water temperature in ºC
float a = 0.019;          //Compensation factor in uS/cm/ºC

//Water resistance & Electrode parameters:
float distance = 1.2;     //in cm
float area = 0.9;         //in cm2 (total area of the two electrodes?)

float Vdrop_Water;        //Water voltage drop
float WaterResistance;    //Water Resistance
float WaterConductivity;  //Water EC

/******Water Level Sensor variables*******/
int addr = 0;             //memory address used to store calibration data--only wet calibration

long int frq;             //frequency counter variable
float R1 = 4680;          //555, R1
float R2 = 99500;         //555, R2

const float distCabls = 0.002;              // Distance between two cables,in m, 2mm
const float Radius = 0.0004064;             // Cable Radious, in m, 0.4mm
const float permFreeSp = 0.000000000008854; // permitivity of free space
const float Pi = 3.14159;                   // Pi
const float CapRef = 0.000000001000;        // 1000pf Reference Capacitor
const float SensorProbeLength = 15.0;       // Sensor Probe-total-cable length

float point_calibration = 0.000000000037;   // 37pf offset; from a 1000pf reference capacitor-this value is assigned from calibration 
//float max_level = 0.000000004953;         // 4953pf at 100% (water), Sensor Probe capacitance, Dielectric Contantant = insultation meterial (PVC) + Water
float max_level;
//float zero_level;
float zero_level = 0.000000000675;          // 0% range is 658pf-675pf; 6pF change, Why?: Due to Temp and RH? How much per ºC and % (HR)? (dry probe). Dielectric Constant = insulation material PVC-55m cable
//float range = 0.000000002636;             // full-efective-sensor probe capacitance range-this value is assigned from calibration
float range;  

float CapFreeSpace;                         //Free Space Capacintance
float Warterlevel;                          //Water level in %
float CapCalibrated;                        //Capacitance calibrated 1
float CapCalibrated_;                       //Capacitance calibrated 2
float PVC_dielectric;                       //PVC Dielectric (cable insulation material) 

/******EC sensor functions*******/
//Water Resistance Function
float RawWaterImp(float Vin_rms_avg,float avg_rms_rc_Water,float Rtest)
{
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

//Raw Electrical Conductivity Function
float RawConduc(float WaterRes){
  /*Cell constant
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
  * 
  */
  
  float G;
  float Raw_k;
  float CellConstant;
   
  CellConstant = (1/2.95); //in 1/cm  //cell constant should be retrieved from a calibration routine using a standard solution
  
  G = 1/(WaterRes); //in 1/Ohm.cm
  
  Raw_k = G*(CellConstant); //ohm.cm-1 = S/cm
  return Raw_k; //uS/cm
  
  }

//Compensated Electrical Conductivity Function  
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

//Water Resistance Voltage Drop Measurment Function
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
  
//Temperature Reading Function
float temperature(){
  sensors.requestTemperatures();
  Celcius=sensors.getTempCByIndex(0);
  return Celcius;
}

/******H-bridge Control for the EC sensor-Chage Injection/Removal*******/
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

/******Water Level Functions*******/

//Sensor Probe Capacitance in Free-Space (no insulation material)
float ProbCapacitanceFreeSpace(float probelength){                       
  CapFreeSpace = (Pi*permFreeSp*probelength)/log(distCabls/(Radius));
  return CapFreeSpace;
  }

//Calculation of Dielectric Constant-Realative Permitivity- of thesensor probe insulation material (PVC) 
float DielecConst(){                       
  float Dielectric = zero_level/ProbCapacitanceFreeSpace(SensorProbeLength);
  return Dielectric;
  }

//Sensor Probe Calibration Function, wet-probe (max_level)
float SensorProbeCalibrationWet(){
    
  max_level = AverageCapacitance();
  Serial.println(addr);
  //Serial.println(addr);
  //range = (max_level - zero_level);
  
  //Write the value to the appropriate byte of the EEPROM
  EEPROM.put(addr, max_level); 
 
  Serial.println(max_level,12);
  //addr = 0;
  //Serial.println(range,12);
  
  }

//Sensor Probe Calibration Function, dry-probe (zero_level)
float SensorProbeCalibrationDry(){

  zero_level = AverageCapacitance();
  Serial.println(addr); 
  //range = (max_level - zero_level);
  //Write the value to the appropriate byte of the EEPROM
  EEPROM.put(addr, zero_level);
  addr = addr + 1;
  Serial.println(EEPROM.get(addr,zero_level),12); 
  //Serial.println(range,12);
  }

void CurrentCalibration(){

  int addr = 0;
  max_level = EEPROM.get(addr,max_level); //Retrive max_level from wet-sensor calibration variable
  range = (max_level - zero_level);
  
  //Serial.print(" Max Level: ");
  //Serial.println(max_level,12); 
  //Serial.print(" Zero Level: ");
  //Serial.println(zero_level,12); 
  //Serial.print(" Range: ");
  //Serial.println(range,12);

  }

//Sensor Probe Length Calculation Function
float SensorProbeLengthImmersed(){
  //max_level  only when the probe is 100% inmersed in warter, hence ProbelengthImmersed can be accurate
  //max_level_max one time calibration! (maximun capacitance), will dependt of the type, and charactersistcs, of water however
  float ProbelengthImmersed = (AverageCapacitance()-zero_level)/(((max_level/zero_level)-1.00)*((Pi*permFreeSp*((DielecConst())))/log(distCabls/(Radius))));
  return ProbelengthImmersed;
}

//Sensor Probe Raw Capacitance Function
float SensorCapacitance(long int freq,float r1,float r2){
  float cap = 1.44/(freq*(r1 + 2*r2));
  return cap;
  }

//N-samples Sensor Probe Capacitance Average Reading Function
 float AverageCapacitance(){
  
  float CapacitanteAvg;
  float sum;
  int i;
  int samples = 10 ;
  float CapCal[samples];
  
  sum = 0.0;
 
  for (i = 0; i<samples; i++){
    delay(5);
    CapCal[i] = SensorCapacitance(frq, R1, R2) + point_calibration-CapRef;
    sum = sum + CapCal[i];
  }
  
  CapacitanteAvg = sum/samples;
  return CapacitanteAvg;
 }

//Water Level Reading (in %) Function
float WaterLevelReading(){
   Warterlevel = ((((AverageCapacitance()) - zero_level)/range))*100;                 //Water Level in %
   return Warterlevel;
}

//Serial Monitor Output
void SerialMonitor(){
  /*Water Level Sensor Readings*/
  Serial.print(WaterLevelReading(),2);
  Serial.print(" %           ");
  Serial.print(SensorProbeLengthImmersed(),2);
  Serial.print(" m           ");
  /*EC Sensor Readinds*/
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


void FrequencyCounter(){
  //Frequency measurment
  FreqCounter::f_comp= 8;                    // Set compensation to 12
  FreqCounter::start(1000);                  // Start counting with gatetime of 100ms
 
  while (FreqCounter::f_ready == 0)          // wait until counter ready
  frq=FreqCounter::f_freq;                   // read result
}
 
/****** main loop*******/
void loop() {
 
 
 //Frequency measurment
 FreqCounter::f_comp= 8;                    // Set compensation to 12
 FreqCounter::start(1000);                  // Start counting with gatetime of 100ms
 
 while (FreqCounter::f_ready == 0)          // wait until counter ready
 frq=FreqCounter::f_freq;                   // read result
 
 
 //FrequencyCounter();                        //Frequency counter library/function, used for the WaterLevel Sensor, it should run continously inside the main loop
 
 /**** Water Level Sensor Calibration & Reading SATES ****/
 if( STATE == 0 ){
  
  int incomingByte = 0;
  
  //Serial.print("Calibrate Sensor Probe? Press 'd' for dry calibration or 'w' for wet calibration and 'n' for no calibration");
  
  Serial.print("Calibrate Sensor Probe? 'w' for wet calibration or 'n' for no calibration"); //press 'w' if it is the first time 
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
  
    if  ( incomingByte == 'w'){
      STATE = 1;                  // wet calibration (SATE 1)
      delay(2);
    }
    else if (incomingByte == 'd'){ // dry calibration (STATE 3 hasn't been implemented)
      STATE = 3;
      delay(2);
    }
    
    else if (incomingByte == 'n'){ //Jump to SATE 2 to start sensor readings
      STATE = 2;
      delay(2);
    } 
  }

 }
 
 else if ( STATE == 1 ){
  
  int incomingByte_ = 0;
  Serial.println(" Immerse the sensor probe complety in water & press 'f' when done");
   if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte_ = Serial.read();
    if  ( incomingByte_ == 'f'){
      SensorProbeCalibrationWet(); //Record Max Value (Wet Calibration Function)
      Serial.println(" Wet Sensor probe calibrated");
      STATE = 0;
      delay(2); 
    }
  }
 }
 
 else if( STATE == 2 ){
  
  CurrentCalibration(); //Use current wet calibration
  //SerialMonitor();
  STATE = 4;
  delay(2);
 }
 
 else if( STATE == 3 ){ //dry calibration (STATE 3 hasn't been implemented)
  
  int incomingByte_1 = 0;
  Serial.println(" Dry the sensor probe & press 'f' when done");
   if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte_1 = Serial.read();
    if  ( incomingByte_1 == 'f'){
      SensorProbeCalibrationDry(); //Dry calibration Function
      Serial.println(" Dry Sensor probe calibrated");
      STATE = 0;
      delay(2); 
    }
  }
 }

 /**** EC Sensor Reading SATE ****/
 else if( STATE == 4 ){
  
  //Inject charge 
  ControlAON();
  ControlBOFF();

  Vdrop_Water = differential(analogPin0, analogPin1);
  //Vdrop_Water = differential(analogPin0, analogPin1);
  
  ControlAOFF();
  ControlBOFF();
  
  delay(2500); //OFF (2.5 seconds)

  //Remove charge form the electrode and avoid water/electrode polarization
  ControlAOFF();
  ControlBON();
  
  //OFF (2.5 seconds)
  ControlAOFF();
  ControlBOFF();
  
  delay(2500);

  //WaterResistance = RawWaterImp((5.0 - 0.08),Vdrop_Water,1000.0);
  WaterResistance = RawWaterImp((5.0 - 0.00),Vdrop_Water,1000.0);
  
  if (WaterResistance < 300.0) //Low conductivity solutions
  {
     WaterResistance = WaterResistance*0.5;
     WaterConductivity = compConduc(RawConduc(WaterResistance), temperature());
     
    }
    
  else{                        // Average conductivity solutions
    //WaterResistance = WaterResistance-00.0;
    WaterResistance = WaterResistance*0.86;
    WaterConductivity = compConduc(RawConduc(WaterResistance), temperature());
    
  }
  STATE = 5; // go to Serial Monitor STATE
  
 }

 else if( STATE == 5){

  SerialMonitor();
  //Serial.println();
  Serial.flush();
  STATE = 2; //Start all over
  
 }

}
