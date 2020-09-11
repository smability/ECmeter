#include <FreqCounter.h>
#include <math.h>
#include <EEPROM.h>


void setup() {
  Serial.begin(57600);                     // connect to the serial port
  Serial.println(" Water Level Sensor V1.0. July 20");
}

int STATE = 3;
int addr = 0;

long int frq;
float R1 = 4680;
float R2 = 99500;

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
float zero_level = 0.000000000669;          // 0% range is 669pf-675pf; 6pF change, Why?: Due to Temp and RH? How much per ºC and % (HR)? (dry probe). Dielectric Constant = insulation material PVC-55m cable
//float range = 0.000000002636;             // full-efective-sensor probe capacitance range-this value is assigned from calibration
float range;  

 
float CapFreeSpace;
float Warterlevel;
float CapCalibrated;
float CapCalibrated_;
float PVC_dielectric;

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
  
  max_level = EEPROM.get(addr,max_level);
  
  range = (max_level - zero_level);

  /*
  Serial.print(" Max Level: ");
  Serial.println(max_level,12); 
  Serial.print(" Zero Level: ");
  Serial.println(zero_level,12); 
  Serial.print(" Range: ");
  Serial.println(range,12);
  */
  
  }

//Sensor Probe length calculation
float SensorProbeLengthImmersed(){
  //max_level  only when the probe is 100% inmersed in warter, hence ProbelengthImmersed can be accurate
  //max_level_max one time calibration! (maximun capacitance), will dependt of the type, and charactersistcs, of water however
  float ProbelengthImmersed = (AverageCapacitance()-zero_level)/(((max_level/zero_level)-1.00)*((Pi*permFreeSp*((DielecConst())))/log(distCabls/(Radius))));
  return ProbelengthImmersed;
}

//Sensor Probe Raw Capacitance 
float SensorCapacitance(long int freq,float r1,float r2){
  float cap = 1.44/(freq*(r1 + 2*r2));
  return cap;
  }

//N-samples Sensor Probe Capacitance Average Reading
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

//Water Level Rading in % 
float WaterLevelReading(){
   Warterlevel = ((((AverageCapacitance()) - zero_level)/range))*100;                 //Water Level in %
   return Warterlevel;
}

//SerialOutput
void SerialMonitor(){
  
  Serial.print(" Water Level (%): ");
  Serial.print(WaterLevelReading(),2);
  Serial.print(" Probe Length Immersed : ");
  Serial.print(SensorProbeLengthImmersed(),2);

  }
 
// main loop
void loop() {

 //Frequency measurment
 FreqCounter::f_comp= 8;                    // Set compensation to 12
 FreqCounter::start(1000);                  // Start counting with gatetime of 100ms
 
 while (FreqCounter::f_ready == 0)          // wait until counter ready
 frq=FreqCounter::f_freq;                   // read result

 /* for debugging purposes
 CapCalibrated = SensorCapacitance(frq, R1, R2) + point_calibration;
 level = ((((SensorCapacitance(frq, R1, R2) + point_calibration-CapRef) - zero_level)/range))*100;
 float SensorCap = SensorCapacitance(frq, R1, R2) - CapRef;                        //Sensor Probe Raw Capacitance
 float SensorCap_cal = SensorCapacitance(frq, R1, R2) + point_calibration-CapRef;  //Ref capacitor of 1000pf 
 */

 
 /* Output Terminal/Plotter for debugging purposes
 Serial.print(" Frequency (Hz): ");
 Serial.print(frq);                         // print frequency
 
 Serial.print(" Capacitance Raw (F): ");
 Serial.print(SensorCap,12);
 
 Serial.print(" Capacitance calibrated (F): ");
 Serial.print(SensorCap_cal,12);
 
 */ 
 
 if(STATE == 0){
  
  int incomingByte = 0;
  
  //Serial.print("Calibrate Sensor Probe? Press 'd' for dry calibration or 'w' for wet calibration and 'n' for no calibration");
  
  Serial.print("Calibrate Sensor Probe? 'w' for wet calibration or 'n' for no calibration");
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
  
    if  ( incomingByte == 'w'){
      STATE = 1;
      delay(2);
    }
    /*
    else if (incomingByte == 'd'){
      STATE = 4;
      delay(2);
    }
    */
    else if (incomingByte == 'n'){
      STATE = 3;
      delay(2);
    } 
  }

 }
 
 else if (STATE == 1){
  
  int incomingByte_ = 0;
  
  Serial.println(" Immerse, complety in water, the sensor probe & press 'f' when done");
   if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte_ = Serial.read();
    if  ( incomingByte_ == 'f'){
      SensorProbeCalibrationWet();
      Serial.println(" Sensor probe, in water, is calibrated");
      STATE = 0;
      delay(2); 
    }
  }
 }
 
 /*
 else if(STATE == 2){
  SerialMonitor();
  delay(2);
 }
 */

 else if(STATE == 3){
  CurrentCalibration();
  SerialMonitor();
  delay(2);
 }

 else if(STATE == 4){
  
 int incomingByte_1 = 0;
 
  Serial.println(" Dry the sensor probe & press 'f' when done");
   if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte_1 = Serial.read();
    if  ( incomingByte_1 == 'f'){
      SensorProbeCalibrationDry();
      Serial.println(" Sensor probe, in air, is calibrated");
      STATE = 0;
      delay(2); 
    }
  }
 }
 
 /*
 Serial.print(" Water Level (%): ");
 Serial.print(WaterLevelReading(),1);
 
 Serial.print(" Probe Length Immersed : ");
 Serial.print(SensorProbeLengthImmersed(),2);
 */
 
 Serial.println();
 Serial.flush();

}
