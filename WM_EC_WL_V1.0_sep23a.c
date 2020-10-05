// Serial library
#include <SoftwareSerial.h>
SoftwareSerial Serial2(8,9); //SIM808 (8: Rx, 9:Tx)
//Sleep mode libraries
#include "Arduino.h"
// avr - Version: Latest
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
//Sensor Libraries
#include <FreqCounter.h>
#include <math.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//Sensor PORT Definitions
#define ONE_WIRE_BUS 4 //Temperature input
//Load & Power Key Controls
// Control Signal A
#define CONTROLA (11)
// Control Signal B
#define CONTROLB (13)
//Temperature Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Power SIM
#define POWERSIM (2) //move D5 to D2
// Power Switch
#define POWERSW (6)
// Power Sensors
#define POWER (7)
// Analog Port
#define LEVEL_SENSOR A5


//Sensor readings are storage in a char array
char BattVal[] = "00.00";
char tempVal[] = "00.00";
char distanceVal[] = "00.00";
char WaterLevelVal[] = "00.00";
char EleCondVal[] = "000.00";
char CompEleCondVal[] = "000.00";

unsigned long InitialTime;

void setup()  {
    //Serial.begin(9600);
    Serial.begin(57600);                       // connect to the serial port, 57600 is used for the frequency counter
    Serial2.begin(9600);                       // SIM8008 baud rate
   
    // Set PIN modes
    pins_init();                              //pin I/O set-up
    Serial.println("Initialising..."); 
    delay(100);
    setupWatchDogTimer();
    InitialTime = millis();                   //record initial Time
    Serial.print("Initial Time in Seconds: ");
    Serial.println((InitialTime)/1000);
    Serial.println("Initialisation complete.");
    delay(100);
    
    sensors.begin();                           //Start Temp Sensor
    //pins_init();                             //pin I/O set-up
    Serial.println("          Water Mappers V1.0. Sep 2020          ");
}

void pins_init()  {
  pinMode(LEVEL_SENSOR, INPUT);
  pinMode(POWER, OUTPUT);
  pinMode(POWERSIM, OUTPUT);
  pinMode(POWERSW, OUTPUT);
  //H-bridge control signals
  pinMode(CONTROLA, OUTPUT);
  pinMode(CONTROLB, OUTPUT);
}

//variables used during the SIM_GPRS function
float voltage; // where?
float batt;
char aux_str[270];

/******WATER LEVEL-EC SENSORS ******/
// Initial STATE 
int STATE = 2;            //Start with the latest calibration dry-probe: zero_level = 0.000000000675 F and wet-probe: max_level = (from calibration function);

//EC Sensor variables
//analog readings H-Bridge
int analogPin0 = A0;
int analogPin1 = A1;
int analogPin3 = A3;

//Temperature and EC compensation
float Celcius = 0.0;      //Water temperature in ºC
float a = 0.019;          //Compensation factor in uS/cm/ºC

//Electrode parameters:
float distance = 1.2;     //in cm
float area = 0.9;         //in cm2 (total area of the two electrodes?)
//Water resistance 
float Vdrop_Water;        //Water voltage drop
float WaterResistance;    //Water Resistance
float WaterConductivity;  //Water EC

//Water Level Sensor variables (freqency calculation)
int addr = 0;             //memory address used to store calibration data--only wet calibration
long int frq;             //frequency counter variable
float R1 = 4680;          //555, R1
float R2 = 99500;         //555, R2

/******Water Level Sensor variables (capacitance calculation)******/
const float distCabls = 0.002;              // Distance between two cables,in m, 2mm
const float Radius = 0.0004064;             // Cable Radious, in m, 0.4mm
const float permFreeSp = 0.000000000008854; // permitivity of free space
const float Pi = 3.14159;                   // Pi
const float CapRef = 0.000000001000;        // 1000pf Reference Capacitor
const float SensorProbeLength = 15.0;       // Sensor Probe-total-cable length

//Water Level Sensor variables (calibration and sensor range calculation)
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

/****** EC SENSOR FUNCTIONS ******/
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

/*****H-BRIDGE CONTROL FOR THE EC SENSOR (CHARGE INJECTION/REMOVAL*****/
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

/****WATER LEVEL FUNCTIONS*****/
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

//Serial Monitor Output EC and WL sensir
void SerialMonitor(){
  /*Water Level Sensor Readings*/
  Serial.print(WaterLevelReading(),2);
  Serial.print(" %           ");
  Serial.print(SensorProbeLengthImmersed(),2);
  Serial.print(" m           ");
  /*EC Sensor Readinds*/
  Serial.print("           ");
  Serial.print(temperature(),2);
  Serial.print(" ºC           ");
  Serial.print(WaterResistance);
  Serial.print(" Ω            ");
  Serial.print(RawConduc(WaterResistance)*1000000,2);
  Serial.print(" µs/cm        ");
  Serial.print((WaterConductivity)*1000000,2);
  Serial.print(" µs/cm        ");
  Serial.println();
}

/**** SLEEP MODE FUNCTION *******/
// This variable is made volatile because it is changed inside an interrupt function
volatile int f_wdt=1;
int count;
// Watchdog Interrupt Service. This is executed when watchdog timed out.
ISR(WDT_vect) {
  if(f_wdt == 0) {
    // here we can implement a counter the can set the f_wdt to true if
    // the watchdog cycle needs to run longer than the maximum of eight
    // seconds.
    //int count; otherwise, we are initializing this variable, to 0, every time ISR is called
    //Serial.println(count);
    if(count == 15){//150 is a 4hrs delay
      f_wdt=1;
      count=0;
    }
    else
      count++;
  }
}

// Enters the arduino into sleep mode.
void enterSleep(void) {
  // There are five different sleep modes in order of power saving:
  // SLEEP_MODE_IDLE - the lowest power saving mode
  // SLEEP_MODE_ADC
  // SLEEP_MODE_PWR_SAVE
  // SLEEP_MODE_STANDBY
  // SLEEP_MODE_PWR_DOWN - the highest power saving mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  // Now enter sleep mode.
  sleep_mode();
  // The program will continue from here after the WDT timeout
  // First thing to do is disable sleep.
  sleep_disable();
  // Re-enable the peripherals.
  power_all_enable();

  memset(aux_str, '\0', 270); // why?
}

// Setup the Watch Dog Timer (WDT)
void setupWatchDogTimer() {

  // The MCU Status Register (MCUSR) is used to tell the cause of the last
  // reset, such as brown-out reset, watchdog reset, etc.
  // NOTE: for security reasons, there is a timed sequence for clearing the
  // WDE and changing the time-out configuration. If you don't use this
  // sequence properly, you'll get unexpected results.

  // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
  MCUSR &= ~(1<<WDRF);

  // Configure the Watchdog timer Control Register (WDTCSR)
  // The WDTCSR is used for configuring the time-out, mode of operation, etc

  // In order to change WDE or the pre-scaler, we need to set WDCE (This will
  // allow updates for 4 clock cycles).

  // Set the WDCE bit (bit 4) and the WDE bit (bit 3) of the WDTCSR. The WDCE
  // bit must be set in order to change WDE or the watchdog pre-scalers.
  // Setting the WDCE bit will allow updates to the pre-scalers and WDE for 4
  // clock cycles then it will be reset by hardware.
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /**
   *  Setting the watchdog pre-scaler value with VCC = 5.0V and 16mHZ
   *  WDP3 WDP2 WDP1 WDP0 | Number of WDT | Typical Time-out at Oscillator Cycles
   *  0    0    0    0    |   2K cycles   | 16 ms
   *  0    0    0    1    |   4K cycles   | 32 ms
   *  0    0    1    0    |   8K cycles   | 64 ms
   *  0    0    1    1    |  16K cycles   | 0.125 s
   *  0    1    0    0    |  32K cycles   | 0.25 s
   *  0    1    0    1    |  64K cycles   | 0.5 s
   *  0    1    1    0    |  128K cycles  | 1.0 s
   *  0    1    1    1    |  256K cycles  | 2.0 s
   *  1    0    0    0    |  512K cycles  | 4.0 s
   *  1    0    0    1    | 1024K cycles  | 8.0 s
  */
  WDTCSR  = (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
  // Enable the WD interrupt (note: no reset).
  WDTCSR |= _BV(WDIE);
}

//Maybe these functions are redundant
float getTemp(){
  float Temperature = temperature();
  return Temperature;
  }

float getDistance(){
  float distance = SensorProbeLengthImmersed();
  return distance;
  }

float getWaterLevel(){
  float waterlevel = WaterLevelReading();
  return waterlevel;
  }

float getElectCond(){
  float EleCon = RawConduc(WaterResistance)*1000000;
  return EleCon;
  }

float getCompElectCond(){
  float CompEleCon = (WaterConductivity)*1000000;
  return CompEleCon;
  }

// Get raw sensor value to measure battery value
int getMaxValue() {
    int sensorValue;

    int sensorMax = 0;

    uint32_t start_time = millis();

    while ((millis()-start_time)<100) // sensor reading gets attenuated if sample <= 10ms, WHY?
    {
      sensorValue = analogRead(LEVEL_SENSOR);

      if(sensorValue > sensorMax)
      {
        sensorMax = sensorValue;
      }
    }
    return sensorMax;
}

// Power ON all Sensors
void powerONSensor()  {
  digitalWrite (POWER, LOW);  // turn power ON
  delay(1); // give time to power up
}  // end of powerONSensor

// Power OFF hall sensor
void powerOFFSensor() {
  digitalWrite (POWER, HIGH);  // turn power OFF
  delay(1); // give time to power down
}  // end of powerOFFSensor

// Power ON Boost
void powerONBoost() {
  //digitalWrite (POWERSW, LOW);  // turn power ON
  digitalWrite (POWERSW, HIGH);  // turn power ON
  delay(1); // give time to power up
}  // end of powerON SIM

// Power OFF Boost
void powerOFFBoost()  {
  //digitalWrite (POWERSW, HIGH);  // turn power OFF
  digitalWrite (POWERSW, LOW);  // turn power OFF
  delay(1); // give time to power down
}  // end of powerOFF SIM

// Power ON SIM
void powerONSIM() {
  digitalWrite (POWERSIM, HIGH);  // turn transistor ON
  delay(1); // give time to power up
}  // end of powerON SIM

// Power OFF SIM
void powerOFFSIM()  {
  digitalWrite (POWERSIM, LOW);  // turn transistor OFF
  delay(1); // give time to power down
}  // end of powerOFF SIM

//Sensor readings
void SENSOR_READINGS(){
  
  //Frequency measurment
 FreqCounter::f_comp= 8;                    // Set compensation to 12
 FreqCounter::start(1000);                  // Start counting with gatetime of 100ms
 
 while (FreqCounter::f_ready == 0)          // wait until counter ready
 frq=FreqCounter::f_freq;                   // read result
 
 
 //FrequencyCounter();                        //Frequency counter library/function, used for the WaterLevel Sensor, it should run continously inside the main loop
 
 // Water Level Sensor Calibration & Reading SATES 
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

 // EC Sensor Reading SATE 
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

//SIM states
enum SIM
    {
      START,SENSOR,
      PWRSIM,ATTACHGPRS,
      OPENGPRS,QUERYGPRS,
      INITHTTP,SENDPARA,
      ADDLEVEL,CLOSEURL,
      ENDPARA,ENDGPRS,
      SIMOFF
    };

SIM SIM_state  = START;

//uint32_t stateTime;

void SIM_GPRS(){
  
  int countloop=0; //why?
  
  char variable;

  switch (SIM_state)
  {
    case START:

      if(f_wdt != 1){  //Wait here indefenetly until a flag is rised; TRUE if f_wdt = 0
        //SIM_state = START;
        //break;
        return;
      }

        SIM_state = SENSOR; // wake-up if f_wdt = 1
      break;

    //Get readings from four sensors
    case SENSOR:
      
      InitialTime = millis();
      Serial.println ("Intial Time: ");
      Serial.println(InitialTime);
      
      Serial.println(F("      Case Sensor     "));
      
      int sensor_max;
      
      memset(BattVal, '\0', sizeof(BattVal)); // why??
      
      // Sensors ON
      powerONSensor();
      // Sensor Readings
     
      sensor_max = getMaxValue();
      voltage = (sensor_max / 1024.0) * 5000; // Gets you voltage in mV
      batt = (voltage/5000) * 100 ; // Get you level in %

      SENSOR_READINGS();
      
      getTemp(); //from function temp
      getDistance(); //from function distance
      getElectCond() ; //from funcion Electrical Conductivity
      getCompElectCond(); //from function Comp Elect Conduc
      getWaterLevel(); //from function WaterLevel
      
 
      Serial.println(F("      Sensor Readings:      "));
      Serial.println(batt);
      Serial.println(getTemp());
      Serial.println(getWaterLevel());
      Serial.println(getDistance());
      Serial.println(getElectCond());
      Serial.println(getCompElectCond());
      
      //float to a char array conversion
      dtostrf(batt, 2, 0, BattVal);
      dtostrf(getTemp(), 2, 2, tempVal);
      dtostrf(getWaterLevel(), 3, 2, WaterLevelVal);
      dtostrf(getDistance(), 3, 2, distanceVal);
      dtostrf(getElectCond(), 3, 2, EleCondVal);
      dtostrf(getCompElectCond(), 3, 2, CompEleCondVal);
      
      
      /* debugging sensor readings as char
      Serial.print(F("---> Sensor Readings (as char): "));
      for(int i=0; i<sizeof(charVal); i++){
        Serial.print(charVal[i]);
        Serial.print(tempVal[i]);
        Serial.print(WaterLevelVal[i]);
        Serial.print(distanceVal[i]);
        Serial.print(EleCondVal[i]);
        Serial.print(CompEleCondVal[i]);
      }
      
      Serial.println();
      */
      
      // wait, certain amount of time, then shut OFF all sensors
      delay(100);
      
      // Sensors OFF
      powerOFFSensor();
      
      SIM_state = PWRSIM;
      
      break;

    case PWRSIM:
    
      Serial.println(F("      Case PWRSIM     "));
        //Power Boost Switch ON
        powerONBoost();
        //PowerSIM ON
        powerONSIM(); //High pulse 1.2 seconds, makes Vc = 0 for 1.2 seconds
        delay(1200);
        powerOFFSIM(); // Vc=VBAT

        sendATcommand("AT+CREG=1", "OK", 200);  // Activating CREG notifications

        countloop=0;
        while( (sendATcommand("AT+CREG?", "+CREG: 1,1", 600) // before 0,1
              || sendATcommand("AT+CREG?", "+CREG: 0,5", 600)
              || sendATcommand("AT+CREG?", "+CREG: 0,1", 600)
              || sendATcommand("AT+CREG?", "+CREG: 2,1", 600)) == 0 ){
                countloop++;
                if(countloop >= 8){
                  break;
                }
              };

        sendATcommand("AT+CREG=0", "OK", 5000);

       if(countloop < 8){
         SIM_state = ATTACHGPRS;
         Serial.println(F("Connected to the network!"));
       }
       else {
         SIM_state = PWRSIM;
         Serial.println(F("NOT Connected to the network!"));
       }

       break;

    case ATTACHGPRS:
    
      memset(aux_str, '\0', 270);
    
      Serial.println(F("      Case ATTACHGPRS     "));
      
      sendATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2000);
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"", "internet.itelcel.com");
      sendATcommand(aux_str, "OK", 2000);
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"USER\",\"%s\"", "webgprs");
      sendATcommand(aux_str, "OK", 2000);
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"PWD\",\"%s\"", "webgprs2002");
      sendATcommand(aux_str, "OK", 2000);

      countloop=0;

      while ( (  sendATcommand("AT+SAPBR=1,1", "OK", 6000)
              || sendATcommand("AT+SAPBR=2,1", "+SAPBR: 1,1,\"", 6000) ) != 1){
               countloop++;
               if(countloop >= 10){
                 break;
               }
             };
      if(countloop < 5){
        SIM_state = INITHTTP;
        Serial.println(F("Connected to the Internet!"));
      }
      else {
        SIM_state = ATTACHGPRS;
        Serial.println(F("NOT Connected to the Internet!"));
      }
      break;

    case INITHTTP:
      Serial.println(F("----- Case INITHTTP -----"));
      countloop=0;
      while (sendATcommand("AT+HTTPINIT", "OK", 1000) != 1)
      {
        countloop++;
        if(countloop >= 3){
          break;
        }
      }; //why (;) ?
      delay(100);
      sendATcommand("AT+HTTPPARA=\"CID\",1", "OK", 10000);

      if(countloop < 3){
        SIM_state = SENDPARA;
        Serial.println(F("Init Http functional!"));
      }
        else {
        SIM_state =  INITHTTP;
        Serial.println(F("Init Http NOT functional!"));
      }
      break;

    case SENDPARA:
     Serial.println(F("     Case SENDPARA     "));
     
     //serial: 36020091410170036
     //token: 870fdc17d12c209791fd4e35f7669f88
     //variables: Temperature, Battery, Level, Distance, Electric Conductivity, Compensated Electrical Conductivity
     
     snprintf(aux_str, sizeof(aux_str), "AT+HTTPPARA=\"URL\",\"http://smability.sidtecmx.com/Device//SetURL?level=%s&distance=%s&ec=%s&cec=%s&temp=%s&batt=%s&deviceID=870fdc17d12c209791fd4e35f7669f88 \"",WaterLevelVal,distanceVal,EleCondVal,CompEleCondVal,tempVal,BattVal);
     
     sendATcommand(aux_str, "OK", 520);

     countloop=1;

     while (sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,3", 11000) != 1){ //45 is the lenght of char text from file csv2sql
       delay(3);
       if(countloop >= 10){
          break;
        };
      countloop++;
    };

     if(countloop >=10 ){
         Serial.println(F("Failure in sending data"));
         //SIM_state = SENDPARA;
         SIM_state = ENDGPRS;
         //evilcounter=evilcounter+1;
     }
     else{
      
         Serial.println(F("Success in sending data"));
         SIM_state = ENDGPRS;
         
         if(sendATcommand("AT+HTTPREAD", "!!", 1200)) // Move inside
         {
           Serial.println(F("Success in handshaking"));
           SIM_state = ENDGPRS;
           //evilcounter=0;
           //Aqui parpadeamos los leds
         }
     }
     
     break;

    case ENDGPRS:
      Serial.println(F("      Case ENDGPRS      "));
      
      sendATcommand("AT+CLTS=0", "OK", 2000); // "Get Local Time" Stamp Disabled
      sendATcommand("AT+HTTPACTION=0", "OK", 2000);
      sendATcommand("AT+SAPBR=0,1", "OK", 4000); // closing bearer if open
      sendATcommand("AT+HTTPTERM", "OK", 4000); // closing http if open
      SIM_state = SIMOFF;
      break;

    case SIMOFF: //  Sleep Mode
      Serial.println(F("      Case SIMOFF     "));
     
      //PowerSIM OFF!
      powerONSIM(); //Vc=low pulse 1.2 seconds
      delay(1200);
      powerOFFSIM(); //Vc=VBAT
      //power Switch OFF
      powerOFFBoost();
      SIM_state = START; // Start over

      f_wdt = 0;// Clear the flag so we can run above code again after the MCU wake up

      Serial.print(F("  Elapsed Time in MilliSeconds: "));
      Serial.println((millis() - InitialTime));

      enterSleep(); // Re-enter sleep mode.

      break;
  }
  

}

//main loop
void loop() {
  
  SIM_GPRS();
  
}

/****** AT COMMANDS SIM FUNCTION   *****/

int8_t sendATcommand(const char* ATcommand, const char* expected_answer, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[270];
    unsigned long previous;

    memset(response, '\0', 270);    // Initialize the string

    delay(10);

    while( Serial2.available() > 0)
      Serial2.read();               // Clean the input buffer

    Serial2.println(ATcommand);    // Send the AT command

    x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(Serial2.available() != 0){
            // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = Serial2.read();
            delay(5);
            x++;
            // check if the desired answer  is in the response of the module
            if (strstr(response, expected_answer) != NULL){
                answer = 1;
            }
            if (strstr(response, "+FTPPUT: 1,6") != NULL){
              Serial.println(F("FTP Error Code"));
                answer = 2;
            }
            if (strstr(response, "+HTTPACTION: 0,6") != NULL){
              Serial.println(F("HTTP Error Code"));
                answer = 2;
            }
            if (strstr(response, "ERROR") != NULL){
              Serial.println(F("AT Command ERROR"));
                answer = 2;
            }
            if (strstr(response, "Location Not Fix") != NULL){
              Serial.println(F("GPS not found"));
                answer = 2;
            }
            /*if (strstr(response, "+CREG: 0,2") != NULL)
            {
              Serial.println(F("Network not found"));
                answer = 2;
            }*/
        }
    }
    // Waits for the asnwer with time out
    while((answer == 0) && ((millis() - previous) < timeout));
      Serial.println(response);    // Send the AT command

    if((millis() - previous) > timeout){
        Serial.print(F("Time exceeded: "));
        Serial.println(millis() - previous);
    }
    return answer;
}
