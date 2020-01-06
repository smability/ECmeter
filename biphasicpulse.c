void setup() {
  // put your setup code here, to run once:
  // Set PIN modes
  pins_init();
}

//Control signal ports

// Control Signal A
#define CONTROLA (11)
// Control Signal B
#define CONTROLB (12)

//port definition
void pins_init()
{
  pinMode(CONTROLA, OUTPUT);
  pinMode(CONTROLB, OUTPUT);

}

// Control A "ON"
void ControlAON()
  {
  digitalWrite (CONTROLA, HIGH);  // turn CONTROL A ON
  delay(1); // give time to power up (mosfet)
  }  // end of powerONSensor

// Control A "OFF"
void ControlAOFF()
  {
  digitalWrite (CONTROLA, LOW);  // turn CONTROL A OFF
  delay(1); // give time to power down (mosfet)
  }  // end of powerONSensor

// Control B "ON"
void ControlBON()
  {
  digitalWrite (CONTROLB, HIGH);  // turn CONTROL B ON
  delay(1); // give time to power up (mosfet)
  }  // end of powerONSensor

// Control B "OFF"
void ControlBOFF()
  {
  digitalWrite (CONTROLB, LOW);  // turn CONTROL B ON
  delay(1); // give time to power down (mosfet)
  }  // end of powerONSensor

  
void loop() {
  // put your main code here, to run repeatedly:
  //Biphasic pulse construction
  //positive part
  ControlAOFF();
  ControlBON();
  delay(250);
  //delay positive part
  ControlAON();
  ControlBON();
  delay(250);
  //negative part
  ControlBOFF();
  ControlAON();
  delay(250);
  //delay negative part
  ControlAON();
  ControlBON();
  delay(250);
}
