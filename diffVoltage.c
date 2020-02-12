void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  
  int ValueA0 = analogRead (A0);  // read A0
  int ValueA1 = analogRead (A1);  // read A1    
      
  float voltageV1 = ValueA0 * (5.0 / 1023.0); // V1 voltage
  float voltageV2 = ValueA1 * (5.0 / 1023.0); // V2 voltage

  float diffVoltage = (voltageV1 - voltageV2);
  
  Serial.print(" V1");
  Serial.print(voltageV1);
  Serial.print(" V2");
  Serial.print(voltageV2);
  Serial.print(" Vdiff");
  Serial.print(diffVoltage);
  Serial.print("\n");
  delay(1); //collect 20 samples = 20ms, (Signal Period)

}
