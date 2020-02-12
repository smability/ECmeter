void setup() {
  // put your setup code here, to run once:
 /*
 * ADC converts from 0 to 5 v
 * with a 4.9,V resolution for 1023 steps
 * voltage conversion (positive and negative part?)
 */
}

float dt = 1; //sampling time = 1ms, integration time = samplingtime*20 = 20ms (integration-total-time)
int n = 20; //number of samples
 
float RMS(float ds, int n){
  //n = 20, since dt = 1ms, period T = 20ms 
  
  int i = 0;
  float sum = 0; // reset sum every 20ms (20 samples)
  float E[n];    //store sensor values
    
    while (i<n){
      
      int sensorValue = analogRead (A0);  // read A0  
      
      float voltage = sensorValue * (5.0 / 1023.0);// convert to voltage  
      
      E[i] = voltage; //collect n=20 samples
       
      sum = sum + E[i]*E[i]*ds; //integrate as as fast as the loop time, then divide by 20 samples then wait for 1ms
      
      delay(ds); //dt=ds=delay=sampling time
      
      i++; 
    }
    
  float avgSig = sum/n;//n=20, dt=1ms hece T = n*dt = 20ms
  
  float RMS_sig = sqrt(avgSig);
  
  return RMS_sig;
  }

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(RMS(dt,n));
 
}
