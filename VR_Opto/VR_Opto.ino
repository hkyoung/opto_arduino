int PMTShutterPin = 7; //BNC out to shutter. Positive is Open(?)! Check! Switch to 10 if need to disconnect. 
int optoPin = 13; //Optogenetics LED pin
int EFTransitionPin = 12; //Servo motor brush pin

int baselineDelay = 100; //pre-opto stimulation baseline duraiton
int shutterCloseDelay = 100; //how long to wait after PMT shutter has been closed
int shutterOpenDelay = 100; //how long to wait before opening the PMT shutter
int optoStimDuration = 10000; //how long will each stimulation bout last
int ImagingTime = 2000; //how long will each imaging bout last (in the middle of stimulation)
int numRepeats = 10;
int EFTransitionPinVal = LOW;

void setup() {
  pinMode(optoPin,OUTPUT); //Optogenetics LED
  pinMode(PMTShutterPin, OUTPUT); //to start the trial
  pinMode(EFTransitionPin,INPUT);
  
  digitalWrite(optoPin,LOW);
  delay(500);
  digitalWrite(optoPin,LOW); //change to HIGH check opto PIN
  delay(500);
  digitalWrite(optoPin,LOW);
  delay(500);
  digitalWrite(PMTShutterPin,LOW);
  delay(500);
  digitalWrite(PMTShutterPin,LOW);//change to HIGH check PMT shutter
  delay(500);
  digitalWrite(PMTShutterPin,LOW);
}

void loop() {
  EFTransitionPinVal = digitalRead(EFTransitionPin);
  if(EFTransitionPinVal == HIGH){
    delay(baselineDelay);
    digitalWrite(PMTShutterPin,LOW);
    delay(shutterCloseDelay);
    for(int i =1; i<=numRepeats; i++) {
      digitalWrite(optoPin,HIGH);
      delay(500);
      digitalWrite(optoPin,LOW);
      delay(500);
    }
    delay(shutterOpenDelay);
    digitalWrite(optoPin,LOW);
    digitalWrite(PMTShutterPin,HIGH);
    delay(ImagingTime);
    digitalWrite(PMTShutterPin,LOW);
  }
  else if(EFTransitionPinVal == LOW){
    digitalWrite(optoPin,LOW);
    digitalWrite(PMTShutterPin,HIGH);
    delay(100);

  }
  else{
    delay(100);
  }
}