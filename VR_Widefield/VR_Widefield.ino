const uint8_t inputPin = 3; // camera trigger
const uint8_t outputPinR = 13; // R LED
const uint8_t outputPinG = 12; // G LED
const uint8_t outputPinB = 11; // B LED
const uint8_t enablePin = 10; // VR trigger 

uint8_t highCount = 0;
bool prevInputState = LOW;
bool pulseActiveR = false;
bool pulseActiveG = false;
bool pulseActiveB = false;
unsigned long pulseStartTime = 0;
const unsigned long pulseDuration = 60; // 60 milliseconds

void setup() {
  pinMode(inputPin, INPUT);
  pinMode(enablePin, INPUT);
  pinMode(outputPinR, OUTPUT);
  pinMode(outputPinG, OUTPUT);
  pinMode(outputPinB, OUTPUT);
  digitalWrite(outputPinR, LOW);
  digitalWrite(outputPinG, LOW);
  digitalWrite(outputPinB, LOW);
}

void loop() {
  bool currentInputState = digitalRead(inputPin);

  // Detect rising edge
  if (currentInputState && !prevInputState) {
    highCount++;

    if (highCount % 3 == 0) {
      digitalWrite(outputPinR, HIGH);
      pulseActiveR = true;
      pulseStartTime = millis();
    }
    else if (highCount % 3 == 1) {
      digitalWrite(outputPinG, HIGH);
      pulseActiveG = true;
      pulseStartTime = millis();
    }
    else {
      digitalWrite(outputPinB, HIGH);
      pulseActiveB = true;
      pulseStartTime = millis();
    }
  }

  // Turn off the output pin after the pulse duration
  if (pulseActiveR && (millis() - pulseStartTime >= pulseDuration)) {
    digitalWrite(outputPinR, LOW);
    pulseActiveR = false;
  } else if (pulseActiveG && (millis() - pulseStartTime >= pulseDuration)) {
    digitalWrite(outputPinG, LOW);
    pulseActiveG = false;
  } else if (pulseActiveB && (millis() - pulseStartTime >= pulseDuration)) {
    digitalWrite(outputPinB, LOW);
    pulseActiveB = false;
  }

  prevInputState = currentInputState;
}
