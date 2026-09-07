const uint8_t inputPin = 5; //5;
const uint8_t outputPin = 13; // 13
const uint8_t enablePin = 12; //12;

bool prevInputState = LOW;
bool pulseActive = false;
unsigned long pulseStartTime = 0;
const unsigned long pulseDuration = 500; // milliseconds

void setup() {
  pinMode(inputPin, INPUT);
  pinMode(enablePin, INPUT);
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, LOW);
}

void loop() {
  bool currentInputState = digitalRead(inputPin);
  bool currentEnableState = digitalRead(enablePin);

  // Rising edge on the frame signal starts a pulse, but only while enabled and
  // only if no pulse is already running -- edges arriving mid-pulse are ignored,
  // so the output is a clean pulseDuration-long HIGH rather than a retrigger.
  if (currentInputState && !prevInputState) {
    if (currentEnableState && !pulseActive) {
      digitalWrite(outputPin, HIGH);
      pulseActive = true;
      pulseStartTime = millis();
    }
  }

  // Turn off the output pin after the pulse duration
  if (pulseActive && (millis() - pulseStartTime >= pulseDuration)) {
    digitalWrite(outputPin, LOW);
    pulseActive = false;
  }

  // Safety abort: if enable drops mid-pulse, force off immediately.
  if (!currentEnableState && pulseActive) {
    digitalWrite(outputPin, LOW);
    pulseActive = false;
  }

  prevInputState = currentInputState;
}
