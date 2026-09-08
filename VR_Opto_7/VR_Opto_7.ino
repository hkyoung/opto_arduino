const uint8_t framePin = 5;      // ScanImage frame/flyback signal input
const uint8_t optoPin = 13;      // Optogenetics LED/laser control
const uint8_t enablePin = 12;    // renamed from EFTransitionPin -- behavior-epoch enable line
const uint8_t PMTShutterPin = 7; // BNC out to shutter driver (configured NORMALLY OPEN:
                                  // HIGH = closed, LOW = open/default -- confirmed via bench test)

// Anchor points every N frame edges (30 Hz: 16 frames = 533 ms). Only every OTHER
// anchor actually fires a pulse -- the alternate ones are skipped entirely.
const uint8_t framesPerAnchor = 32; //16
uint16_t highCount = 0;
bool fireThisAnchor = true; // alternates true/false at each anchor; true = fire

bool prevFrameState = LOW;
bool prevEnableState = LOW;

bool optoActive = false;
unsigned long optoStartTime = 0;
unsigned long optoLagTime = 33;
const unsigned long optoPulseDuration = 950; // fixed 400 ms pulse when an anchor fires

void setup() {
  pinMode(framePin, INPUT);
  pinMode(enablePin, INPUT);
  pinMode(optoPin, OUTPUT);
  pinMode(PMTShutterPin, OUTPUT);

  digitalWrite(optoPin, LOW);
  digitalWrite(PMTShutterPin, LOW); // start parked open
}

void loop() {
  bool currentFrameState = digitalRead(framePin);
  bool currentEnableState = digitalRead(enablePin);

  // New enable epoch: reset count and always fire on the first anchor.
  if (currentEnableState && !prevEnableState) {
    highCount = 0;
    fireThisAnchor = true;
  }

  // Safety abort: if enable drops mid-pulse, force off/open immediately.
  if (!currentEnableState && prevEnableState && optoActive) {
    digitalWrite(optoPin, LOW);
    digitalWrite(PMTShutterPin, LOW); // reopen
    optoActive = false;
  }

  // Anchored on the first edge after enable (edges 1, 17, 33, ...).
  // Every other anchor fires a 500 ms pulse; the alternate ones are skipped.
  if (currentEnableState) {
    if (currentFrameState && !prevFrameState) { // rising edge on frame signal
      highCount++;
      if ((highCount - 1) % framesPerAnchor == 0) {
        if (fireThisAnchor && !optoActive) {
          digitalWrite(PMTShutterPin, HIGH); // close shutter while opto is on
          optoStartTime = millis();
          optoActive = true;
        }
        if (!fireThisAnchor && optoActive) {
          digitalWrite(PMTShutterPin, LOW); // reopen shutter while opto is on
          optoActive = false;
        }
        fireThisAnchor = !fireThisAnchor; // alternate regardless of whether this one fired
      }
    }
  }
  
  // Start opto with a lag
  if (optoActive && (millis() - optoStartTime >= optoLagTime)  && (millis() - (optoStartTime+optoLagTime) < optoPulseDuration)) {
    digitalWrite(optoPin, HIGH);
  }
  
  // End the pulse (500 ms after it started) and reopen the shutter.
  if (optoActive && (millis() - (optoStartTime+optoLagTime) >= optoPulseDuration)) {
    digitalWrite(optoPin, LOW);
    //digitalWrite(PMTShutterPin, LOW); // reopen
    //optoActive = false;
  }

  prevFrameState = currentFrameState;
  prevEnableState = currentEnableState;
}
