const uint8_t framePin = 5;      // ScanImage frame/flyback signal input
const uint8_t optoPin = 13;      // Optogenetics LED/laser control
const uint8_t enablePin = 12;    // renamed from EFTransitionPin -- behavior-epoch enable line

// Anchor points every N frame edges. With framesPerAnchor = 1 every frame is an
// anchor, and since only every OTHER anchor fires, stim lands on alternate
// frames: 15 Hz stim off a 30 Hz frame clock (1 frame = 33.3 ms).
const uint8_t framesPerAnchor = 1;
uint16_t highCount = 0;
bool fireThisAnchor = true; // alternates true/false at each anchor; true = fire

bool prevFrameState = LOW;
bool prevEnableState = LOW;

bool optoActive = false;
unsigned long optoStartTime = 0;
// 1 ms lag + 31 ms pulse = 32 ms, which fits inside a 33.3 ms frame at 30 Hz and
// leaves ~1.3 ms of guard before the next frame edge, keeping the light clear of
// flyback at both ends.
unsigned long optoLagTime = 1000; // microseconds
const unsigned long optoPulseDuration = 31000; // microseconds

void setup() {
  pinMode(framePin, INPUT);
  pinMode(enablePin, INPUT);
  pinMode(optoPin, OUTPUT);
  digitalWrite(optoPin, LOW);
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
    optoActive = false;
  }

  // Anchored on the first edge after enable. With framesPerAnchor = 1 that is
  // every edge (1, 2, 3, ...); every other anchor fires a 31 ms pulse and the
  // alternate ones are skipped.
  if (currentEnableState) {
    if (currentFrameState && !prevFrameState) { // rising edge on frame signal
      highCount++;
      if ((highCount - 1) % framesPerAnchor == 0) {
        if (fireThisAnchor && !optoActive) {
          optoStartTime = micros();
          optoActive = true;
        }
        // This is what clears optoActive -- the timeout below only drives the
        // pin LOW. Relies on framesPerAnchor = 1 so a skipped anchor always
        // follows a firing one; raising framesPerAnchor would strand it true.
        if (!fireThisAnchor && optoActive) {
          optoActive = false;
        }
        fireThisAnchor = !fireThisAnchor; // alternate regardless of whether this one fired
      }
    }
  }

  // Start opto with a lag and keep it on until duration reached
  if (optoActive && (micros() - optoStartTime >= optoLagTime)  && (micros() - (optoStartTime+optoLagTime) < optoPulseDuration)) {
    digitalWrite(optoPin, HIGH);
  }
  
  // End the pulse once duration reached
  if (optoActive && (micros() - (optoStartTime+optoLagTime) >= optoPulseDuration)) {
    digitalWrite(optoPin, LOW);
  }
  
  prevFrameState = currentFrameState;
  prevEnableState = currentEnableState;
}
