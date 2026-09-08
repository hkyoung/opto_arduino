// VR_Opto_9 -- combines VR_Opto_8's frame-interleaved opto pulse with
// VR_Opto_7's PMT shutter output.
//
// Opto and shutter run in alternating phases of framesPerAnchor frames each:
// the shutter closes and the light fires for one phase, then both rest for the
// next. Everything below is derived from framesPerAnchor, so changing that one
// constant retimes the whole sketch and keeps the shutter/opto guard bands
// intact -- see the table at framesPerAnchor.
//
// Phase timing (framesPerAnchor = 2, 30 Hz imaging, frame = 33.3 ms):
//   t =  0.0 ms  frame edge -- shutter CLOSES
//   t =  1.0 ms  opto ON      (optoLagTime after the shutter is already shut)
//   t = 65.0 ms  opto OFF     (optoTrailGuard before the shutter moves)
//   t = 66.7 ms  frame edge -- shutter REOPENS
// The shutter therefore brackets the light at both ends and the stimulation
// never runs while the shutter is open. Rest phases leave the shutter open.
//
// Note the light stays on across the frame boundary inside a multi-frame phase,
// so intra-phase flyback periods are illuminated. That is safe here precisely
// because the shutter is closed for the whole phase -- but it does mean those
// frames are not usable imaging frames.

const uint8_t framePin = 5;      // ScanImage frame/flyback signal input
const uint8_t optoPin = 13;      // Optogenetics LED/laser control
const uint8_t enablePin = 12;    // behavior-epoch enable line
const uint8_t PMTShutterPin = 7; // BNC out to shutter driver (NORMALLY OPEN:
                                 // HIGH = closed, LOW = open/default -- confirmed
                                 // via bench test, carried over from VR_Opto_7)

// ---------------------------------------------------------------------------
// THE ONE KNOB. Frames per phase, at a 30 Hz imaging clock:
//
//   framesPerAnchor   phase     shutter cycles/s   opto pulse   notes
//         1           33.3 ms        15.0          30.6 ms      max shutter rate
//         2           66.7 ms         7.5          64.0 ms      <-- current
//         4          133.3 ms         3.75        130.6 ms      gentler still
//
// Shutter is rated for its top rate in ~4 s bursts with ~1 min dead time, so
// framesPerAnchor = 1 is a short-burst-only configuration. This is currently set
// to the slower 2-frame version for testing; set it back to 1 to return to the
// max-rate implementation -- nothing else needs to change.
const uint8_t framesPerAnchor = 2;
// ---------------------------------------------------------------------------

uint16_t highCount = 0;
bool fireThisAnchor = true; // alternates true/false at each anchor; true = fire

bool prevFrameState = LOW;
bool prevEnableState = LOW;

bool optoActive = false;
unsigned long optoStartTime = 0;

// Imaging frame period. Change if the microscope is not running at 30 Hz.
const unsigned long framePeriod = 33333; // microseconds (30 Hz)

// Guard bands that keep the light strictly inside the shutter-closed window:
// optoLagTime after the shutter starts closing, optoTrailGuard before it
// reopens. Both are generous relative to the shutter's own travel time.
const unsigned long optoLagTime = 1000;    // microseconds
const unsigned long optoTrailGuard = 1700; // microseconds

// Fill the phase, minus the guards at each end. Derived so that retiming via
// framesPerAnchor cannot accidentally push light past the shutter.
const unsigned long optoPulseDuration =
    (unsigned long)framesPerAnchor * framePeriod - optoLagTime - optoTrailGuard;

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

  // Safety abort: if enable drops mid-pulse, force off and reopen immediately.
  if (!currentEnableState && prevEnableState && optoActive) {
    digitalWrite(optoPin, LOW);
    digitalWrite(PMTShutterPin, LOW); // reopen
    optoActive = false;
  }

  // Anchored on the first edge after enable, then every framesPerAnchor edges
  // (with framesPerAnchor = 2: edges 1, 3, 5, ...). Every other anchor fires;
  // the alternate ones end the phase.
  if (currentEnableState) {
    if (currentFrameState && !prevFrameState) { // rising edge on frame signal
      highCount++;
      if ((highCount - 1) % framesPerAnchor == 0) {
        if (fireThisAnchor && !optoActive) {
          digitalWrite(PMTShutterPin, HIGH); // close shutter before the light
          optoStartTime = micros();
          optoActive = true;
        }
        // This is what clears optoActive and reopens the shutter -- the timeout
        // below only drives the opto pin LOW. Safe for any framesPerAnchor,
        // since fireThisAnchor alternates between anchors, so a skipped anchor
        // always follows a firing one.
        if (!fireThisAnchor && optoActive) {
          digitalWrite(PMTShutterPin, LOW); // reopen after the light is done
          optoActive = false;
        }
        fireThisAnchor = !fireThisAnchor; // alternate regardless of whether this one fired
      }
    }
  }

  // Start opto with a lag and keep it on until duration reached
  if (optoActive && (micros() - optoStartTime >= optoLagTime) && (micros() - (optoStartTime + optoLagTime) < optoPulseDuration)) {
    digitalWrite(optoPin, HIGH);
  }

  // End the pulse once duration reached. The shutter stays closed until the next
  // anchor (see the skipped-anchor branch above).
  if (optoActive && (micros() - (optoStartTime + optoLagTime) >= optoPulseDuration)) {
    digitalWrite(optoPin, LOW);
  }

  prevFrameState = currentFrameState;
  prevEnableState = currentEnableState;
}
