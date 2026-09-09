// VR_Opto_9 -- combines VR_Opto_8's frame-interleaved opto pulse with
// VR_Opto_7's PMT shutter output.
//
// A stim event is triggered by a frame edge and then runs entirely on absolute
// micros() deadlines, so the whole close/fire/reopen sequence completes INSIDE
// the stim frame and the blade is stationary before the next imaging frame
// begins. Shutter reopen is deliberately NOT tied to the following frame edge --
// that was the old behavior, and it left the blade travelling during the first
// 5 ms of every imaging frame (visible as a dark band across the top of the
// raster).
//
// Event timeline (30 Hz imaging, frame = 33.333 ms, blade = 5 ms both ways):
//   t =  0.00 ms  frame edge -- shutter CLOSE commanded
//   t =  5.00 ms  blade fully closed
//   t =  6.00 ms  opto ON        (optoLagTime; 1.0 ms margin past full closure)
//   t = 26.00 ms  opto OFF       (optoPulseDuration = 20 ms)
//   t = 28.00 ms  shutter OPEN commanded (shutterOpenDelay = 2 ms after light off)
//   t = 33.00 ms  blade fully open
//   t = 33.33 ms  next frame edge -- imaging frame starts on a still, open blade
//
// The light never overlaps either blade transition, and the sequence has 0.33 ms
// of slack against the frame boundary. That slack is thin: dropping
// optoPulseDuration to 19 ms would restore it to 1.33 ms if the blade turns out
// to be slower than spec under sustained cycling. The static_assert block below
// will refuse to compile if any edit breaks the budget.
//
// The stim frame itself is a PARTIAL frame, not a dark one: the shutter reopens
// at t = 28 ms, so the last ~5 ms of that frame images normally (bottom ~16% of
// the raster has signal, the rest is black). Opto is already off by then, so
// there is no exposure risk -- but don't let downstream frame classification be
// confused by a partially-bright frame.

const uint8_t framePin = 5;      // ScanImage frame/flyback signal input
const uint8_t optoPin = 13;      // Optogenetics LED/laser control
const uint8_t enablePin = 12;    // behavior-epoch enable line
const uint8_t PMTShutterPin = 7; // BNC out to shutter driver (NORMALLY OPEN:
                                 // HIGH = closed, LOW = open/default -- confirmed
                                 // via bench test, carried over from VR_Opto_7)

// ---------------------------------------------------------------------------
// REPETITION KNOB. framesPerAnchor sets how often a stim event fires; it no
// longer affects the pulse shape, which is fixed by the deadlines below.
// A stim event occupies one frame, so at a 30 Hz imaging clock:
//
//   framesPerAnchor   stim every   stim rate   shutter cycles/s   light duty
//         1            2 frames      15.0 Hz        15.0            30.0%
//         2            4 frames       7.5 Hz         7.5            15.0%
//         4            8 frames       3.75 Hz        3.75             7.5%
//
// Shutter is rated for its top rate in ~4 s bursts with ~1 min dead time, so
// framesPerAnchor = 1 is a short-burst-only configuration. Note that reducing
// the pulse length does NOT reduce shutter wear -- only framesPerAnchor does,
// since the blade still does one full close/open cycle per stim event.
const uint8_t framesPerAnchor = 1;
// ---------------------------------------------------------------------------

uint16_t highCount = 0;
bool fireThisAnchor = true; // alternates true/false at each anchor; true = fire

bool prevFrameState = LOW;
bool prevEnableState = LOW;

bool optoActive = false;        // a stim event is in progress
unsigned long optoStartTime = 0; // micros() at the triggering frame edge

// Imaging frame period. Change if the microscope is not running at 30 Hz.
const unsigned long framePeriod = 33333; // microseconds (30 Hz)

// Measured full blade travel from command edge, open and close (bench-measured).
// Only used by the compile-time budget checks; the firmware never waits on it.
const unsigned long shutterTravel = 5000; // microseconds

// The three deadlines that define a stim event, all relative to the frame edge.
const unsigned long optoLagTime = 6000;        // shutter CLOSE -> opto ON
const unsigned long optoPulseDuration = 20000; // opto ON duration
const unsigned long shutterOpenDelay = 2000;   // opto OFF -> shutter OPEN

// Derived absolute deadlines, measured from optoStartTime.
const unsigned long optoOffTime = optoLagTime + optoPulseDuration;
const unsigned long shutterOpenTime = optoOffTime + shutterOpenDelay;
const unsigned long eventEndTime = shutterOpenTime + shutterTravel;

// Budget guards -- these are the two invariants the whole design rests on.
// If a future edit breaks either one, this fails to compile rather than
// silently exposing the PMT.
static_assert(optoLagTime >= shutterTravel,
              "opto would fire before the blade is fully closed");
static_assert(eventEndTime <= framePeriod,
              "blade would still be moving when the next imaging frame starts");

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

  // Safety abort: if enable drops mid-event, kill the light and reopen.
  if (!currentEnableState && prevEnableState && optoActive) {
    digitalWrite(optoPin, LOW);
    digitalWrite(PMTShutterPin, LOW); // reopen
    optoActive = false;
  }

  // Anchored on the first edge after enable, then every framesPerAnchor edges.
  // Every other anchor fires; the alternate ones are the rest interval.
  if (currentEnableState) {
    if (currentFrameState && !prevFrameState) { // rising edge on frame signal
      highCount++;
      if ((highCount - 1) % framesPerAnchor == 0) {
        if (fireThisAnchor && !optoActive) {
          digitalWrite(PMTShutterPin, HIGH); // close shutter before the light
          optoStartTime = micros();
          optoActive = true;
        }
        fireThisAnchor = !fireThisAnchor; // alternate regardless of whether this one fired
      }
    }
  }

  // Run the event off absolute deadlines. Each event completes well before the
  // next anchor, so optoActive is cleared here rather than by a later frame edge.
  if (optoActive) {
    unsigned long elapsed = micros() - optoStartTime;

    if (elapsed >= optoLagTime && elapsed < optoOffTime) {
      digitalWrite(optoPin, HIGH);
    }

    if (elapsed >= optoOffTime) {
      digitalWrite(optoPin, LOW);
    }

    // Reopen last, and only after the light has been off for shutterOpenDelay.
    if (elapsed >= shutterOpenTime) {
      digitalWrite(PMTShutterPin, LOW);
      optoActive = false;
    }
  }

  prevFrameState = currentFrameState;
  prevEnableState = currentEnableState;
}
