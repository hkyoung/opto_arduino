/*
  ===================================================================================
  Widefield Camera Trigger + Optogenetic Laser Controller  (v4)
  Non-blocking, millis()-based timing on Arduino
  ===================================================================================

  OVERVIEW
  --------
  This firmware coordinates two fully independent subsystems:

  1. CAMERA TRIGGER (Pin 3)
     Generates a free-running pulse train while an experiment is active.
     Each pulse goes HIGH for exposureHighMs, then LOW for the remainder of
     framePeriodMs.  Absolute-time scheduling (anchored to experiment start)
     prevents cumulative drift even if the main loop occasionally lags.
     Default: 200 ms period (5 Hz), 170 ms HIGH.

  2. OPTOGENETIC YELLOW LASER (Pin 13)
     Pulses independently of the camera with its own configurable period
     (optoPeriodMs, default 800 ms) and ON duration (optoOnDurationMs,
     default 200 ms).  Two activation sources:

     a) Pin 10 (external behavior/VR TTL):
        - HIGH  -> laser begins pulsing at the base rate.
        - LOW   -> current pulse completes, then a ramp-down sequence
                   progressively stretches the period over a configurable
                   total duration (rampDownDurationMs, default 1500 s).
                   The ramp is divided into rampSteps equal intervals
                   (default 5); at each boundary the period is multiplied
                   by periodMultiplier (default 2).  After the full ramp
                   duration the laser shuts off.
        - If Pin 10 goes HIGH again *during* ramp-down, the ramp is
          cancelled and full-rate pulsing resumes immediately.

     b) Serial "o<seconds>" command (e.g. o5, o10):
        - Pulses the laser for the specified number of seconds using the
          current period and ON-duration settings.
        - Works regardless of experiment state or Pin 10.
        - NO ramp-down when this timed test ends -- the laser simply stops.

  PIN MAP
  -------
    Pin  3  - Camera trigger          (OUTPUT, active HIGH)
    Pin 10  - Behavior/VR TTL input   (INPUT,  HIGH = opto active)
    Pin 13  - Yellow opto laser       (OUTPUT, active HIGH)

  Note: Pins 11 (blue LED) and 12 (green LED) from earlier versions have been
  removed entirely.  Those LEDs are now controlled manually (always-on or
  always-off) outside of this firmware.

  DEFAULT TIMING (all editable at runtime via serial)
  ---------------------------------------------------
    Camera frame period    : 200 ms  (5 Hz)
    Camera exposure HIGH   : 170 ms
    Opto laser period      : 800 ms
    Opto laser ON duration : 200 ms  (-> 600 ms OFF per cycle)
    Ramp-down duration     : 1 500 000 ms  (1500 s / 25 min)
    Ramp-down steps        : 5
    Ramp-down multiplier   : 2       (period doubles each step)

  SERIAL COMMANDS (115 200 baud)
  ------------------------------
    s              Start experiment (camera begins, Pin 10 monitoring active)
    x              Stop everything immediately (experiment, laser, ramp-down)
    o<seconds>     Test opto pulsing for N seconds (e.g. o5, o 10).
                   Works any time, ignores Pin 10, no ramp-down at end.
    f <ms>         Set camera frame period
    h <ms>         Set camera exposure HIGH width
    p <ms>         Set opto laser period
    d <ms>         Set opto laser ON duration
    r <ms>         Set ramp-down total duration (in ms)
    n <steps>      Set number of ramp-down steps (integer >= 1)
    m <factor>     Set ramp-down period multiplier (integer >= 2)
    ?              Show help and current settings
*/

#include <Arduino.h>

// =====================================================================
//  PIN DEFINITIONS
// =====================================================================
const uint8_t PIN_CAMERA = 3;    // Camera trigger output (active HIGH)
const uint8_t PIN_BEHAV  = 10;   // Behavior / VR TTL input
const uint8_t PIN_LASER  = 13;   // Yellow optogenetic laser output

// =====================================================================
//  USER-EDITABLE CONFIGURATION
//  All values can also be changed at runtime through serial commands.
// =====================================================================

// --- Camera timing ---
unsigned long framePeriodMs   = 200UL;   // Period between camera triggers (ms)
                                         // 200 ms = 5 Hz frame rate
unsigned long exposureHighMs  = 170UL;   // Duration camera trigger stays HIGH (ms)

// --- Opto laser timing ---
unsigned long optoPeriodMs      = 500UL; // Full cycle length of one laser pulse (ms)
                                         // Laser is ON for optoOnDurationMs, then
                                         // OFF for (optoPeriodMs - optoOnDurationMs)
unsigned long optoOnDurationMs  = 200UL; // How long the laser stays ON each cycle (ms)

// --- Ramp-down configuration ---
// When Pin 10 transitions HIGH -> LOW during a running experiment, the laser
// does not stop abruptly.  Instead it enters a "ramp-down" phase:
//   - The total ramp lasts rampDownDurationMs (e.g. 1 500 000 ms = 1500 s).
//   - The ramp is divided into rampSteps equal-length intervals.
//   - At each interval boundary the laser pulse period is multiplied by
//     periodMultiplier (e.g. doubled), while ON duration stays the same.
//   - After the full ramp duration the laser turns off permanently (until
//     Pin 10 goes HIGH again).
//
// Example with defaults (period 800 ms, multiplier 2, 5 steps, 1500 s total):
//     0 –  300 s : period =   800 ms  (base)
//   300 –  600 s : period = 1 600 ms  (×2)
//   600 –  900 s : period = 3 200 ms  (×4)
//   900 – 1200 s : period = 6 400 ms  (×8)
//  1200 – 1500 s : period = 12 800 ms (×16)
//  After 1500 s  : laser OFF
unsigned long rampDownDurationMs = 300000UL;  // Total ramp-down time (ms)
unsigned int  rampSteps          = 5;          // Number of intervals in the ramp
unsigned int  periodMultiplier   = 2;          // Factor applied to period each step

// =====================================================================
//  EXPERIMENT STATE
// =====================================================================
volatile bool running           = false;  // Is the experiment active?
unsigned long expStartMs        = 0;      // millis() when experiment started
unsigned long lastStatusMs      = 0;      // millis() when last status line printed
unsigned long framesSent        = 0;      // Total camera trigger pulses fired

// --- Camera pulse tracking ---
bool          cameraOn          = false;  // Is the camera trigger currently HIGH?
unsigned long cameraOffAtMs     = 0;      // Scheduled time to turn camera LOW
long          lastCameraFrameIdx = -1;    // Last frame index processed (for scheduler)

// =====================================================================
//  OPTO LASER STATE MACHINE
// =====================================================================
// The laser can be in one of four states:
//   OPTO_IDLE           – Laser off, nothing happening.
//   OPTO_ACTIVE_PIN10   – Pulsing because Pin 10 is HIGH (experiment must be running).
//   OPTO_ACTIVE_SERIAL  – Pulsing because of a serial "o" test command.
//   OPTO_RAMPDOWN       – Pin 10 went LOW; laser is ramping down.
enum OptoState {
  OPTO_IDLE,
  OPTO_ACTIVE_PIN10,
  OPTO_ACTIVE_SERIAL,
  OPTO_RAMPDOWN
};

OptoState     optoState           = OPTO_IDLE;
bool          laserPhysicallyOn   = false;      // Physical state of Pin 13
unsigned long laserToggleAtMs     = 0;          // Next scheduled ON/OFF toggle time

// --- Serial "o" test ---
unsigned long optoSerialEndMs     = 0;          // When the timed serial test expires

// --- Ramp-down tracking ---
unsigned long rampDownStartMs     = 0;          // millis() when ramp-down began
unsigned int  currentRampStep     = 0;          // Current step index (0-based)
unsigned long currentRampPeriodMs = 0;          // Effective period in current ramp step
bool          pendingRampDown     = false;       // True if waiting for current pulse to
                                                // finish before entering ramp-down

// =====================================================================
//  PIN 10 DEBOUNCE AND TRANSITION LOGGING
// =====================================================================
// A 5 ms debounce filter is applied to Pin 10 to reject electrical noise.
// The filtered (debounced) state is used both for opto activation decisions
// and for logging transitions in the stop report.

const unsigned long BEHAV_FILTER_MS = 5UL;      // Debounce window (ms)
int           lastBehavRaw          = LOW;       // Most recent raw digitalRead
int           debouncedBehavState   = LOW;       // Stable debounced state
unsigned long lastBehavEdgeMs       = 0;         // When raw value last changed

// Transition event log (timestamps stored relative to experiment start when running)
const uint16_t MAX_BEHAV_EVENTS = 64;
unsigned long  behavEventTimeMs[MAX_BEHAV_EVENTS];
int            behavEventState[MAX_BEHAV_EVENTS];
uint16_t       behavEventCount = 0;

// =====================================================================
//  STATUS REPORTING
// =====================================================================
const unsigned long STATUS_INTERVAL_MS = 10000UL;  // Print status every 10 s

// =====================================================================
//  UTILITY FUNCTIONS
// =====================================================================

// Immediately force the laser output LOW and update tracking flag.
void setLaserOff() {
  digitalWrite(PIN_LASER, LOW);
  laserPhysicallyOn = false;
}

// Immediately force the camera trigger LOW and update tracking flag.
void setCameraOff() {
  digitalWrite(PIN_CAMERA, LOW);
  cameraOn = false;
}

// Clamp camera timing values to sane ranges.  exposureHighMs is allowed to
// equal framePeriodMs (100 % duty) but not exceed it.
void clipCameraTimings() {
  if (framePeriodMs  < 1UL) framePeriodMs  = 1UL;
  if (exposureHighMs < 1UL) exposureHighMs = 1UL;
  if (exposureHighMs > framePeriodMs) exposureHighMs = framePeriodMs;
}

// Clamp opto timing values.  We intentionally allow optoOnDurationMs >=
// optoPeriodMs (100 % duty cycle -- laser stays on continuously).
void clipOptoTimings() {
  if (optoPeriodMs     < 1UL) optoPeriodMs     = 1UL;
  if (optoOnDurationMs < 1UL) optoOnDurationMs = 1UL;
}

// Format a millisecond count as  HH:MM:SS.mmm  for human-readable display.
void formatHMSms(unsigned long totalMs, char* out, size_t n) {
  unsigned long ms = totalMs % 1000UL;
  unsigned long s  = (totalMs / 1000UL)   % 60UL;
  unsigned long m  = (totalMs / 60000UL)  % 60UL;
  unsigned long h  = totalMs / 3600000UL;
  snprintf(out, n, "%02lu:%02lu:%02lu.%03lu", h, m, s, ms);
}

// Compute  base * (multiplier ^ exponent)  with unsigned-long overflow guard.
// If the result would overflow a 32-bit unsigned long the function returns
// the maximum representable value (0xFFFFFFFF ≈ 49.7 days in ms).
unsigned long safePow(unsigned long base, unsigned int multiplier,
                      unsigned int exponent) {
  unsigned long result = base;
  for (unsigned int i = 0; i < exponent; i++) {
    // Check whether the next multiplication would overflow
    if (result > 0xFFFFFFFFUL / multiplier) {
      return 0xFFFFFFFFUL;  // cap at max
    }
    result *= multiplier;
  }
  return result;
}

// =====================================================================
//  SERIAL OUTPUT HELPERS
// =====================================================================

void printHelp() {
  Serial.println(F("\n================ COMMANDS ================"));
  Serial.println(F("  s              Start experiment"));
  Serial.println(F("  x              Stop everything + report"));
  Serial.println(F("  o<sec>         Test opto for N seconds (o5, o 10)"));
  Serial.println(F("  f <ms>         Camera frame period"));
  Serial.println(F("  h <ms>         Camera exposure HIGH width"));
  Serial.println(F("  p <ms>         Opto laser period"));
  Serial.println(F("  d <ms>         Opto laser ON duration"));
  Serial.println(F("  r <ms>         Ramp-down total duration (ms)"));
  Serial.println(F("  n <steps>      Ramp-down step count"));
  Serial.println(F("  m <factor>     Ramp-down period multiplier (>=2)"));
  Serial.println(F("  ?              Show help & settings"));
  Serial.println(F("==========================================\n"));
}

void printSettings() {
  // Camera
  Serial.println(F("--- Camera Settings ---"));
  Serial.print(F("  Frame period     : ")); Serial.print(framePeriodMs);
  Serial.print(F(" ms  (")); Serial.print(1000.0f / (float)framePeriodMs, 2);
  Serial.println(F(" Hz)"));
  Serial.print(F("  Exposure HIGH    : ")); Serial.print(exposureHighMs);
  Serial.println(F(" ms"));

  // Opto
  Serial.println(F("--- Opto Laser Settings ---"));
  Serial.print(F("  Pulse period     : ")); Serial.print(optoPeriodMs);
  Serial.println(F(" ms"));
  Serial.print(F("  ON duration      : ")); Serial.print(optoOnDurationMs);
  Serial.println(F(" ms"));
  if (optoOnDurationMs >= optoPeriodMs) {
    Serial.println(F("  OFF duration     : 0 ms  (100% duty cycle)"));
  } else {
    Serial.print(F("  OFF duration     : ")); Serial.print(optoPeriodMs - optoOnDurationMs);
    Serial.println(F(" ms"));
  }

  // Ramp-down
  Serial.println(F("--- Ramp-Down Settings ---"));
  Serial.print(F("  Total duration   : ")); Serial.print(rampDownDurationMs);
  Serial.print(F(" ms  (")); Serial.print(rampDownDurationMs / 1000.0, 1);
  Serial.println(F(" s)"));
  Serial.print(F("  Steps            : ")); Serial.println(rampSteps);
  Serial.print(F("  Period multiplier: ")); Serial.println(periodMultiplier);
  if (rampSteps > 0) {
    unsigned long stepDur = rampDownDurationMs / (unsigned long)rampSteps;
    Serial.print(F("  Step duration    : ")); Serial.print(stepDur);
    Serial.print(F(" ms  (")); Serial.print(stepDur / 1000.0, 1);
    Serial.println(F(" s)"));

    // Print preview of ramp schedule
    Serial.println(F("  Ramp schedule preview:"));
    unsigned long p = optoPeriodMs;
    for (unsigned int i = 0; i < rampSteps; i++) {
      unsigned long segStart = stepDur * i;
      unsigned long segEnd   = stepDur * (i + 1);
      Serial.print(F("    Step ")); Serial.print(i + 1);
      Serial.print(F(": ")); Serial.print(segStart / 1000.0, 1);
      Serial.print(F("s - ")); Serial.print(segEnd / 1000.0, 1);
      Serial.print(F("s  ->  period = ")); Serial.print(p);
      Serial.println(F(" ms"));
      p = safePow(optoPeriodMs, periodMultiplier, i + 1);
    }
    Serial.print(F("    After ")); Serial.print(rampDownDurationMs / 1000.0, 1);
    Serial.println(F("s  ->  laser OFF"));
  } else {
    Serial.println(F("  Step duration    : N/A (0 steps -- immediate off)"));
  }
}

// Print a comprehensive status line with experiment time, frame count,
// and full opto state including ramp-down progress.
void printStatusLine(unsigned long nowMs) {
  unsigned long elapsed = running ? (nowMs - expStartMs) : 0UL;
  char buf[24];
  formatHMSms(elapsed, buf, sizeof(buf));

  Serial.print(F("[STATUS] Elapsed: ")); Serial.print(buf);
  Serial.print(F("  |  Frames sent: ")); Serial.print(framesSent);

  // Opto state
  Serial.print(F("  |  Opto: "));
  switch (optoState) {
    case OPTO_IDLE:
      Serial.print(F("OFF"));
      break;

    case OPTO_ACTIVE_PIN10:
      Serial.print(F("ACTIVE (Pin10), laser "));
      Serial.print(laserPhysicallyOn ? F("ON") : F("OFF"));
      break;

    case OPTO_ACTIVE_SERIAL: {
      float secLeft = (optoSerialEndMs > nowMs)
                      ? (optoSerialEndMs - nowMs) / 1000.0f : 0.0f;
      Serial.print(F("ACTIVE (Serial test, "));
      Serial.print(secLeft, 1);
      Serial.print(F("s left), laser "));
      Serial.print(laserPhysicallyOn ? F("ON") : F("OFF"));
      break;
    }

    case OPTO_RAMPDOWN: {
      unsigned long rElapsed   = nowMs - rampDownStartMs;
      unsigned long rRemaining = (rElapsed < rampDownDurationMs)
                                 ? (rampDownDurationMs - rElapsed) : 0UL;
      Serial.print(F("RAMP-DOWN step "));
      Serial.print(currentRampStep + 1); Serial.print(F("/"));
      Serial.print(rampSteps);
      Serial.print(F("  elapsed ")); Serial.print(rElapsed / 1000.0, 1);
      Serial.print(F("s  remaining ")); Serial.print(rRemaining / 1000.0, 1);
      Serial.print(F("s  period ")); Serial.print(currentRampPeriodMs);
      Serial.print(F("ms  laser ")); Serial.print(laserPhysicallyOn ? F("ON") : F("OFF"));
      break;
    }
  }
  Serial.println();
}

// =====================================================================
//  PIN 10 DEBOUNCE AND LOGGING
// =====================================================================

// Record a Pin 10 transition in the event log.
void logBehavTransition(int newState, unsigned long nowMs) {
  if (behavEventCount < MAX_BEHAV_EVENTS) {
    behavEventState[behavEventCount]  = newState;
    // Store time relative to experiment start if running; otherwise absolute
    behavEventTimeMs[behavEventCount] = running ? (nowMs - expStartMs) : nowMs;
    behavEventCount++;
  }
}

// Sample Pin 10 through a 5 ms debounce filter.
// Returns true if the debounced state just changed (caller should act on it).
bool sampleBehavDebounced(unsigned long nowMs) {
  int raw = digitalRead(PIN_BEHAV);

  // If the raw reading differs from the last sample, restart the
  // debounce timer — we need to see a stable reading for at least
  // BEHAV_FILTER_MS before accepting a state change.
  if (raw != lastBehavRaw) {
    lastBehavRaw   = raw;
    lastBehavEdgeMs = nowMs;
  }

  // Accept the new state only after it has been stable for the filter window
  if ((nowMs - lastBehavEdgeMs) >= BEHAV_FILTER_MS) {
    if (debouncedBehavState != raw) {
      debouncedBehavState = raw;
      logBehavTransition(raw, nowMs);
      return true;   // state changed
    }
  }
  return false;       // no change
}

// =====================================================================
//  OPTO LASER CONTROL
// =====================================================================

// Begin a new pulse cycle: turn the laser ON and schedule the first OFF
// toggle.  `period` is the active period for this cycle (may differ from
// the base during ramp-down).
void startOptoPulseCycle(unsigned long nowMs, unsigned long period) {
  digitalWrite(PIN_LASER, HIGH);
  laserPhysicallyOn = true;

  // If ON duration >= period we have 100 % duty; laser stays on until
  // the end of the period rather than toggling off mid-cycle.
  if (optoOnDurationMs >= period) {
    laserToggleAtMs = nowMs + period;  // toggle at end of period (to restart)
  } else {
    laserToggleAtMs = nowMs + optoOnDurationMs;  // schedule OFF
  }
}

// Transition the opto system into the ramp-down state.  This is called
// either immediately when Pin 10 goes LOW (if laser is in OFF phase) or
// after the current ON pulse completes (if pendingRampDown was set).
void beginRampDown(unsigned long nowMs) {
  if (rampSteps == 0) {
    // Edge case: zero ramp steps means "turn off immediately"
    Serial.println(F("[OPTO] Ramp-down: 0 steps configured. Laser OFF immediately."));
    setLaserOff();
    optoState = OPTO_IDLE;
    return;
  }

  optoState           = OPTO_RAMPDOWN;
  rampDownStartMs     = nowMs;
  currentRampStep     = 0;
  currentRampPeriodMs = optoPeriodMs;  // Step 0 uses the base period

  Serial.println(F("[OPTO] === RAMP-DOWN STARTED ==="));
  Serial.print(F("[OPTO]   Step 1/")); Serial.print(rampSteps);
  Serial.print(F("  period: ")); Serial.print(currentRampPeriodMs);
  Serial.print(F(" ms  step duration: "));
  Serial.print(rampDownDurationMs / (unsigned long)rampSteps);
  Serial.println(F(" ms"));

  // Begin the first ramp-down pulse cycle
  startOptoPulseCycle(nowMs, currentRampPeriodMs);
}

// Immediately kill all opto activity and reset to idle.
void stopOptoImmediately() {
  setLaserOff();
  optoState       = OPTO_IDLE;
  pendingRampDown = false;
  Serial.println(F("[OPTO] Laser OFF.  State -> IDLE."));
}

// Main opto state-machine service routine.  Called every loop() iteration.
// Handles serial test timeouts, ramp-down step progression, and the
// ON/OFF toggle cycle of the laser.
void serviceOpto(unsigned long nowMs) {

  // Nothing to do if idle
  if (optoState == OPTO_IDLE) return;

  // ----- Serial test timeout check -----
  if (optoState == OPTO_ACTIVE_SERIAL && nowMs >= optoSerialEndMs) {
    Serial.println(F("[OPTO] Serial test duration complete.  Laser OFF."));
    stopOptoImmediately();
    return;
  }

  // ----- Ramp-down: check for step advancement or completion -----
  if (optoState == OPTO_RAMPDOWN) {
    unsigned long rampElapsed = nowMs - rampDownStartMs;

    // Has the full ramp duration elapsed?
    if (rampElapsed >= rampDownDurationMs) {
      Serial.println(F("[OPTO] === RAMP-DOWN COMPLETE ===  Laser permanently OFF."));
      stopOptoImmediately();
      return;
    }

    // Check whether we have crossed into a new ramp step
    unsigned long stepDurMs = rampDownDurationMs / (unsigned long)rampSteps;
    unsigned int  newStep   = (unsigned int)(rampElapsed / stepDurMs);
    if (newStep >= rampSteps) newStep = rampSteps - 1;  // safety clamp

    if (newStep > currentRampStep) {
      currentRampStep = newStep;
      // Recompute the period: basePeriod × multiplier^step, with overflow guard
      currentRampPeriodMs = safePow(optoPeriodMs, periodMultiplier, currentRampStep);

      Serial.print(F("[OPTO] Ramp-down step ")); Serial.print(currentRampStep + 1);
      Serial.print(F("/")); Serial.print(rampSteps);
      Serial.print(F(" reached  |  New period: ")); Serial.print(currentRampPeriodMs);
      Serial.print(F(" ms  |  Ramp elapsed: ")); Serial.print(rampElapsed / 1000.0, 1);
      Serial.println(F(" s"));
    }
  }

  // ----- Laser ON/OFF toggle cycling -----
  if (nowMs >= laserToggleAtMs) {

    // Determine which period governs this cycle
    unsigned long activePeriod = (optoState == OPTO_RAMPDOWN)
                                 ? currentRampPeriodMs
                                 : optoPeriodMs;

    if (laserPhysicallyOn) {
      // --- Laser was ON; turn it OFF now ---
      setLaserOff();

      // If a ramp-down is pending (Pin 10 went LOW while we were mid-pulse),
      // this is the moment to begin the ramp.
      if (pendingRampDown) {
        pendingRampDown = false;
        beginRampDown(nowMs);
        return;  // beginRampDown starts its own new cycle
      }

      // Schedule the next ON.  The OFF phase lasts for the remainder of the
      // period after subtracting the ON duration.
      if (optoOnDurationMs >= activePeriod) {
        // 100 % duty: turn back on immediately
        digitalWrite(PIN_LASER, HIGH);
        laserPhysicallyOn = true;
        laserToggleAtMs   = nowMs + activePeriod;
      } else {
        unsigned long offDuration = activePeriod - optoOnDurationMs;
        laserToggleAtMs = nowMs + offDuration;  // next ON time
      }

    } else {
      // --- Laser was OFF; turn it ON for a new cycle ---
      digitalWrite(PIN_LASER, HIGH);
      laserPhysicallyOn = true;

      if (optoOnDurationMs >= activePeriod) {
        // 100 % duty: stay on for the full period
        laserToggleAtMs = nowMs + activePeriod;
      } else {
        laserToggleAtMs = nowMs + optoOnDurationMs;
      }
    }
  }
}

// Called whenever the debounced Pin 10 state changes during a running
// experiment.  Decides whether to start pulsing, begin ramp-down, or
// cancel an active ramp-down.
void handleBehavTransition(int newState, unsigned long nowMs) {
  // Only act on Pin 10 transitions while the experiment is running
  if (!running) return;

  if (newState == HIGH) {
    // ---- Pin 10 went HIGH: start or resume opto pulsing ----

    if (optoState == OPTO_RAMPDOWN) {
      // Cancel ramp-down and resume full-rate pulsing
      Serial.println(F("[OPTO] Pin10 -> HIGH during ramp-down.  Cancelling ramp, "
                       "resuming full-rate pulsing."));
      pendingRampDown = false;
      optoState = OPTO_ACTIVE_PIN10;
      startOptoPulseCycle(nowMs, optoPeriodMs);
      return;
    }

    if (optoState == OPTO_ACTIVE_SERIAL) {
      // A serial test is running — don't interfere.  Pin 10 will be
      // honoured once the serial test naturally ends (checked each loop).
      Serial.println(F("[OPTO] Pin10 -> HIGH but serial test is active.  "
                       "Pin10 will take over when test ends."));
      return;
    }

    if (optoState == OPTO_ACTIVE_PIN10) {
      // Already active from Pin 10, nothing to do
      return;
    }

    // Start Pin-10-driven opto pulsing (from IDLE)
    optoState = OPTO_ACTIVE_PIN10;
    Serial.println(F("[OPTO] Pin10 -> HIGH.  Opto pulsing ACTIVATED."));
    Serial.print(F("[OPTO]   Period: ")); Serial.print(optoPeriodMs);
    Serial.print(F(" ms  ON: ")); Serial.print(optoOnDurationMs);
    Serial.println(F(" ms"));
    startOptoPulseCycle(nowMs, optoPeriodMs);

  } else {
    // ---- Pin 10 went LOW: initiate ramp-down ----

    if (optoState == OPTO_ACTIVE_PIN10) {
      if (laserPhysicallyOn) {
        // Laser is mid-pulse: flag for ramp-down after this pulse completes
        pendingRampDown = true;
        Serial.println(F("[OPTO] Pin10 -> LOW (mid-pulse).  "
                         "Will ramp down after current pulse finishes."));
      } else {
        // Laser is in its OFF phase: begin ramp-down immediately
        Serial.println(F("[OPTO] Pin10 -> LOW.  Beginning ramp-down now."));
        beginRampDown(nowMs);
      }
    }
    // If opto is not ACTIVE_PIN10 (e.g. serial test or idle), ignore LOW
  }
}

// =====================================================================
//  CAMERA TRIGGER
// =====================================================================

// Turn camera trigger OFF if its deadline has passed.
void serviceCameraOff(unsigned long nowMs) {
  if (cameraOn && nowMs >= cameraOffAtMs) {
    setCameraOff();
  }
}

// Fire a single camera trigger pulse.  Sets Pin 3 HIGH and schedules the
// OFF time based on exposureHighMs.  Increments the frame counter.
void triggerCameraFrame(unsigned long frameStartAbsMs) {
  digitalWrite(PIN_CAMERA, HIGH);
  cameraOn       = true;
  cameraOffAtMs  = frameStartAbsMs + exposureHighMs;
  framesSent++;
}

// =====================================================================
//  SERIAL COMMAND PARSING
// =====================================================================

// Read one line from serial, trim whitespace, and return it.
String readTrimmedLine() {
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

void handleCommand(const String& line) {
  if (line.length() == 0) return;

  // ---- START EXPERIMENT ----
  if (line.equalsIgnoreCase("s")) {
    if (!running) {
      clipCameraTimings();
      clipOptoTimings();

      Serial.println(F("\n>> START experiment."));
      running              = true;
      expStartMs           = millis();
      lastStatusMs         = expStartMs;
      lastCameraFrameIdx   = -1;
      framesSent           = 0;

      // Ensure outputs are off before we begin
      setCameraOff();
      setLaserOff();
      optoState       = OPTO_IDLE;
      pendingRampDown = false;

      // Reset the behaviour transition log and seed with the current state
      behavEventCount = 0;
      int raw           = digitalRead(PIN_BEHAV);
      lastBehavRaw      = raw;
      debouncedBehavState = raw;
      lastBehavEdgeMs   = millis();
      logBehavTransition(raw, millis());

      Serial.println(F("   Camera triggering ACTIVE.  Pin10 monitoring ACTIVE."));
      printSettings();

      // If Pin 10 is already HIGH at the moment we start, activate opto now
      if (debouncedBehavState == HIGH) {
        Serial.println(F("[OPTO] Pin10 is HIGH at experiment start.  "
                         "Activating opto pulsing immediately."));
        optoState = OPTO_ACTIVE_PIN10;
        startOptoPulseCycle(millis(), optoPeriodMs);
      }
    } else {
      Serial.println(F(">> Already running."));
    }
    return;
  }

  // ---- STOP EVERYTHING ----
  if (line.equalsIgnoreCase("x")) {
    bool anythingWasActive = running || (optoState != OPTO_IDLE);

    if (anythingWasActive) {
      unsigned long nowMs   = millis();
      unsigned long elapsed = running ? (nowMs - expStartMs) : 0UL;

      // Kill everything immediately
      setCameraOff();
      setLaserOff();
      optoState       = OPTO_IDLE;
      pendingRampDown = false;

      Serial.println(F("\n>> STOP.  Everything halted."));

      // If the experiment was running, print a report
      if (running) {
        running = false;
        char buf[24];
        formatHMSms(elapsed, buf, sizeof(buf));
        Serial.println(F("--- Experiment Report ---"));
        Serial.print(F("  Duration          : ")); Serial.println(buf);
        Serial.print(F("  Total seconds     : ")); Serial.println(elapsed / 1000.0, 3);
        Serial.print(F("  Camera frames sent: ")); Serial.println(framesSent);

        // Print behaviour transition log
        if (behavEventCount > 0) {
          Serial.println(F("  Pin10 transitions (state @ ms from start):"));
          for (uint16_t i = 0; i < behavEventCount; i++) {
            Serial.print(F("    "));
            Serial.print(behavEventState[i] == HIGH ? F("HIGH") : F("LOW "));
            Serial.print(F("  @  "));
            Serial.print(behavEventTimeMs[i]);
            Serial.println(F(" ms"));
          }
        } else {
          Serial.println(F("  No Pin10 transitions logged."));
        }
        Serial.println(F("--- End of Report ---\n"));
      } else {
        // Experiment wasn't running, but we stopped a serial test or ramp
        Serial.println(F("  (No experiment was running.  Opto activity stopped.)\n"));
      }
    } else {
      Serial.println(F(">> Nothing to stop."));
    }
    return;
  }

  // ---- HELP ----
  if (line.equalsIgnoreCase("?")) {
    printHelp();
    printSettings();
    return;
  }

  // ---- OPTO SERIAL TEST  (o<seconds> or o <seconds>) ----
  if (line.charAt(0) == 'o' || line.charAt(0) == 'O') {
    // Extract the numeric part (handle both "o5" and "o 5")
    String numPart;
    if (line.length() > 1 && line.charAt(1) == ' ') {
      numPart = line.substring(2);  // "o 5" -> "5"
    } else {
      numPart = line.substring(1);  // "o5"  -> "5"
    }
    numPart.trim();

    if (numPart.length() > 0 && isDigit(numPart.charAt(0))) {
      long seconds = numPart.toInt();
      if (seconds <= 0) {
        Serial.println(F("!! Invalid duration.  Use: o5  or  o 10"));
        return;
      }

      // Conflict checks
      if (optoState == OPTO_ACTIVE_PIN10) {
        Serial.println(F("!! Rejected: opto already active via Pin10."));
        return;
      }
      if (optoState == OPTO_RAMPDOWN) {
        Serial.println(F("!! Rejected: opto is currently in ramp-down."));
        return;
      }
      if (optoState == OPTO_ACTIVE_SERIAL) {
        Serial.println(F("!! Rejected: a serial opto test is already running."));
        return;
      }

      clipOptoTimings();
      optoState       = OPTO_ACTIVE_SERIAL;
      optoSerialEndMs = millis() + (unsigned long)seconds * 1000UL;

      Serial.print(F("[OPTO] Serial test: pulsing for ")); Serial.print(seconds);
      Serial.print(F(" s  (period ")); Serial.print(optoPeriodMs);
      Serial.print(F(" ms, ON ")); Serial.print(optoOnDurationMs);
      Serial.println(F(" ms).  No ramp-down at end."));

      startOptoPulseCycle(millis(), optoPeriodMs);
      return;
    }
    // If we get here, it wasn't a valid "o" command -- fall through to unknown
  }

  // ---- CAMERA FRAME PERIOD ----
  if (line.startsWith("f ") || line.startsWith("F ")) {
    long v = line.substring(2).toInt();
    if (v > 0) {
      framePeriodMs = (unsigned long)v;
      clipCameraTimings();
      Serial.print(F(">> Camera frame period -> ")); Serial.print(framePeriodMs);
      Serial.print(F(" ms  (")); Serial.print(1000.0f / (float)framePeriodMs, 2);
      Serial.println(F(" Hz)"));
    } else {
      Serial.println(F("!! Invalid.  Use: f 200"));
    }
    return;
  }

  // ---- CAMERA EXPOSURE HIGH ----
  if (line.startsWith("h ") || line.startsWith("H ")) {
    long v = line.substring(2).toInt();
    if (v > 0) {
      exposureHighMs = (unsigned long)v;
      clipCameraTimings();
      Serial.print(F(">> Camera exposure HIGH -> ")); Serial.print(exposureHighMs);
      Serial.println(F(" ms"));
    } else {
      Serial.println(F("!! Invalid.  Use: h 170"));
    }
    return;
  }

  // ---- OPTO LASER PERIOD ----
  if (line.startsWith("p ") || line.startsWith("P ")) {
    long v = line.substring(2).toInt();
    if (v > 0) {
      optoPeriodMs = (unsigned long)v;
      clipOptoTimings();
      Serial.print(F(">> Opto period -> ")); Serial.print(optoPeriodMs);
      Serial.println(F(" ms"));
    } else {
      Serial.println(F("!! Invalid.  Use: p 800"));
    }
    return;
  }

  // ---- OPTO LASER ON DURATION ----
  if (line.startsWith("d ") || line.startsWith("D ")) {
    long v = line.substring(2).toInt();
    if (v > 0) {
      optoOnDurationMs = (unsigned long)v;
      clipOptoTimings();
      Serial.print(F(">> Opto ON duration -> ")); Serial.print(optoOnDurationMs);
      Serial.println(F(" ms"));
      if (optoOnDurationMs >= optoPeriodMs) {
        Serial.println(F("   Note: ON duration >= period.  "
                         "Laser will stay on continuously (100% duty)."));
      }
    } else {
      Serial.println(F("!! Invalid.  Use: d 200"));
    }
    return;
  }

  // ---- RAMP-DOWN TOTAL DURATION ----
  if (line.startsWith("r ") || line.startsWith("R ")) {
    long v = line.substring(2).toInt();
    if (v > 0) {
      rampDownDurationMs = (unsigned long)v;
      Serial.print(F(">> Ramp-down duration -> ")); Serial.print(rampDownDurationMs);
      Serial.print(F(" ms  (")); Serial.print(rampDownDurationMs / 1000.0, 1);
      Serial.println(F(" s)"));
    } else {
      Serial.println(F("!! Invalid.  Use: r 1500000"));
    }
    return;
  }

  // ---- RAMP-DOWN STEP COUNT ----
  if (line.startsWith("n ") || line.startsWith("N ")) {
    long v = line.substring(2).toInt();
    if (v >= 1) {
      rampSteps = (unsigned int)v;
      Serial.print(F(">> Ramp-down steps -> ")); Serial.println(rampSteps);
    } else {
      Serial.println(F("!! Invalid (must be >= 1).  Use: n 5"));
    }
    return;
  }

  // ---- RAMP-DOWN PERIOD MULTIPLIER ----
  if (line.startsWith("m ") || line.startsWith("M ")) {
    long v = line.substring(2).toInt();
    if (v >= 2) {
      periodMultiplier = (unsigned int)v;
      Serial.print(F(">> Ramp-down period multiplier -> ")); Serial.println(periodMultiplier);
    } else {
      Serial.println(F("!! Invalid (must be >= 2).  Use: m 2"));
    }
    return;
  }

  Serial.println(F("!! Unknown command.  Type '?' for help."));
}

// =====================================================================
//  WARM-UP SEQUENCE  (runs once in setup, blocking is OK here)
// =====================================================================
// Fires 3 camera trigger pulses and 1 laser test pulse to verify that
// the hardware responds before real data collection begins.

void warmupSequence() {
  Serial.println(F("\n--- Warm-up: 3 camera triggers + 1 laser test ---"));
  clipCameraTimings();
  clipOptoTimings();

  // Fire 3 camera trigger pulses at the configured timing
  for (int i = 0; i < 3; i++) {
    unsigned long fStart = millis();
    digitalWrite(PIN_CAMERA, HIGH);

    // Blocking spin for the exposure HIGH duration (acceptable in setup)
    while (millis() < fStart + exposureHighMs) { /* spin */ }
    digitalWrite(PIN_CAMERA, LOW);

    // Wait for remainder of the frame period
    while (millis() < fStart + framePeriodMs) { /* spin */ }

    Serial.print(F("  Camera trigger ")); Serial.print(i + 1);
    Serial.println(F("/3 fired."));
  }

  // Fire one laser test pulse
  Serial.print(F("  Laser test pulse ("));
  Serial.print(optoOnDurationMs); Serial.println(F(" ms)..."));
  unsigned long lStart = millis();
  digitalWrite(PIN_LASER, HIGH);
  while (millis() < lStart + optoOnDurationMs) { /* spin */ }
  digitalWrite(PIN_LASER, LOW);
  Serial.println(F("  Laser test pulse complete."));

  Serial.println(F("--- Warm-up done ---\n"));
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  // Configure pin modes
  pinMode(PIN_CAMERA, OUTPUT);
  pinMode(PIN_LASER,  OUTPUT);
  pinMode(PIN_BEHAV,  INPUT);   // Clean TTL from external behaviour/VR system

  // Start with all outputs LOW
  setCameraOff();
  setLaserOff();

  // Initialise serial
  Serial.begin(115200);
  while (!Serial) { ; }  // Wait for USB serial (needed on Leonardo / Micro)

  Serial.println(F("\n============================================"));
  Serial.println(F("  Widefield Camera + Opto Laser Controller"));
  Serial.println(F("  v4 — Independent camera & laser clocks"));
  Serial.println(F("============================================"));
  printHelp();
  printSettings();

  // Seed the Pin 10 debounce state with the current reading
  int raw            = digitalRead(PIN_BEHAV);
  lastBehavRaw       = raw;
  debouncedBehavState = raw;
  lastBehavEdgeMs    = millis();

  // Run hardware warm-up
  warmupSequence();

  Serial.println(F("Ready.  Press 's' to START, '?' for help."));
  Serial.print(F("Pin10 current state: "));
  Serial.println(digitalRead(PIN_BEHAV) == HIGH ? F("HIGH") : F("LOW"));
  Serial.println();
}

// =====================================================================
//  MAIN LOOP
// =====================================================================
void loop() {

  // ---- 1. Process any incoming serial commands ----
  while (Serial.available()) {
    String line = readTrimmedLine();
    if (line.length()) handleCommand(line);
  }

  unsigned long nowMs = millis();

  // ---- 2. Monitor Pin 10 (debounced) ----
  // Check for state transitions and act on them (activate opto or ramp down).
  bool behavChanged = sampleBehavDebounced(nowMs);
  if (behavChanged) {
    Serial.print(F("[PIN10] Transition -> "));
    Serial.println(debouncedBehavState == HIGH ? F("HIGH") : F("LOW"));
    handleBehavTransition(debouncedBehavState, nowMs);
  }

  // ---- 3. Service the opto laser state machine ----
  // This handles serial-test timeouts, ramp-down step progression, and the
  // ON/OFF pulse cycle independently of the camera.
  serviceOpto(nowMs);

  // ---- 4. Service the camera trigger (only while experiment runs) ----
  if (running) {
    // Turn camera OFF if its deadline has passed
    serviceCameraOff(nowMs);

    // Absolute-time camera frame scheduler.
    // We compute which frame *should* have been triggered by now based on
    // elapsed time from the experiment start.  If the loop lagged, the
    // catch-up while-loop ensures no frames are skipped.
    long idxNow = (long)((nowMs - expStartMs) / framePeriodMs);
    while (idxNow > lastCameraFrameIdx) {
      lastCameraFrameIdx++;
      unsigned long frameStartAbs = expStartMs
                                    + (unsigned long)lastCameraFrameIdx * framePeriodMs;
      triggerCameraFrame(frameStartAbs);
    }

    // Print periodic status every STATUS_INTERVAL_MS
    if (nowMs - lastStatusMs >= STATUS_INTERVAL_MS) {
      printStatusLine(nowMs);
      lastStatusMs = nowMs;
    }

  } else {
    // Not running: still service camera OFF in case we just stopped mid-pulse
    serviceCameraOff(nowMs);
  }
}
