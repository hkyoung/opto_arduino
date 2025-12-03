/*
  Widefield + Strobing Fluorescence + Optogenetic Control (non-blocking, millis-based)
  Implements: absolute-timed frame scheduler.
  Pattern: Three-block experiment (440 seconds total):
    - Pre-green block (100s): GREEN always ON (pulsed), RED OFF
    - Middle block (240s): GREEN always ON (pulsed), RED Opto (20s OFF, 10s ON) x 8 cycles
    - Post-green block (100s): GREEN always ON (pulsed), RED OFF
  BLUE always OFF.
  During RED ON blocks, RED pulses simultaneously with GREEN.
  
  Pins (per user spec):
    3  - Camera exposure/frame-start trigger (OUTPUT, DIGITAL; active HIGH)
    10 - Behavior/VR switch (INPUT, DIGITAL) - Logged only
    11 - BLUE LED  (OUTPUT, DIGITAL) - OFF
    12 - GREEN LED (OUTPUT, DIGITAL) - Widefield/IOS/vascular (ON every frame)
    13 - RED LED   (OUTPUT, DIGITAL) - Optogenetics (Timed pattern)

  Defaults:
    framePeriodMs  = 250 (4 Hz)
    exposureHighMs = 100
    first frame is BLUE

  Serial commands (115200 baud):
    s                -> start experiment
    x                -> stop experiment + report
    f <ms>           -> set frame period (ms)
    h <ms>           -> set exposure HIGH width (ms)
    c blue|green     -> set first color (default blue)
    ?                -> help + current settings
*/

#include <Arduino.h>

// ---- Pin map ----
const uint8_t PIN_CAMERA = 3;
const uint8_t PIN_BLUE   = 11;
const uint8_t PIN_GREEN  = 12;
const uint8_t PIN_RED    = 13;

// ---- Config (user changeable at runtime) ----
unsigned long framePeriodMs   = 250UL; // default 4 Hz
unsigned long exposureHighMs  = 100UL; // HIGH width for camera + LEDs
bool firstFrameBlue           = false; // Unused in new pattern

// ---- Run state ----
volatile bool running         = false;
unsigned long expStartMs      = 0;     // experiment start (millis)
unsigned long lastStatusMs    = 0;
unsigned long framesSent      = 0;     // count of exposure HIGH pulses (frames)

// Pulse state (for turning things OFF on time)
bool exposureOn               = false;
unsigned long exposureOffAtMs = 0;
bool blueOn                   = false;
bool greenOn                  = false;
bool redOn                    = false;
unsigned long blueOffAtMs     = 0;
unsigned long greenOffAtMs    = 0;
unsigned long redOffAtMs      = 0;
bool inRedBlock               = false;

// Frame scheduler (absolute to expStartMs)
long lastProcessedFrameIndex  = -1;    // last frame index we triggered

// Status interval
const unsigned long STATUS_INTERVAL_MS = 10000UL;

// Experiment timing constants
const unsigned long PRE_GREEN_MS = 100000UL;   // 100 seconds pre-green block
const unsigned long RED_PHASE_MS = 240000UL;   // 240 seconds middle block (8 cycles of 30s)
const unsigned long POST_GREEN_MS = 100000UL;  // 100 seconds post-green block
const unsigned long TOTAL_EXP_MS = PRE_GREEN_MS + RED_PHASE_MS + POST_GREEN_MS; // 440 seconds total

// --- Utilities ---
void setAllLedsLow() {
  digitalWrite(PIN_BLUE,  LOW);
  digitalWrite(PIN_GREEN, LOW);
  digitalWrite(PIN_RED,   LOW);
  blueOn = greenOn = redOn = false;
}

void setCameraLow() {
  digitalWrite(PIN_CAMERA, LOW);
  exposureOn = false;
}

void clipTimings() {
  if (framePeriodMs < 1UL) framePeriodMs = 1UL;
  if (exposureHighMs < 1UL) exposureHighMs = 1UL;
  if (exposureHighMs > framePeriodMs) exposureHighMs = framePeriodMs; // enforce
}

void printHelp() {
  Serial.println(F("\n=== Controls ==="));
  Serial.println(F(" s            : start experiment"));
  Serial.println(F(" x            : stop experiment and print report"));
  Serial.println(F(" f <ms>       : set frame period (ms), e.g. 'f 125'"));
  Serial.println(F(" h <ms>       : set exposure HIGH width (ms), e.g. 'h 100'"));
  Serial.println(F(" c blue|green : (Disabled in this version)"));
  Serial.println(F(" ?            : show help & current settings\n"));
}

void printSettings() {
  Serial.print(F(" Settings -> framePeriodMs: ")); Serial.print(framePeriodMs); Serial.print(F(" ms  ("));
  if (framePeriodMs > 0) {
    float hz = 1000.0f / (float)framePeriodMs;
    Serial.print(hz, 3); Serial.print(F(" Hz"));
  } else {
    Serial.print(F("n/a Hz"));
  }
  Serial.println(F(")"));
  Serial.print(F("             exposureHighMs: ")); Serial.print(exposureHighMs); Serial.println(F(" ms"));
  Serial.println(F("             Pattern       : Pre-green (100s) + Middle (240s: RED 20s OFF/10s ON x8) + Post-green (100s) = 440s total"));
}

void formatHMSms(unsigned long totalMs, char* out, size_t n) {
  unsigned long ms  = totalMs % 1000UL;
  unsigned long s   = (totalMs / 1000UL) % 60UL;
  unsigned long m   = (totalMs / 60000UL) % 60UL;
  unsigned long h   = (totalMs / 3600000UL);
  snprintf(out, n, "%02lu:%02lu:%02lu:%03lu", h, m, s, ms);
}

void statusLine(unsigned long nowMs) {
  unsigned long elapsed = running ? (nowMs - expStartMs) : 0UL;
  char buf[24];
  formatHMSms(elapsed, buf, sizeof(buf));
  Serial.print(F("[STATUS] Elapsed ")); Serial.print(buf);
  Serial.print(F(" | Frames sent: ")); Serial.println(framesSent);
}

// Turn OFF things whose deadlines have passed
void serviceOffEvents(unsigned long nowMs) {
  if (exposureOn && nowMs >= exposureOffAtMs) {
    setCameraLow();
  }
  if (blueOn && nowMs >= blueOffAtMs) {
    digitalWrite(PIN_BLUE, LOW); blueOn = false;
  }
  if (greenOn && nowMs >= greenOffAtMs) {
    digitalWrite(PIN_GREEN, LOW); greenOn = false;
  }
  if (redOn && nowMs >= redOffAtMs) {
    digitalWrite(PIN_RED, LOW); redOn = false;
  }
}

// Forward declaration
void stopExperiment(unsigned long nowMs);

// Launch one scheduled frame at its *absolute* start time
void triggerFrame(unsigned long frameStartAbsMs, long frameIndex) {
  // Exposure HIGH
  digitalWrite(PIN_CAMERA, HIGH);
  exposureOn      = true;
  exposureOffAtMs = frameStartAbsMs + exposureHighMs;

  // 1) BLUE is always OFF
  digitalWrite(PIN_BLUE, LOW);
  blueOn = false;

  // 2) GREEN is always ON (pulsing)
  digitalWrite(PIN_GREEN, HIGH);
  greenOn = true;
  greenOffAtMs = frameStartAbsMs + exposureHighMs;

  // 3) RED Logic: Three-block pattern
  unsigned long elapsed = frameStartAbsMs - expStartMs;
  
  // Check if experiment duration exceeded
  if (elapsed >= TOTAL_EXP_MS) {
    if (running) stopExperiment(millis());
    return;
  }

  // Determine which block we're in
  if (elapsed < PRE_GREEN_MS) {
    // Pre-green block: RED OFF
    digitalWrite(PIN_RED, LOW);
    redOn = false;
    if (inRedBlock) {
      inRedBlock = false;
      Serial.println(F(">> RED OFF"));
    }
  } else if (elapsed < PRE_GREEN_MS + RED_PHASE_MS) {
    // Middle block: RED logic (20s OFF, 10s ON) x 8 cycles
    // Calculate elapsed time relative to start of middle block
    unsigned long middleElapsed = elapsed - PRE_GREEN_MS;
    unsigned long cycleTime = middleElapsed % 30000UL;
    bool currentRedBlock = (cycleTime >= 20000UL); // within the ON window of a cycle

    if (currentRedBlock != inRedBlock) {
      inRedBlock = currentRedBlock;
      if (inRedBlock) {
         unsigned long cycleNum = (middleElapsed / 30000UL) + 1;
         Serial.print(F(">> RED ON (Block ")); Serial.print(cycleNum); Serial.println(F(") - Green continues pulsing"));
      } else {
         Serial.println(F(">> RED OFF"));
      }
    }

    if (inRedBlock) {
      digitalWrite(PIN_RED, HIGH);
      redOn = true;
      redOffAtMs = frameStartAbsMs + exposureHighMs;
    } else {
      digitalWrite(PIN_RED, LOW);
      redOn = false;
    }
  } else {
    // Post-green block: RED OFF
    digitalWrite(PIN_RED, LOW);
    redOn = false;
    if (inRedBlock) {
      inRedBlock = false;
      Serial.println(F(">> RED OFF"));
    }
  }

  framesSent++;
}

// Serial parsing helpers
String readTrimmedLine() {
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

void stopExperiment(unsigned long nowMs) {
  running = false;
  unsigned long elapsed = nowMs - expStartMs;
  setCameraLow();
  setAllLedsLow();
  Serial.println(F("\n>> STOP (Final Report):"));
  char buf[24];
  formatHMSms(elapsed, buf, sizeof(buf));
  Serial.print(F(" Duration (HH:MM:SS:ms): ")); Serial.println(buf);
  Serial.print(F(" Total seconds: ")); Serial.println(elapsed / 1000.0, 3);
  Serial.print(F(" Frames (exposure HIGH) sent: ")); Serial.println(framesSent);
  Serial.println();
}

void handleCommand(const String& line) {
  if (line.length() == 0) return;
  if (line.equalsIgnoreCase("s")) {
    if (!running) {
      clipTimings();
      Serial.println(F(">> START requested."));
      running = true;
      expStartMs = millis();
      lastStatusMs = expStartMs;
      lastProcessedFrameIndex = -1;
      framesSent = 0;

      // Reset outputs/pulse states
      setCameraLow();
      setAllLedsLow();
      inRedBlock = false;

      Serial.println(F("Experiment running..."));
      printSettings();
    } else {
      Serial.println(F(">> Already running."));
    }
    return;
  }
  if (line.equalsIgnoreCase("x")) {
    if (running) {
      stopExperiment(millis());
    } else {
      Serial.println(F(">> Not running."));
    }
    return;
  }

  if (line.equalsIgnoreCase("?")) {
    printHelp();
    printSettings();
    return;
  }

  if (line.startsWith("f ") || line.startsWith("F ")) {
    long v = line.substring(2).toInt();
    if (v > 0) {
      framePeriodMs = (unsigned long)v;
      clipTimings();
      Serial.print(F(">> framePeriodMs set to ")); Serial.print(framePeriodMs); Serial.println(F(" ms"));
    } else {
      Serial.println(F("!! Invalid frame period. Use: f 125"));
    }
    return;
  }

  if (line.startsWith("h ") || line.startsWith("H ")) {
    long v = line.substring(2).toInt();
    if (v > 0) {
      exposureHighMs = (unsigned long)v;
      clipTimings();
      Serial.print(F(">> exposureHighMs set to ")); Serial.print(exposureHighMs); Serial.println(F(" ms"));
    } else {
      Serial.println(F("!! Invalid HIGH width. Use: h 100"));
    }
    return;
  }

  if (line.startsWith("c ") || line.startsWith("C ")) {
    Serial.println(F("!! Color change disabled. Pattern is fixed to GREEN only + Opto schedule."));
    return;
  }

  Serial.println(F("!! Unknown command. Type '?' for help."));
}

// ---- Warm-up in setup(): 6 frames, GREEN only ----
void warmupSequenceNonBlocking() {
  Serial.println(F("\nWarm-up: 6 frames @ current timing (GREEN only)..."));
  clipTimings();
  unsigned long start = millis();
  long sent = -1;

  while (sent < 5) {
    unsigned long now = millis();
    long idxNow = (long)((now - start) / framePeriodMs);
    while (idxNow > sent && sent < 5) {
      sent++;
      unsigned long frameStart = start + (unsigned long)sent * framePeriodMs;

      // Exposure HIGH
      digitalWrite(PIN_CAMERA, HIGH);
      unsigned long offAt = frameStart + exposureHighMs;

      // LEDs - Always GREEN for warmup
      digitalWrite(PIN_GREEN, HIGH);

      // Hold
      while (millis() < offAt) { /* spin */ }

      // Turn off
      digitalWrite(PIN_CAMERA, LOW);
      digitalWrite(PIN_GREEN,  LOW);
      digitalWrite(PIN_RED,    LOW);

      // Wait for frame end
      unsigned long frameEnd = frameStart + framePeriodMs;
      while (millis() < frameEnd) { /* spin */ }

      Serial.print(F(" Warm-up frame ")); Serial.print(sent + 1);
      Serial.println(F(": GREEN"));
    }
  }
  Serial.println(F("Warm-up done.\n"));
}

void setup() {
  pinMode(PIN_CAMERA, OUTPUT);
  pinMode(PIN_BLUE,   OUTPUT);
  pinMode(PIN_GREEN,  OUTPUT);
  pinMode(PIN_RED,    OUTPUT);

  setCameraLow();
  setAllLedsLow();

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println(F("\n== Widefield/Fluorescence/Opto Controller =="));
  printHelp();
  printSettings();

  // Warm-up sequence
  warmupSequenceNonBlocking();

  Serial.println(F("Press 's' to START, '?' for help."));
}

void loop() {
  // ---- Serial command handling ----
  while (Serial.available()) {
    String line = readTrimmedLine();
    if (line.length()) handleCommand(line);
  }

  unsigned long nowMs = millis();

  // ---- Running experiment ----
  if (running) {
    // 1) Service OFF deadlines first to get exact on-times
    serviceOffEvents(nowMs);

    // 2) Absolute frame scheduler based on start time (catch-up if loop lagged)
    long idxNow = (long)((nowMs - expStartMs) / framePeriodMs);

    // Process any frames we owe up to idxNow
    while (idxNow > lastProcessedFrameIndex) {
      lastProcessedFrameIndex++;
      unsigned long frameStartAbs = expStartMs + (unsigned long)lastProcessedFrameIndex * framePeriodMs;

      triggerFrame(frameStartAbs, lastProcessedFrameIndex);
    }

    // 3) Periodic status
    if (nowMs - lastStatusMs >= STATUS_INTERVAL_MS) {
      statusLine(nowMs);
      lastStatusMs = nowMs;
    }
  } else {
    // Not running: ensure everything is off
    serviceOffEvents(nowMs);
  }
}