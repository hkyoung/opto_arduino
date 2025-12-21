/*
  Widefield + Yellow Laser + Optogenetic Control (non-blocking, millis-based)
  Implements: absolute-timed frame scheduler, GREEN every frame, YELLOW+RED when Pin10=HIGH,
  warm-up sequence, serial start/stop and live params, 10s status updates,
  Pin10 transition logging with a 5 ms filter (for logging only), and detailed stop report.

  Pins (per user spec):
    3  - Camera exposure/frame-start trigger (OUTPUT, DIGITAL; active HIGH)
    10 - Behavior/VR switch (INPUT, DIGITAL) - HIGH enables YELLOW+RED during frames
    11 - YELLOW LASER (OUTPUT, DIGITAL) - co-activated when Pin10=HIGH
    12 - GREEN LED (OUTPUT, DIGITAL) - Widefield/IOS/vascular (on every frame)
    13 - RED LED   (OUTPUT, DIGITAL) - Optogenetics (co-activated when Pin10=HIGH)

  Defaults:
    framePeriodMs  = 200 (5 Hz)
    exposureHighMs = 175

  Serial commands (115200 baud):
    s                -> start experiment
    x                -> stop experiment + report
    f <ms>           -> set frame period (ms)
    h <ms>           -> set exposure HIGH width (ms)
    ?                -> help + current settings
*/

#include <Arduino.h>

// ---- Pin map ----
const uint8_t PIN_CAMERA = 3;
const uint8_t PIN_BEHAV  = 10;
const uint8_t PIN_YELLOW = 11;  // Was BLUE, now YELLOW LASER
const uint8_t PIN_GREEN  = 12;
const uint8_t PIN_RED    = 13;

// ---- Config (user changeable at runtime) ----
unsigned long framePeriodMs   = 200UL; // default 5 Hz
unsigned long exposureHighMs  = 175UL; // HIGH width for camera + LEDs

// ---- Run state ----
volatile bool running         = false;
unsigned long expStartMs      = 0;     // experiment start (millis)
unsigned long lastStatusMs    = 0;
unsigned long framesSent      = 0;     // count of exposure HIGH pulses (frames)

// Pulse state (for turning things OFF on time)
bool exposureOn               = false;
unsigned long exposureOffAtMs = 0;
bool yellowOn                 = false;
bool greenOn                  = false;
bool redOn                    = false;
unsigned long yellowOffAtMs   = 0;
unsigned long greenOffAtMs    = 0;
unsigned long redOffAtMs      = 0;

// Frame scheduler (absolute to expStartMs)
long lastProcessedFrameIndex  = -1;    // last frame index we triggered

// Behavior switch (Pin 10) logging with 5 ms filter (logging only)
const unsigned long BEHAV_FILTER_MS = 5UL; // does NOT affect scheduling/LED logic
int lastBehavSample          = LOW;
int debouncedBehavState      = LOW;
unsigned long lastBehavSampleTime = 0;
unsigned long lastDebounceChange  = 0;

const uint16_t MAX_BEHAV_EVENTS = 64;
unsigned long behavChangeTimeMs[MAX_BEHAV_EVENTS]; // relative to exp start when running; absolute before start
int           behavChangeState[MAX_BEHAV_EVENTS];
uint16_t      behavEventCount  = 0;

// Status interval
const unsigned long STATUS_INTERVAL_MS = 10000UL;

// --- Utilities ---
void setAllLedsLow() {
  digitalWrite(PIN_YELLOW, LOW);
  digitalWrite(PIN_GREEN,  LOW);
  digitalWrite(PIN_RED,    LOW);
  yellowOn = greenOn = redOn = false;
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
  Serial.println(F(" f <ms>       : set frame period (ms), e.g. 'f 250'"));
  Serial.println(F(" h <ms>       : set exposure HIGH width (ms), e.g. 'h 175'"));
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
  Serial.println(F("             GREEN on every frame"));
  Serial.println(F("             YELLOW+RED on when VR switch (Pin10) HIGH"));
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

void logBehavTransition(int newState, unsigned long nowMs) {
  if (behavEventCount < MAX_BEHAV_EVENTS) {
    behavChangeState[behavEventCount] = newState;
    behavChangeTimeMs[behavEventCount] = nowMs - (running ? expStartMs : 0UL); // relative to experiment if running
    behavEventCount++;
  }
}

// Turn OFF things whose deadlines have passed
void serviceOffEvents(unsigned long nowMs) {
  if (exposureOn && nowMs >= exposureOffAtMs) {
    setCameraLow();
  }
  if (yellowOn && nowMs >= yellowOffAtMs) {
    digitalWrite(PIN_YELLOW, LOW); yellowOn = false;
  }
  if (greenOn && nowMs >= greenOffAtMs) {
    digitalWrite(PIN_GREEN, LOW); greenOn = false;
  }
  if (redOn && nowMs >= redOffAtMs) {
    digitalWrite(PIN_RED, LOW); redOn = false;
  }
}

// Debounced/logging-only update for Pin 10 (5 ms filter)
void sampleBehavForLog(unsigned long nowMs) {
  int raw = digitalRead(PIN_BEHAV);
  if (raw != lastBehavSample) {
    lastBehavSample = raw;
    lastDebounceChange = nowMs; // start filter window
  }
  if ((nowMs - lastDebounceChange) >= BEHAV_FILTER_MS) {
    if (debouncedBehavState != raw) {
      debouncedBehavState = raw;
      logBehavTransition(debouncedBehavState, nowMs);
    }
  }
}

// Launch one scheduled frame at its *absolute* start time
void triggerFrame(unsigned long frameStartAbsMs, long frameIndex, int behavStateAtStart) {
  // Exposure HIGH
  digitalWrite(PIN_CAMERA, HIGH);
  exposureOn      = true;
  exposureOffAtMs = frameStartAbsMs + exposureHighMs;

  // GREEN is always on every frame
  digitalWrite(PIN_GREEN, HIGH);
  greenOn = true;
  greenOffAtMs = frameStartAbsMs + exposureHighMs;

  // If behavior switch is HIGH, co-activate YELLOW and RED (exactly exposure window)
  if (behavStateAtStart == HIGH) {
    digitalWrite(PIN_YELLOW, HIGH);
    yellowOn = true;
    yellowOffAtMs = frameStartAbsMs + exposureHighMs;

    digitalWrite(PIN_RED, HIGH);
    redOn = true;
    redOffAtMs = frameStartAbsMs + exposureHighMs;
  }

  framesSent++;
}

// Serial parsing helpers
String readTrimmedLine() {
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
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

      // Reset behavior log and seed with current debounced state
      behavEventCount = 0;
      int raw = digitalRead(PIN_BEHAV);
      lastBehavSample = raw;
      debouncedBehavState = raw;
      lastDebounceChange = millis();
      logBehavTransition(debouncedBehavState, millis());

      Serial.println(F("Experiment running..."));
      printSettings();
    } else {
      Serial.println(F(">> Already running."));
    }
    return;
  }
  if (line.equalsIgnoreCase("x")) {
    if (running) {
      running = false;
      unsigned long nowMs = millis();
      unsigned long elapsed = nowMs - expStartMs;
      setCameraLow();
      setAllLedsLow();
      Serial.println(F("\n>> STOP requested. Final report:"));
      char buf[24];
      formatHMSms(elapsed, buf, sizeof(buf));
      Serial.print(F(" Duration (HH:MM:SS:ms): ")); Serial.println(buf);
      Serial.print(F(" Total seconds: ")); Serial.println(elapsed / 1000.0, 3);
      Serial.print(F(" Frames (exposure HIGH) sent: ")); Serial.println(framesSent);

      // Print behavior transitions (relative to experiment start)
      if (behavEventCount > 0) {
        Serial.println(F(" Behavior switch (Pin 10) transitions (state @ time_from_start_ms):"));
        for (uint16_t i = 0; i < behavEventCount; i++) {
          Serial.print(F("  "));
          Serial.print(behavChangeState[i] == HIGH ? F("HIGH") : F("LOW"));
          Serial.print(F(" @ "));
          Serial.print(behavChangeTimeMs[i]);
          Serial.println(F(" ms"));
        }
        // Also explicitly show first HIGH and next LOW after it
        long firstHighIdx = -1, nextLowIdx = -1;
        for (uint16_t i = 0; i < behavEventCount; i++) {
          if (behavChangeState[i] == HIGH) { firstHighIdx = i; break; }
        }
        if (firstHighIdx >= 0) {
          for (uint16_t j = firstHighIdx + 1; j < behavEventCount; j++) {
            if (behavChangeState[j] == LOW) { nextLowIdx = j; break; }
          }
        }
        if (firstHighIdx >= 0) {
          Serial.print(F(" First HIGH at: ")); Serial.print(behavChangeTimeMs[firstHighIdx]); Serial.println(F(" ms from start"));
          if (nextLowIdx >= 0) {
            Serial.print(F(" First LOW after that at: ")); Serial.print(behavChangeTimeMs[nextLowIdx]); Serial.println(F(" ms from start"));
          } else {
            Serial.println(F(" No subsequent LOW after first HIGH during run."));
          }
        }
      } else {
        Serial.println(F(" No behavior transitions logged."));
      }
      Serial.println();
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
      Serial.println(F("!! Invalid frame period. Use: f 200"));
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
      Serial.println(F("!! Invalid HIGH width. Use: h 175"));
    }
    return;
  }

  Serial.println(F("!! Unknown command. Type '?' for help."));
}

// ---- Warm-up in setup(): 6 frames, GREEN every frame; on 4th frame add YELLOW+RED ----
void warmupSequenceNonBlocking() {
  Serial.println(F("\nWarm-up: 6 frames @ current timing (GREEN every frame; 4th = GREEN+YELLOW+RED)..."));
  clipTimings();
  unsigned long start = millis();
  long sent = -1;

  while (sent < 5) {
    unsigned long now = millis();
    long idxNow = (long)((now - start) / framePeriodMs);
    while (idxNow > sent && sent < 5) {
      sent++;
      unsigned long frameStart = start + (unsigned long)sent * framePeriodMs;

      // On 4th frame (index 3), add YELLOW+RED
      bool addYellowRed = (sent == 3);

      // Exposure HIGH window
      digitalWrite(PIN_CAMERA, HIGH);
      unsigned long offAt = frameStart + exposureHighMs;

      // GREEN always on
      digitalWrite(PIN_GREEN, HIGH);
      if (addYellowRed) {
        digitalWrite(PIN_YELLOW, HIGH);
        digitalWrite(PIN_RED, HIGH);
      }

      // Hold to exact off time (tight spin is acceptable in setup only)
      while (millis() < offAt) { /* spin */ }

      // Turn off
      digitalWrite(PIN_CAMERA, LOW);
      digitalWrite(PIN_GREEN,  LOW);
      digitalWrite(PIN_YELLOW, LOW);
      digitalWrite(PIN_RED,    LOW);

      // Ensure full frame period elapses
      unsigned long frameEnd = frameStart + framePeriodMs;
      while (millis() < frameEnd) { /* spin */ }

      Serial.print(F(" Warm-up frame ")); Serial.print(sent + 1);
      Serial.print(F(": ")); Serial.println(addYellowRed ? F("GREEN+YELLOW+RED") : F("GREEN"));
    }
  }
  Serial.println(F("Warm-up done.\n"));
}

void setup() {
  pinMode(PIN_CAMERA, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_GREEN,  OUTPUT);
  pinMode(PIN_RED,    OUTPUT);
  pinMode(PIN_BEHAV,  INPUT); // clean TTL

  setCameraLow();
  setAllLedsLow();

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println(F("\n== Widefield/Yellow Laser/Opto Controller v2 =="));
  printHelp();
  printSettings();

  // Initialize behavior filter/log (absolute time until run starts)
  int raw = digitalRead(PIN_BEHAV);
  lastBehavSample = raw;
  debouncedBehavState = raw;
  lastDebounceChange = millis();
  logBehavTransition(debouncedBehavState, millis());

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

  // ---- Behavior input monitoring (5 ms filtered for LOGGING only) ----
  sampleBehavForLog(nowMs);
  // Note: LED gating uses the *instantaneous* state sampled at exact frame start
  // (i.e., debouncing does not alter frame timing or gating)

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

      // Sample *raw* behavior state at the instant the frame starts (no debounce; clean TTL)
      int behavAtStart = digitalRead(PIN_BEHAV);
      triggerFrame(frameStartAbs, lastProcessedFrameIndex, behavAtStart);
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

