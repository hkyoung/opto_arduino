/*
  ============================================================================================
   VR_Opto_v5  —  Volume-locked optogenetic stimulation controller
   Target: Arduino Uno  (ATmega328P @ 16 MHz)
  ============================================================================================

   !!  SERIAL MONITOR MUST BE SET TO  250000  BAUD  !!
       At the Arduino IDE default of 9600 you will see only garbage characters.
       250000 baud is used instead of 115200 because 115200 has a 2.1 % baud-rate error on a
       16 MHz AVR, whereas 250000 divides the 16 MHz clock exactly (UBRR = 3, error = 0.0 %).
       It also halves the per-character cost, from 86.8 us to 40.0 us.

  --------------------------------------------------------------------------------------------
   WHAT THIS FIRMWARE DOES
  --------------------------------------------------------------------------------------------
   It watches an incoming two-photon frame clock, reconstructs the volume structure of the
   acquisition from it, and delivers an optogenetic stimulation pulse of a fixed, user-set
   millisecond duration at one specific frame position inside every Nth volume.

   The stimulation pulse duration is deliberately NOT tied to the frame clock. Only the pulse
   TRIGGER is frame-locked; the pulse LENGTH is a hardware timer in milliseconds. Frame rate
   therefore never enters any timing calculation.

  --------------------------------------------------------------------------------------------
   PIN MAP  (unchanged from VR_Opto_4 — existing wiring is correct)
  --------------------------------------------------------------------------------------------
     Pin 6   INPUT    Frame clock from the microscope. Rising edges are counted.  (PD6/PCINT22)
     Pin 9   INPUT    Enable / behaviour-VR TTL gate.  HIGH = stimulation permitted. (PB1)
     Pin 13  OUTPUT   Optogenetic stimulation trigger, active HIGH.                 (PB5)

   Both inputs are plain INPUT (no pull-ups), matching the previously working configuration.
   They must be driven push-pull by the source hardware.

   NOTE ON PIN 13 AT RESET: the Uno bootloader blinks pin 13 on every reset and every upload,
   before any sketch code runs. Firmware cannot suppress this. You have confirmed this flash
   is acceptable.

  --------------------------------------------------------------------------------------------
   THE VOLUME GRID  (defaults: 5 planes + 2 flybacks = 7 frames/volume, stim every 2nd volume)
  --------------------------------------------------------------------------------------------
   A "volume" is planesPerVolume imaging frames followed by flybackFrames flyback frames.
   The stimulation fires on the rising edge of the FIRST flyback frame of a stimulated volume,
   i.e. frame (planesPerVolume + 1) = frame 6 of 7. Because the pulse then runs for
   pulseDurationMs, the light covers: both flybacks of volume N, all planes of volume N+1,
   and both flybacks of volume N+1.

     Cumulative frame   Volume   Frame in volume        Opto
     ----------------   ------   ---------------        ----
          1  -  5         V1     planes 1-5             off
          6  -  7         V1     flybacks 6-7           off
          8  - 12         V2     planes 1-5             off
             13           V2     flyback 6              FIRES  <-- pulse starts here
             14           V2     flyback 7              on
         15  - 19         V3     planes 1-5             on
         20  - 21         V3     flybacks 6-7           on
         22  - 26         V4     planes 1-5             off      <-- clean control volume
             27           V4     flyback 6              FIRES
                                 ... and so on, every 14 frames

   Fully stimulated volumes are V3, V5, V7, ...   Clean control volumes are V4, V6, V8, ...
   Fire points land on cumulative frames 13, 27, 41, 55, ... (spacing = 14 frames).

   WINDOW ARITHMETIC: 9 frames (2 flybacks + 5 planes + 2 flybacks) at 30 Hz is exactly
   300.0 ms. The default pulseDurationMs is 295 ms, which leaves a 5 ms guard band so that a
   true frame rate slightly above 30.000 Hz cannot leak light into the control volume's first
   plane. Use the inferred frame rate in the status report to tune this after one session.

  --------------------------------------------------------------------------------------------
   TIMING ARCHITECTURE  —  why nothing in loop() can disturb the stimulus
  --------------------------------------------------------------------------------------------
   1. FRAME COUNTING AND PULSE START happen inside the pin-change interrupt (PCINT2, pin 6).
      Onset latency is ~4 us instead of "up to one loop pass", and counting is completely
      immune to anything the main loop is doing, including serial transmission.

   2. PULSE TERMINATION happens inside a Timer1 output-compare interrupt, not a millis() poll
      in loop(). Prescaler /1024 gives 64 us resolution and a 4.194 s ceiling, so a 295 ms
      pulse is accurate to better than 0.05 ms regardless of loop load.

   3. SERIAL OUTPUT IS CHUNKED. Text is queued into our own ring buffer and only
      Serial.availableForWrite() bytes are handed to the UART each loop pass, so no print ever
      spins waiting for buffer space. Long status blocks dribble out over a few milliseconds of
      wall time while the loop keeps running freely.

   4. 250000 BAUD (see top of file).

   The combination means the 10-second status report has no effect at all on stimulus timing.

   TIMER1 AND PIN 9 — IMPORTANT: Timer1's OC1A compare-output pin is physically Arduino pin 9,
   which is our enable input. We enable ONLY the compare-match INTERRUPT (OCIE1A) and leave the
   compare-output mode bits (COM1A1/COM1A0) cleared, so Timer1 never drives pin 9 and it stays
   an ordinary digital input. The register setup below looks alarming for this reason; it is
   safe. Timer1 is otherwise unused: millis() is Timer0, and neither Servo nor analogWrite() on
   pins 9/10 is used anywhere in this sketch.

  --------------------------------------------------------------------------------------------
   FRAME CLOCK CONDITIONING
  --------------------------------------------------------------------------------------------
   The frame clock pulse is ~1 ms wide. PCINT fires on both edges, so the ISR first checks the
   pin level and discards falling edges. A 10 ms REFRACTORY LOCKOUT then rejects contact bounce
   and ringing: after a counted edge, further edges are ignored for 10 ms. This is a lockout,
   not a stability filter -- a stability filter longer than the 1 ms pulse width would reject
   every real frame. At 33.3 ms per frame the 10 ms lockout has a 3.3x margin against ever
   swallowing a genuine frame.

  --------------------------------------------------------------------------------------------
   RAMP-DOWN
  --------------------------------------------------------------------------------------------
   FRAME-LOCKED MODE: the ramp multiplies the VOLUME DIVISOR. With the defaults (300 s total,
   5 steps of 60 s, multiplier 2) stimulation goes from every 2nd volume -> 4th -> 8th -> 16th
   -> 32nd, then stops and the system returns to ARMED-IDLE so the next pin 9 rising edge
   resumes at base rate. Pulse duration is unchanged throughout.

   Note that step 0 (the first 60 s) runs at the BASE divisor, exactly as in VR_Widefield_v4.
   The ramp therefore does not begin reducing the rate until 60 s in.

   Because spacing is counted in whole volumes, any multiplier -- including non-powers-of-2
   such as 3 or 5 -- keeps every fire point on frame 6 of a volume by construction. Divisor
   changes take effect at the next volume boundary.

   FREE-RUN MODE ('r'): the ramp multiplies the PERIOD instead (e.g. 2 Hz -> 1 -> 0.5 -> 0.25
   -> 0.125), ON duration unchanged, then stops fully. Because ON duration is held while the
   period grows, a 100 %-duty free-run becomes pulsed as the ramp proceeds.

   Triggers for ramp-down: pin 9 falling edge, the 'g 0' gate override, or the 'q' command.
   If pin 9 goes LOW mid-pulse the current pulse is allowed to finish first.

   Cancelling a ramp: a pin 9 RISING edge, 'g 1', or 'g a' while pin 9 is HIGH. Cancellation is
   edge-triggered rather than level-triggered so that 'g 0' does not immediately self-cancel
   while pin 9 is still physically HIGH -- pin 9 must go LOW and then HIGH again. If a pin 9
   rising edge cancels a ramp that 'g 0' started, the override is cleared back to AUTO and a
   notice is printed (otherwise the state would read "forced off" while stimulating).

  --------------------------------------------------------------------------------------------
   SERIAL COMMANDS  (commas and spaces are interchangeable: "r 2 250" == "r,2,250")
  --------------------------------------------------------------------------------------------
     s              Start run. Resets all counters. Frame counting begins.
     x              Stop everything immediately + print summary.
     q              Ramp down now (works in both frame-locked and free-run mode).
     ?              Show command list, all settings, and the fire schedule.
     v              Print the status block immediately.

     d <ms>         Opto pulse duration, 1-4000 ms.  ** the only live-editable parameter **
     n <planes>     Planes per volume.
     y <frames>     Flyback frames per volume.
     e <n>          Stimulate every Nth volume.
     a <frames>     Phase offset in frames (0 = first counted edge is frame 1 of volume 1).
     t <ms>         Ramp total duration.
     k <steps>      Ramp step count.
     m <factor>     Ramp period/divisor multiplier.

     o              Fire ONE pulse of duration d (manual test).
     o<sec>         Manual timed latch: hold the laser ON for <sec> seconds, then off.
     r <hz>,<ms>    Free-run stimulation at <hz> with <ms> ON, until 'x' or 'q'.
     g a | 1 | 0    Gate override: a = follow pin 9, 1 = force enabled, 0 = force disabled.

   PARAMETER LOCKING: n, y, e, a, t, k and m all change the experiment structure, so they are
   REJECTED while a run is in progress. Stop with 'x' first. Only 'd' may be changed live, and
   it takes effect from the next pulse (a pulse already in flight keeps its original duration).

   The gate override always resets to AUTO on reset and on upload.

  --------------------------------------------------------------------------------------------
   10-SECOND STATUS REPORT AND ANOMALY DETECTION
  --------------------------------------------------------------------------------------------
   Reported every 10 s while running, and on demand with 'v':
     - position:      cumulative frame, volume, and frame-within-volume
     - inferred rates: frame rate, volume rate, and stimulation rate, all measured
     - opto state, pulses delivered, ramp step
     - anomaly counters

   PHASE SLIP is the failure mode that matters most here, and it is invisible to frame counts:
   if the board MISSES a frame clock edge the counter simply does not increment, so consecutive
   deliveries are still exactly `divisor` counted frames apart while the stimulus has silently
   slipped one frame relative to the microscope. Phase slip is only detectable in the TIME
   domain -- a missed edge shows up as an inter-edge interval near 2x nominal, a spurious edge
   as a short one. Hence the per-edge interval measurement.

   The nominal interval is the median of the first 32 measured intervals, computed once and
   then held fixed. A fixed nominal is both cheaper than a sliding median and more robust,
   since it cannot be dragged by the anomalies it is meant to catch. Tolerance is +/-25 %.

   Detection is report-only. The frame counter is never auto-corrected, because a mis-diagnosis
   (for instance during a genuine acquisition pause) would corrupt alignment rather than fix it.

   Fire points skipped because pin 9 was LOW are normal baseline operation and are NOT reported.

  --------------------------------------------------------------------------------------------
   RESOURCE NOTES  (Arduino Uno: 32 KB flash, 2 KB SRAM)
  --------------------------------------------------------------------------------------------
   SRAM: ~760 bytes of static allocation -- 320 for the serial output ring, 128 for the core's
   own UART buffers, 64 for the calibration window, 40 for the command buffer, and the rest in
   counters. Roughly 1.2 KB is left for the stack. All fixed strings live in flash via PSTR(),
   and String is not used anywhere, so there is no heap fragmentation over a long session.

   ISR budget: the frame clock ISR is ~15 us against a 33 333 us frame period, i.e. under
   0.05 % of the CPU. No 32-bit division or float arithmetic occurs in either ISR.

  ============================================================================================
*/

#include <Arduino.h>


/* ============================================================================================
   ============================  USER-CHANGEABLE PARAMETERS  ==================================
   ============================================================================================
   These are the values you will actually want to edit. Everything below this block is
   machinery. All of them are also settable at runtime over serial -- see the command list.
*/

// ---- Volume structure -------------------------------------------------------------------
unsigned long planesPerVolume     = 5UL;        // 'n'  imaging frames per volume
unsigned long flybackFrames       = 2UL;        // 'y'  flyback frames per volume
                                                //      => 7 frames per volume by default
unsigned long stimEveryNthVolume  = 2UL;        // 'e'  stimulate every Nth volume
                                                //      => fire every 14 frames by default
unsigned long phaseOffsetFrames   = 0UL;        // 'a'  frame-grid offset; 0 = first counted
                                                //      edge is frame 1 of volume 1

// ---- Stimulation pulse ------------------------------------------------------------------
unsigned long pulseDurationMs     = 295UL;      // 'd'  opto pulse length, 1..4000 ms.
                                                //      295 ms covers 9 frames at 30 Hz
                                                //      (300.0 ms) with a 5 ms guard band.

// ---- Ramp-down --------------------------------------------------------------------------
unsigned long rampDurationMs      = 300000UL;   // 't'  total ramp length (300 s)
unsigned long rampSteps           = 5UL;        // 'k'  number of equal steps
unsigned long rampMultiplier      = 2UL;        // 'm'  divisor (or period) multiplier per step

// ---- Frame clock conditioning -----------------------------------------------------------
unsigned long refractoryUs        = 10000UL;    // 10 ms lockout after each counted edge.
                                                // Must be < one frame period and > any bounce.
const unsigned long GATE_DEBOUNCE_MS      = 5UL;      // pin 9 level stability filter
const unsigned long STATUS_INTERVAL_MS    = 10000UL;  // status report cadence
const unsigned long INTERVAL_TOL_PERCENT  = 25UL;     // anomaly window around nominal
const unsigned long SERIAL_BAUD           = 250000UL; // ** set the Serial Monitor to match **

/* ==========================  END USER-CHANGEABLE PARAMETERS  ===============================
   ============================================================================================
*/


// ============================================================================================
//   PIN DEFINITIONS  (register-level names are used inside the ISRs for speed)
// ============================================================================================
const uint8_t PIN_FRAMECLK = 6;    // PD6 / PCINT22 — frame clock in
const uint8_t PIN_ENABLE   = 9;    // PB1           — enable / VR gate in
const uint8_t PIN_OPTO     = 13;   // PB5           — opto trigger out

#define FRAMECLK_IS_HIGH()  (PIND & _BV(PD6))
#define OPTO_HIGH()         (PORTB |=  _BV(PB5))
#define OPTO_LOW()          (PORTB &= ~_BV(PB5))

// Timer1 at /1024 on a 16 MHz clock ticks at 15625 Hz => 64 us per tick.
// ticks = ms * 15625 / 1000 = ms * 125 / 8.   4000 ms -> 62500 ticks, inside the 65535 ceiling.
const unsigned long PULSE_MS_MAX = 4000UL;
#define MS_TO_TICKS(ms)  ((uint16_t)(((unsigned long)(ms) * 125UL) / 8UL))


// ============================================================================================
//   OPTO STATE MACHINE
// ============================================================================================
enum OptoState : uint8_t {
  OPTO_OFF,            // no run in progress
  OPTO_ARMED,          // run active, gate LOW — grid advances, fire points suppressed
  OPTO_ACTIVE,         // run active, gate HIGH — delivering on the volume grid
  OPTO_RAMP,           // run active, ramping down (gate is bypassed during the ramp)
  OPTO_FREERUN,        // 'r' free-run, independent of gate and frame clock
  OPTO_FREERUN_RAMP    // 'r' free-run, ramping its period down
};
OptoState optoState = OPTO_OFF;

enum GateOverride : uint8_t { GATE_AUTO, GATE_FORCE_ON, GATE_FORCE_OFF };
GateOverride gateOverride = GATE_AUTO;   // always AUTO after reset/upload


// ============================================================================================
//   STATE SHARED WITH THE ISRs
//   Everything an ISR touches is volatile. Multi-byte values are read from loop() through
//   atomic helpers, because a 32-bit read is not atomic on an 8-bit core.
// ============================================================================================

// --- grid position ---
volatile unsigned long frameCount     = 0UL;   // cumulative counted frame clock edges
volatile unsigned long volumeCount    = 1UL;   // cumulative volume index, 1-based
volatile unsigned long frameInVolume  = 0UL;   // 1..framesPerVolume
volatile unsigned long volumeInCycle  = 0UL;   // 0..stimEveryNthVolume-1; 0 == stim volume

// --- derived grid constants, recomputed by recomputeGrid() ---
volatile unsigned long framesPerVolume = 7UL;  // planes + flybacks
volatile unsigned long firePointFrame  = 6UL;  // first flyback = planesPerVolume + 1
volatile unsigned long activeDivisor   = 2UL;  // current volume divisor (base, or ramped)

// --- pulse hardware state ---
volatile bool     pulseActive     = false;     // a Timer1-terminated pulse is in flight
volatile uint16_t pulseTicks      = 0;         // pre-computed tick count for the next pulse
volatile bool     stimArmed       = false;     // loop() sets this; the ISR's only gate check
volatile bool     manualHold      = false;     // 'o<sec>' latch is holding the pin HIGH

// --- delivery bookkeeping ---
volatile unsigned long deliveries         = 0UL;  // scheduled pulses delivered
volatile unsigned long lastDeliveryFrame  = 0UL;
volatile unsigned long overlapRestarts    = 0UL;  // fire point arrived while a pulse was on
volatile bool          deliveryFlag       = false;// set by ISR, consumed by loop()
volatile unsigned long lastSpacing        = 0UL;  // frames between the last two deliveries

// --- interval statistics for phase-slip detection ---
volatile unsigned long lastEdgeUs       = 0UL;
volatile bool          haveFirstEdge    = false;
volatile unsigned long lastIntervalUs   = 0UL;
volatile unsigned long minIntervalUs    = 0xFFFFFFFFUL;
volatile unsigned long maxIntervalUs    = 0UL;
volatile unsigned long shortIntervals   = 0UL;  // suspected spurious edges
volatile unsigned long longIntervals    = 0UL;  // suspected missed edges
volatile unsigned long refractoryDrops  = 0UL;  // edges rejected by the 10 ms lockout

// Anomalous intervals are pushed here by the ISR and interpreted by loop(). This is what turns
// "an interval looked wrong" into "the phase has slipped by N frames": the loop divides the
// bad interval by the nominal to recover how many edges went missing. The division is far too
// slow for an ISR (~40 us for a 32-bit divide), hence the hand-off. Anomalies are rare, so
// four slots is ample; if it ever overflows the event is still counted, just not quantified.
const uint8_t          ANOM_SLOTS = 4;
volatile unsigned long anomRing[ANOM_SLOTS];
volatile uint8_t       anomHead = 0, anomTail = 0;

// --- nominal-interval calibration (median of the first 32 intervals) ---
const uint8_t CALIB_N = 32;
volatile uint16_t      calibBuf[CALIB_N];       // interval >> 4, i.e. units of 16 us
volatile uint8_t       calibCount   = 0;
volatile bool          nominalLocked = false;
volatile unsigned long nominalUs    = 0UL;
volatile unsigned long nominalLoUs  = 0UL;
volatile unsigned long nominalHiUs  = 0UL;


// ============================================================================================
//   LOOP-ONLY STATE
// ============================================================================================
bool          running          = false;
bool          summaryPrinted   = true;
unsigned long runStartMs       = 0UL;
unsigned long firstEdgeMs      = 0UL;      // when the first frame edge was noticed
bool          haveFirstEdgeMs  = false;
unsigned long lastStatusMs     = 0UL;

// gate debounce
int           gateRawLast      = LOW;
int           gatePinStable    = LOW;
unsigned long gateChangedMs    = 0UL;

// ramp
unsigned long rampStartMs      = 0UL;
unsigned long rampStepNow      = 0UL;
bool          pendingRamp      = false;    // pin 9 went LOW mid-pulse; ramp after it ends
unsigned long baseDivisor      = 2UL;      // divisor to restore when a ramp is cancelled

// free-run
unsigned long freeRunPeriodMs  = 500UL;
unsigned long freeRunBasePerMs = 500UL;
unsigned long freeRunOnMs      = 250UL;
unsigned long freeRunNextMs    = 0UL;
bool          freeRunPinOn     = false;
unsigned long freeRunPulses    = 0UL;

// manual test
unsigned long manualUntilMs    = 0UL;
unsigned long manualPulses     = 0UL;

// anomaly bookkeeping done in loop()
unsigned long spacingMismatches = 0UL;
bool          gridDisturbed     = true;    // suppresses the spacing check across grid changes
unsigned long stallEvents       = 0UL;
bool          inStall           = false;
unsigned long lastFrameSeen     = 0UL;
unsigned long lastFrameSeenMs   = 0UL;

// Quantified phase slip, accumulated from the anomaly ring. framesMissed is the headline
// number: it is how many frame clock edges the board believes it never saw, and therefore
// exactly how far the volume grid has slipped behind the microscope.
unsigned long framesMissed      = 0UL;
unsigned long framesSpurious    = 0UL;

// Delivery timestamps, used so the measured stimulation rate is computed between the first and
// last delivery rather than from the start of the run. Otherwise a long gate-off baseline would
// drag the reported rate far below the true value.
unsigned long firstDeliveryMs   = 0UL;
unsigned long lastDeliveryMs    = 0UL;
unsigned long deliveriesSeen    = 0UL;


// ============================================================================================
//   ATOMIC ACCESS HELPERS
//   A 32-bit load on an 8-bit core takes four instructions; an interrupt landing in the middle
//   would return a torn value. These helpers cost ~1 us and remove the whole class of bug.
// ============================================================================================
static inline unsigned long atomicGet32(volatile unsigned long *p) {
  unsigned long v;
  uint8_t s = SREG;
  cli();
  v = *p;
  SREG = s;
  return v;
}

static inline void atomicSet32(volatile unsigned long *p, unsigned long v) {
  uint8_t s = SREG;
  cli();
  *p = v;
  SREG = s;
}


// ============================================================================================
//   NON-BLOCKING SERIAL OUTPUT
//   Text is queued here and drained a few bytes at a time in loop(), only ever as many bytes
//   as Serial.availableForWrite() reports free. Serial.write() therefore never spins, so no
//   print can ever delay the loop -- which is what makes a verbose status report safe.
// ============================================================================================
const uint16_t OUTBUF_SIZE = 320;
char           outBuf[OUTBUF_SIZE];
uint16_t       outHead = 0, outTail = 0;
unsigned long  outDropped = 0UL;

static inline uint16_t outUsed() {
  return (uint16_t)((outHead + OUTBUF_SIZE - outTail) % OUTBUF_SIZE);
}
static inline uint16_t outFree() {
  return (uint16_t)(OUTBUF_SIZE - 1 - outUsed());
}

// Queue a RAM string.
void outStr(const char *s) {
  while (*s) {
    if (outFree() == 0) { outDropped++; return; }
    outBuf[outHead] = *s++;
    outHead = (uint16_t)((outHead + 1) % OUTBUF_SIZE);
  }
}

// Queue a PROGMEM string (use with the P() macro below to keep flash strings out of SRAM).
void outStrP(const char *s) {
  char c;
  while ((c = (char)pgm_read_byte(s++)) != '\0') {
    if (outFree() == 0) { outDropped++; return; }
    outBuf[outHead] = c;
    outHead = (uint16_t)((outHead + 1) % OUTBUF_SIZE);
  }
}
#define P(str) outStrP(PSTR(str))

void outNL() { outStr("\r\n"); }

// Queue an unsigned long as decimal.
void outUL(unsigned long v) {
  char b[12];
  snprintf(b, sizeof(b), "%lu", v);
  outStr(b);
}

// Queue a value held in hundredths as "X.YY" -- avoids %f, which avr-libc's snprintf does not
// support by default, and avoids dragging in the floating-point formatter at all.
void outCenti(unsigned long centi) {
  char b[16];
  snprintf(b, sizeof(b), "%lu.%02lu", centi / 100UL, centi % 100UL);
  outStr(b);
}

// Drain a bounded number of bytes to the UART. Called every loop pass.
void serviceSerialOut() {
  int room = Serial.availableForWrite();
  while (room > 0 && outTail != outHead) {
    Serial.write(outBuf[outTail]);
    outTail = (uint16_t)((outTail + 1) % OUTBUF_SIZE);
    room--;
  }
}


// ============================================================================================
//   MULTI-LINE BLOCK PRINTER
//   Help, settings, status and summary are all long. Rather than dumping them into the ring in
//   one go (which would overflow it), a block emits ONE LINE PER LOOP PASS, and only when the
//   ring has room. The loop runs thousands of times a second, so a 30-line block drains in a
//   few milliseconds of wall time without ever blocking.
// ============================================================================================
enum BlockKind : uint8_t { BLK_NONE, BLK_HELP, BLK_SETTINGS, BLK_STATUS, BLK_SUMMARY };
BlockKind blockKind = BLK_NONE;
uint8_t   blockLine = 0;
BlockKind blockThen = BLK_NONE;   // optional block to run immediately after this one

void requestBlock(BlockKind k, BlockKind then_ = BLK_NONE) {
  blockKind = k;
  blockLine = 0;
  blockThen = then_;
}


// ============================================================================================
//   SMALL MATH HELPERS
// ============================================================================================

// base * multiplier^exponent with a hard ceiling, so a long ramp cannot wrap around.
unsigned long safePow(unsigned long base, unsigned long mult, unsigned long expo) {
  unsigned long r = base;
  for (unsigned long i = 0; i < expo; i++) {
    if (mult == 0) return r;
    if (r > 0xFFFFFFFFUL / mult) return 0xFFFFFFFFUL;
    r *= mult;
  }
  return r;
}

// Rate in hundredths of Hz from a count and an elapsed time in ms.
// Two scalings so that neither the multiply nor the divide overflows or loses resolution:
// the fine path is exact to 0.1 % and valid to ~4 hours at 30 Hz; the coarse path takes over
// after that.
unsigned long centiRate(unsigned long count, unsigned long elapsedMs) {
  if (elapsedMs == 0UL) return 0UL;
  if (count <= 400000UL) {
    unsigned long den = elapsedMs / 10UL;
    if (den == 0UL) den = 1UL;
    return (count * 10000UL) / den;
  }
  unsigned long den = elapsedMs / 1000UL;
  if (den == 0UL) den = 1UL;
  return (count * 100UL) / den;
}

// Microseconds as hundredths of a millisecond, for "33.33 ms" style output.
static inline unsigned long usToCentiMs(unsigned long us) { return us / 10UL; }

void formatHMS(unsigned long ms, char *out, size_t n) {
  unsigned long s = (ms / 1000UL) % 60UL;
  unsigned long m = (ms / 60000UL) % 60UL;
  unsigned long h = ms / 3600000UL;
  snprintf(out, n, "%02lu:%02lu:%02lu", h, m, s);
}


// ============================================================================================
//   PULSE HARDWARE  (Timer1 compare, /1024 prescaler)
// ============================================================================================

// Start a pulse and arm Timer1 to end it. Safe to call from inside an ISR.
static inline void pulseStart(uint16_t ticks) {
  if (ticks < 1) ticks = 1;
  OPTO_HIGH();
  pulseActive = true;
  TCNT1  = 0;
  OCR1A  = ticks;
  TIFR1  = _BV(OCF1A);                             // clear any stale compare flag
  TIMSK1 = _BV(OCIE1A);                            // enable compare-match interrupt only
  TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10);     // CTC mode, prescaler /1024, start
  // COM1A1/COM1A0 in TCCR1A are left at 0, so OC1A (Arduino pin 9) is NOT driven.
}

// Force the output off and disarm the timer. Used by 'x', by mode changes, and at boot.
void pulseAbort() {
  uint8_t s = SREG;
  cli();
  TCCR1B = 0;                  // stop Timer1
  TIMSK1 = 0;                  // disarm the compare interrupt
  OPTO_LOW();
  pulseActive = false;
  manualHold  = false;
  SREG = s;
}

// Hold the output on indefinitely (manual latch, or 100 %-duty free-run). No timer involved,
// so durations longer than Timer1's 4.194 s ceiling are fine; loop() ends these.
void pulseHoldOn() {
  uint8_t s = SREG;
  cli();
  TCCR1B = 0;
  TIMSK1 = 0;
  OPTO_HIGH();
  pulseActive = false;
  manualHold  = true;
  SREG = s;
}


// ============================================================================================
//   GRID RECOMPUTATION
//   Called whenever the volume structure changes, and at every 's'. Never called while a run
//   is in progress, because the structural parameters are locked during a run.
// ============================================================================================
void recomputeGrid() {
  if (planesPerVolume < 1UL)    planesPerVolume = 1UL;
  if (flybackFrames   < 1UL)    flybackFrames   = 1UL;   // >=1 so a fire point exists
  if (stimEveryNthVolume < 1UL) stimEveryNthVolume = 1UL;

  unsigned long fpv = planesPerVolume + flybackFrames;
  atomicSet32(&framesPerVolume, fpv);
  atomicSet32(&firePointFrame,  planesPerVolume + 1UL);   // first flyback frame
  atomicSet32(&activeDivisor,   stimEveryNthVolume);
  baseDivisor = stimEveryNthVolume;
}

// Seed the grid from the phase offset so that the NEXT counted edge is frame (offset + 1).
void seedGridFromOffset() {
  unsigned long fpv = framesPerVolume;
  unsigned long off = phaseOffsetFrames;
  uint8_t s = SREG;
  cli();
  frameCount    = off;
  frameInVolume = off % fpv;
  volumeCount   = (off / fpv) + 1UL;
  volumeInCycle = volumeCount % activeDivisor;
  SREG = s;
}

// Change the active divisor (ramp steps do this) while preserving ABSOLUTE volume phase.
// The modulo is done here in loop context, once per divisor change, so the ISR only ever has
// to increment and compare.
void setActiveDivisor(unsigned long d) {
  if (d < 1UL) d = 1UL;
  uint8_t s = SREG;
  cli();
  activeDivisor = d;
  volumeInCycle = volumeCount % d;
  SREG = s;
  gridDisturbed = true;      // spacing check is meaningless across a divisor change
}


// ============================================================================================
//   INTERRUPT SERVICE ROUTINES
// ============================================================================================

// -------- Frame clock: count the edge, advance the grid, start the pulse -------------------
// Budget is roughly 15 us against a 33333 us frame period, i.e. under 0.05 % of the CPU.
ISR(PCINT2_vect)
{
  // PCINT fires on both edges. The frame clock pulse is only ~1 ms wide, so discard the
  // falling edge by reading the pin level.
  if (!FRAMECLK_IS_HIGH()) return;

  // micros() called from inside an ISR can, rarely, be off by 1024 us if a Timer0 overflow
  // happens to land while interrupts are masked here. That is ~3 % of one frame period and
  // well inside the +/-25 % anomaly window, so it cannot produce a false slip report. It is
  // only ever used for interval statistics, never for stimulus timing.
  unsigned long nowUs = micros();

  if (haveFirstEdge) {
    unsigned long dtUs = nowUs - lastEdgeUs;

    // ---- refractory lockout: reject bounce and ringing ----
    if (dtUs < refractoryUs) { refractoryDrops++; return; }

    // ---- interval statistics: the only way to see phase slip ----
    lastIntervalUs = dtUs;
    if (dtUs < minIntervalUs) minIntervalUs = dtUs;
    if (dtUs > maxIntervalUs) maxIntervalUs = dtUs;

    if (nominalLocked) {
      if (dtUs < nominalLoUs || dtUs > nominalHiUs) {
        if (dtUs < nominalLoUs) shortIntervals++;      // suspected spurious edge
        else                    longIntervals++;       // suspected missed edge
        uint8_t nh = (uint8_t)((anomHead + 1) % ANOM_SLOTS);
        if (nh != anomTail) { anomRing[anomHead] = dtUs; anomHead = nh; }
      }
    } else if (calibCount < CALIB_N) {
      calibBuf[calibCount++] = (uint16_t)(dtUs >> 4);  // 16 us units keeps this in 64 bytes
    }
  } else {
    haveFirstEdge = true;
  }
  lastEdgeUs = nowUs;

  // ---- advance the frame / volume grid ----
  frameCount++;
  frameInVolume++;
  if (frameInVolume > framesPerVolume) {
    frameInVolume = 1UL;
    volumeCount++;
    volumeInCycle++;
    if (volumeInCycle >= activeDivisor) volumeInCycle = 0UL;
  }

  // ---- fire point? ----
  // volumeInCycle == 0 marks a stimulated volume; firePointFrame is its first flyback.
  if (frameInVolume == firePointFrame && volumeInCycle == 0UL && stimArmed) {
    if (pulseActive || manualHold) overlapRestarts++;   // restart-on-overlap, and log it
    lastSpacing        = frameCount - lastDeliveryFrame;
    lastDeliveryFrame  = frameCount;
    deliveries++;
    deliveryFlag       = true;
    pulseStart(pulseTicks);
  }
}

// -------- Timer1 compare: end the pulse ----------------------------------------------------
// This is what decouples pulse LENGTH from the frame clock. Sub-0.05 ms accurate regardless
// of what loop() happens to be doing.
ISR(TIMER1_COMPA_vect)
{
  OPTO_LOW();
  pulseActive = false;
  TCCR1B = 0;      // stop the timer
  TIMSK1 = 0;      // disarm
}


// ============================================================================================
//   FRAME CLOCK INTERRUPT ENABLE / DISABLE
//   The pin-change interrupt is only enabled between 's' and 'x', so frame counting genuinely
//   stops outside a run and there is zero ISR cost while idle.
// ============================================================================================
void frameClkInterruptEnable(bool on) {
  uint8_t s = SREG;
  cli();
  if (on) {
    PCIFR  |= _BV(PCIF2);        // clear any pending flag
    PCMSK2 |= _BV(PCINT22);      // unmask PD6
    PCICR  |= _BV(PCIE2);        // enable PORTD pin-change group
  } else {
    PCMSK2 &= (uint8_t)~_BV(PCINT22);
    if (PCMSK2 == 0) PCICR &= (uint8_t)~_BV(PCIE2);
  }
  SREG = s;
}


// ============================================================================================
//   NOMINAL INTERVAL CALIBRATION
//   Median of the first 32 intervals, computed once in loop context, then frozen.
// ============================================================================================
void tryLockNominal() {
  if (nominalLocked) return;
  if (calibCount < CALIB_N) return;

  uint16_t tmp[CALIB_N];
  uint8_t s = SREG;
  cli();
  for (uint8_t i = 0; i < CALIB_N; i++) tmp[i] = calibBuf[i];
  SREG = s;

  // insertion sort — 32 elements, one time only
  for (uint8_t i = 1; i < CALIB_N; i++) {
    uint16_t k = tmp[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = k;
  }

  unsigned long med = (unsigned long)tmp[CALIB_N / 2] << 4;   // back to microseconds
  unsigned long lo  = med - (med * INTERVAL_TOL_PERCENT) / 100UL;
  unsigned long hi  = med + (med * INTERVAL_TOL_PERCENT) / 100UL;

  s = SREG;
  cli();
  nominalUs     = med;
  nominalLoUs   = lo;
  nominalHiUs   = hi;
  nominalLocked = true;
  SREG = s;

  P("[CAL] Nominal frame interval locked: "); outCenti(usToCentiMs(med));
  P(" ms  (tolerance "); outCenti(usToCentiMs(lo)); P(" - ");
  outCenti(usToCentiMs(hi)); P(" ms)"); outNL();
}


// ============================================================================================
//   RAMP-DOWN
// ============================================================================================
unsigned long rampStepDurMs() {
  unsigned long k = (rampSteps < 1UL) ? 1UL : rampSteps;
  unsigned long d = rampDurationMs / k;
  return (d < 1UL) ? 1UL : d;
}

void beginRampFrameLocked(unsigned long nowMs) {
  if (rampSteps < 1UL) {                 // degenerate config: stop straight away
    P("[RAMP] 0 steps configured -> stimulation OFF, ARMED-IDLE."); outNL();
    optoState = OPTO_ARMED;
    stimArmed = false;
    return;
  }
  optoState    = OPTO_RAMP;
  rampStartMs  = nowMs;
  rampStepNow  = 0UL;
  stimArmed    = true;                   // the gate is bypassed for the duration of the ramp
  setActiveDivisor(baseDivisor);         // step 0 runs at the base divisor
  P("[RAMP] Started. Step 1/"); outUL(rampSteps);
  P("  every "); outUL(baseDivisor); P(" volumes  (step = ");
  outUL(rampStepDurMs()); P(" ms)"); outNL();
}

void beginRampFreeRun(unsigned long nowMs) {
  if (rampSteps < 1UL) {
    P("[RAMP] 0 steps configured -> free-run stopped."); outNL();
    pulseAbort();
    optoState = OPTO_OFF;
    return;
  }
  optoState        = OPTO_FREERUN_RAMP;
  rampStartMs      = nowMs;
  rampStepNow      = 0UL;
  freeRunPeriodMs  = freeRunBasePerMs;
  P("[RAMP] Free-run ramp started. Step 1/"); outUL(rampSteps);
  P("  period "); outUL(freeRunPeriodMs); P(" ms"); outNL();
}

void endRampToArmedIdle() {
  P("[RAMP] Complete. Stimulation OFF -> ARMED-IDLE "
    "(next pin 9 rising edge resumes at base rate)."); outNL();
  setActiveDivisor(baseDivisor);
  optoState = OPTO_ARMED;
  stimArmed = false;
}

void cancelRamp(unsigned long nowMs, const char *reasonP) {
  (void)nowMs;
  setActiveDivisor(baseDivisor);
  optoState = OPTO_ACTIVE;
  stimArmed = true;
  pendingRamp = false;
  P("[RAMP] Cancelled ("); outStrP(reasonP);
  P("). Resuming every "); outUL(baseDivisor); P(" volumes."); outNL();
}

// Re-enabling the gate must cancel a ramp in EITHER of its two forms:
//   - OPTO_RAMP        the ramp is already running, or
//   - pendingRamp      the ramp is queued behind an in-flight pulse and has not started yet.
// Missing the second case was a real bug: a pin 9 rising edge inside the pulse window left
// pendingRamp set, so the ramp began anyway a few hundred milliseconds later even though the
// gate had gone back HIGH. Returns true if anything was cancelled.
bool cancelRampAnyForm(unsigned long nowMs, const char *reasonP) {
  if (optoState == OPTO_RAMP) { cancelRamp(nowMs, reasonP); return true; }
  if (pendingRamp) {
    pendingRamp = false;
    optoState   = OPTO_ACTIVE;
    stimArmed   = true;
    P("[RAMP] Pending ramp cancelled before it started ("); outStrP(reasonP);
    P("). Staying at every "); outUL(baseDivisor); P(" volumes."); outNL();
    return true;
  }
  return false;
}

void serviceRamp(unsigned long nowMs) {
  if (optoState != OPTO_RAMP && optoState != OPTO_FREERUN_RAMP) return;

  unsigned long el = nowMs - rampStartMs;

  if (el >= rampDurationMs) {
    if (optoState == OPTO_RAMP) {
      endRampToArmedIdle();
    } else {
      P("[RAMP] Free-run ramp complete. Stopped."); outNL();
      pulseAbort();
      freeRunPinOn = false;
      optoState    = OPTO_OFF;
    }
    return;
  }

  unsigned long stepDur = rampStepDurMs();
  unsigned long step    = el / stepDur;
  if (step >= rampSteps) step = rampSteps - 1UL;

  if (step > rampStepNow) {
    rampStepNow = step;
    if (optoState == OPTO_RAMP) {
      unsigned long d = safePow(baseDivisor, rampMultiplier, rampStepNow);
      if (d > 65535UL) d = 65535UL;
      setActiveDivisor(d);
      P("[RAMP] Step "); outUL(rampStepNow + 1UL); P("/"); outUL(rampSteps);
      P("  -> every "); outUL(d); P(" volumes"); outNL();
    } else {
      unsigned long p = safePow(freeRunBasePerMs, rampMultiplier, rampStepNow);
      if (p > 3600000UL) p = 3600000UL;
      freeRunPeriodMs = p;
      P("[RAMP] Step "); outUL(rampStepNow + 1UL); P("/"); outUL(rampSteps);
      P("  -> period "); outUL(p); P(" ms"); outNL();
    }
  }
}


// ============================================================================================
//   GATE (PIN 9) HANDLING
// ============================================================================================
bool gateEffectiveLevel() {
  if (gateOverride == GATE_FORCE_ON)  return true;
  if (gateOverride == GATE_FORCE_OFF) return false;
  return (gatePinStable == HIGH);
}

// A falling effective gate starts a ramp. If a pulse is in flight, wait for it to finish.
void requestRampDown(unsigned long nowMs, const char *reasonP) {
  if (optoState != OPTO_ACTIVE) return;
  bool inFlight;
  uint8_t s = SREG; cli(); inFlight = pulseActive || manualHold; SREG = s;

  if (inFlight) {
    pendingRamp = true;
    P("[GATE] "); outStrP(reasonP);
    P(" mid-pulse -> ramp will start when this pulse ends."); outNL();
  } else {
    P("[GATE] "); outStrP(reasonP); P(" -> ramping down."); outNL();
    beginRampFrameLocked(nowMs);
  }
}

// Pin 9 rising edge. Cancellation is edge-triggered so that 'g 0' does not self-cancel while
// pin 9 is still physically HIGH.
void onGateRisingEdge(unsigned long nowMs) {
  if (!running) return;

  if (gateOverride == GATE_FORCE_OFF) {
    // The override outranks the pin, but a rising edge is an explicit operator action, so
    // clear the override -- otherwise the state would read "forced off" while stimulating.
    gateOverride = GATE_AUTO;
    P("[GATE] Pin 9 rising edge while override was FORCED OFF -> override cleared to AUTO.");
    outNL();
  }

  if (cancelRampAnyForm(nowMs, PSTR("pin 9 rising edge"))) return;
  if (optoState == OPTO_ARMED) {
    optoState = OPTO_ACTIVE;
    stimArmed = true;
    pendingRamp = false;
    P("[GATE] Enabled. Stimulation ACTIVE from the next scheduled fire point."); outNL();
  }
}

void onGateFallingEdge(unsigned long nowMs) {
  if (!running) return;
  if (gateOverride == GATE_FORCE_ON) return;    // override outranks the pin
  requestRampDown(nowMs, PSTR("Pin 9 LOW"));
}

void serviceGate(unsigned long nowMs) {
  int raw = digitalRead(PIN_ENABLE);
  if (raw != gateRawLast) { gateRawLast = raw; gateChangedMs = nowMs; }

  if ((nowMs - gateChangedMs) >= GATE_DEBOUNCE_MS && gatePinStable != raw) {
    gatePinStable = raw;
    if (raw == HIGH) onGateRisingEdge(nowMs);
    else             onGateFallingEdge(nowMs);
  }
}


// ============================================================================================
//   RUN CONTROL
// ============================================================================================
void resetRunCounters() {
  uint8_t s = SREG;
  cli();
  frameCount = 0UL; volumeCount = 1UL; frameInVolume = 0UL; volumeInCycle = 0UL;
  deliveries = 0UL; lastDeliveryFrame = 0UL; overlapRestarts = 0UL; lastSpacing = 0UL;
  deliveryFlag = false;
  haveFirstEdge = false; lastEdgeUs = 0UL; lastIntervalUs = 0UL;
  minIntervalUs = 0xFFFFFFFFUL; maxIntervalUs = 0UL;
  shortIntervals = 0UL; longIntervals = 0UL; refractoryDrops = 0UL;
  calibCount = 0; nominalLocked = false; nominalUs = 0UL; nominalLoUs = 0UL; nominalHiUs = 0UL;
  SREG = s;

  anomHead = 0; anomTail = 0;

  spacingMismatches = 0UL;
  gridDisturbed     = true;
  stallEvents       = 0UL;
  inStall           = false;
  manualPulses      = 0UL;
  freeRunPulses     = 0UL;
  framesMissed      = 0UL;
  framesSpurious    = 0UL;
  firstDeliveryMs   = 0UL;
  lastDeliveryMs    = 0UL;
  deliveriesSeen    = 0UL;
  haveFirstEdgeMs   = false;
  firstEdgeMs       = 0UL;
  lastFrameSeen     = 0UL;
  lastFrameSeenMs   = 0UL;
  outDropped        = 0UL;
}

void cmdStart(unsigned long nowMs) {
  if (optoState == OPTO_FREERUN || optoState == OPTO_FREERUN_RAMP) {
    P("!! Free-run is active. Send 'x' or 'q' first."); outNL();
    return;
  }
  if (running) { P(">> Already running."); outNL(); return; }

  // Print the previous run's summary if 'x' did not already do it.
  if (!summaryPrinted) requestBlock(BLK_SUMMARY);

  recomputeGrid();
  pulseTicks = MS_TO_TICKS(pulseDurationMs);
  resetRunCounters();
  seedGridFromOffset();

  running        = true;
  summaryPrinted = false;
  runStartMs     = nowMs;
  lastStatusMs   = nowMs;
  pendingRamp    = false;

  // Seed the gate debounce from the current pin level without generating a synthetic edge.
  gateRawLast   = digitalRead(PIN_ENABLE);
  gatePinStable = gateRawLast;
  gateChangedMs = nowMs;

  pulseAbort();
  frameClkInterruptEnable(true);

  if (gateEffectiveLevel()) {
    optoState = OPTO_ACTIVE;
    stimArmed = true;
  } else {
    optoState = OPTO_ARMED;
    stimArmed = false;
  }

  P(">> RUN STARTED."); outNL();
  P("   Grid: "); outUL(planesPerVolume); P(" planes + "); outUL(flybackFrames);
  P(" flyback = "); outUL(framesPerVolume); P(" frames/volume; fire on frame ");
  outUL(firePointFrame); P(" of every "); outUL(stimEveryNthVolume);
  P(" volumes (every "); outUL(framesPerVolume * stimEveryNthVolume); P(" frames)."); outNL();
  P("   Pulse: "); outUL(pulseDurationMs); P(" ms.  Gate: ");
  if (gateOverride == GATE_FORCE_ON)       P("FORCED ON");
  else if (gateOverride == GATE_FORCE_OFF) P("FORCED OFF");
  else                                     outStrP(gatePinStable == HIGH ? PSTR("pin 9 HIGH")
                                                                         : PSTR("pin 9 LOW"));
  P("  -> "); outStrP(stimArmed ? PSTR("ACTIVE") : PSTR("ARMED-IDLE (waiting for pin 9)"));
  outNL();
}

void cmdStop() {
  bool wasSomething = running || (optoState != OPTO_OFF);

  frameClkInterruptEnable(false);
  pulseAbort();
  freeRunPinOn = false;
  stimArmed    = false;
  pendingRamp  = false;
  setActiveDivisor(baseDivisor);

  if (!wasSomething) { P(">> Nothing to stop."); outNL(); return; }

  P(">> STOP. All outputs off."); outNL();

  if (running) {
    running = false;
    requestBlock(BLK_SUMMARY);
    summaryPrinted = true;
  }
  optoState = OPTO_OFF;
}


// ============================================================================================
//   FREE-RUN MODE  ('r <hz>,<ms>')
//   Independent of both the enable pin and the frame clock. ON phases are Timer1-terminated
//   for accuracy; OFF phases are timed with millis() in loop(), which is entirely adequate for
//   a manual bench mode and avoids chaining Timer1 reloads for periods beyond 4.194 s.
// ============================================================================================
void serviceFreeRun(unsigned long nowMs) {
  if (optoState != OPTO_FREERUN && optoState != OPTO_FREERUN_RAMP) return;

  // 100 % (or over-100 %) duty: hold the output on continuously. As a ramp stretches the
  // period, duty falls below 100 % and this automatically becomes pulsed again.
  if (freeRunOnMs >= freeRunPeriodMs) {
    if (!manualHold) { pulseHoldOn(); freeRunPulses++; }
    return;
  }
  if (manualHold) { pulseAbort(); }   // leaving 100 % duty

  if ((long)(nowMs - freeRunNextMs) >= 0) {
    freeRunNextMs = nowMs + freeRunPeriodMs;
    unsigned long on = freeRunOnMs;
    if (on > PULSE_MS_MAX) on = PULSE_MS_MAX;
    uint8_t s = SREG; cli(); pulseStart(MS_TO_TICKS(on)); SREG = s;
    freeRunPulses++;
  }
}


// ============================================================================================
//   MANUAL TEST  ('o' / 'o<sec>')
// ============================================================================================
void serviceManual(unsigned long nowMs) {
  if (!manualHold) return;
  if ((long)(nowMs - manualUntilMs) >= 0) {
    pulseAbort();
    P("[TEST] Manual latch ended."); outNL();
  }
}


// ============================================================================================
//   ANOMALY BOOKKEEPING DONE IN LOOP CONTEXT
// ============================================================================================
void serviceAnomalies(unsigned long nowMs) {
  unsigned long fc = atomicGet32(&frameCount);

  // First edge of the run: anchor the clock used for the inferred rates.
  if (!haveFirstEdgeMs && fc > 0UL) {
    haveFirstEdgeMs = true;
    firstEdgeMs     = nowMs;
    lastFrameSeen   = fc;
    lastFrameSeenMs = nowMs;
  }

  // ---- quantify phase slip from the anomaly ring ----
  // An interval of ~2x nominal means one edge was never seen, ~3x means two, and so on. This
  // is the ONLY way to detect a missed edge: the frame counter is the reference, so when an
  // edge is missed the counter simply does not advance and every count-based check still
  // agrees with itself while the grid has slipped behind the microscope.
  unsigned long nom = atomicGet32(&nominalUs);
  while (anomTail != anomHead && nom > 0UL) {
    unsigned long dt;
    uint8_t sr = SREG;
    cli();
    dt = anomRing[anomTail];
    anomTail = (uint8_t)((anomTail + 1) % ANOM_SLOTS);
    SREG = sr;

    // round(dt / nominal) is how many frame periods the gap spanned; 1 is normal.
    unsigned long periods = (dt + nom / 2UL) / nom;
    if (periods > 1UL) {
      unsigned long lost = periods - 1UL;
      framesMissed += lost;
      P("[!! SLIP] Gap of "); outCenti(usToCentiMs(dt));
      P(" ms at frame "); outUL(fc); P(" -> "); outUL(lost);
      P(" frame(s) missed. Grid is now "); outUL(framesMissed);
      P(" frame(s) behind the microscope."); outNL();
    } else if (periods == 0UL) {
      framesSpurious++;
      P("[!! SLIP] Short gap of "); outCenti(usToCentiMs(dt));
      P(" ms at frame "); outUL(fc); P(" -> suspected spurious edge (total ");
      outUL(framesSpurious); P(")."); outNL();
    }
  }

  // ---- delivery bookkeeping and spacing check ----
  // Consecutive deliveries should be exactly framesPerVolume * activeDivisor apart. Note the
  // limitation: this checks the grid against ITSELF, so it catches logic faults, divisor
  // changes and gate gaps, but it cannot see a missed edge. The slip accounting above is what
  // covers that. Grid changes set gridDisturbed so the first delivery after them is exempt.
  bool got;
  uint8_t s = SREG; cli(); got = deliveryFlag; deliveryFlag = false; SREG = s;
  if (got) {
    deliveriesSeen++;
    lastDeliveryMs = nowMs;
    if (deliveriesSeen == 1UL) firstDeliveryMs = nowMs;

    unsigned long sp  = atomicGet32(&lastSpacing);
    unsigned long exp = atomicGet32(&framesPerVolume) * atomicGet32(&activeDivisor);
    if (gridDisturbed) {
      gridDisturbed = false;          // this delivery re-anchors the check
    } else if (sp != exp) {
      spacingMismatches++;
      P("[!! GRID] Delivery spacing "); outUL(sp); P(" frames, expected "); outUL(exp);
      P("  (frame "); outUL(atomicGet32(&lastDeliveryFrame)); P(")"); outNL();
    }
  }

  // ---- frame clock stall detection ----
  // Uses only loop-side observation of frameCount, so it costs the ISR nothing.
  if (fc != lastFrameSeen) {
    lastFrameSeen   = fc;
    lastFrameSeenMs = nowMs;
    if (inStall) {
      inStall = false;
      P("[CLK] Frame clock resumed."); outNL();
    }
  } else if (haveFirstEdgeMs && !inStall) {
    unsigned long nomMs = nominalLocked ? (atomicGet32(&nominalUs) / 1000UL) : 100UL;
    if (nomMs < 2UL) nomMs = 2UL;
    if ((nowMs - lastFrameSeenMs) > (3UL * nomMs)) {
      inStall = true;
      stallEvents++;
      P("[!! CLK] Frame clock stalled at frame "); outUL(fc); P("."); outNL();
    }
  }
}


// ============================================================================================
//   BLOCK CONTENT
// ============================================================================================

void emitHelpLine(uint8_t i) {
  switch (i) {
    case 0: P("=============== VR_Opto_v5  COMMANDS ==============="); break;
    case 1: P("  s            Start run (resets counters)"); break;
    case 2: P("  x            Stop everything + summary"); break;
    case 3: P("  q            Ramp down now"); break;
    case 4: P("  v            Status now      ?   Help + settings"); break;
    case 5: P("  d <ms>       Pulse duration 1-4000  [LIVE]"); break;
    case 6: P("  n <planes>   Planes per volume      [locked while running]"); break;
    case 7: P("  y <frames>   Flyback frames/volume  [locked while running]"); break;
    case 8: P("  e <n>        Stim every Nth volume  [locked while running]"); break;
    case 9: P("  a <frames>   Phase offset           [locked while running]"); break;
    case 10: P("  t <ms>       Ramp duration          [locked while running]"); break;
    case 11: P("  k <steps>    Ramp steps             [locked while running]"); break;
    case 12: P("  m <factor>   Ramp multiplier        [locked while running]"); break;
    case 13: P("  o            One test pulse of d ms"); break;
    case 14: P("  o<sec>       Manual latch ON for <sec> s"); break;
    case 15: P("  r <hz>,<ms>  Free-run until x/q (not while running)"); break;
    case 16: P("  g a|1|0      Gate: auto / force on / force off"); break;
    case 17: P("  (commas and spaces are interchangeable)"); break;
    case 18: P("==================================================="); break;
    default: blockKind = BLK_NONE; return;
  }
  outNL();
}

void emitSettingsLine(uint8_t i) {
  unsigned long fpv = planesPerVolume + flybackFrames;
  switch (i) {
    case 0: P("--- Volume structure ---"); break;
    case 1: P("  planes/volume    n = "); outUL(planesPerVolume); break;
    case 2: P("  flybacks/volume  y = "); outUL(flybackFrames);
            P("   => "); outUL(fpv); P(" frames/volume"); break;
    case 3: P("  stim every       e = "); outUL(stimEveryNthVolume);
            P(" volumes  => fire every "); outUL(fpv * stimEveryNthVolume); P(" frames"); break;
    case 4: P("  phase offset     a = "); outUL(phaseOffsetFrames); P(" frames"); break;
    case 5: P("  fire point         = frame "); outUL(planesPerVolume + 1UL);
            P(" of "); outUL(fpv); P("  (first flyback)"); break;
    case 6: P("--- Pulse ---"); break;
    case 7: P("  duration         d = "); outUL(pulseDurationMs);
            P(" ms  (Timer1, 1-4000 ms)"); break;
    case 8: P("--- Ramp-down ---"); break;
    case 9: P("  duration         t = "); outUL(rampDurationMs); P(" ms"); break;
    case 10: P("  steps            k = "); outUL(rampSteps);
             P("   step = "); outUL(rampStepDurMs()); P(" ms"); break;
    case 11: P("  multiplier       m = "); outUL(rampMultiplier); break;
    case 12: {
      P("  schedule           = every ");
      for (unsigned long st = 0; st < rampSteps && st < 8UL; st++) {
        unsigned long d = safePow(stimEveryNthVolume, rampMultiplier, st);
        if (d > 65535UL) d = 65535UL;
        outUL(d);
        if (st + 1UL < rampSteps) P(" -> ");
      }
      P(" volumes, then OFF");
      break;
    }
    case 13: P("--- Frame clock ---"); break;
    case 14: P("  refractory lockout = "); outUL(refractoryUs / 1000UL); P(" ms"); break;
    case 15: P("  anomaly tolerance  = +/-"); outUL(INTERVAL_TOL_PERCENT); P(" %"); break;
    case 16: P("--- Gate (pin 9) ---"); break;
    case 17: P("  override           = ");
             if (gateOverride == GATE_FORCE_ON)       P("FORCED ON");
             else if (gateOverride == GATE_FORCE_OFF) P("FORCED OFF");
             else                                     P("AUTO (follows pin 9)");
             break;
    case 18: P("  pin 9 level now    = ");
             outStrP(digitalRead(PIN_ENABLE) == HIGH ? PSTR("HIGH") : PSTR("LOW")); break;
    default: blockKind = BLK_NONE; return;
  }
  outNL();
}

const char *optoStateNameP() {
  switch (optoState) {
    case OPTO_OFF:           return PSTR("OFF (no run)");
    case OPTO_ARMED:         return PSTR("ARMED-IDLE (gate low)");
    case OPTO_ACTIVE:        return PSTR("ACTIVE");
    case OPTO_RAMP:          return PSTR("RAMP-DOWN");
    case OPTO_FREERUN:       return PSTR("FREE-RUN");
    case OPTO_FREERUN_RAMP:  return PSTR("FREE-RUN RAMP");
  }
  return PSTR("?");
}

void emitStatusLine(uint8_t i) {
  unsigned long nowMs = millis();
  unsigned long fc  = atomicGet32(&frameCount);
  unsigned long vc  = atomicGet32(&volumeCount);
  unsigned long fiv = atomicGet32(&frameInVolume);
  unsigned long fpv = atomicGet32(&framesPerVolume);
  unsigned long dv  = atomicGet32(&deliveries);
  unsigned long span = haveFirstEdgeMs ? (nowMs - firstEdgeMs) : 0UL;

  switch (i) {
    case 0: {
      char t[12];
      formatHMS(running ? (nowMs - runStartMs) : 0UL, t, sizeof(t));
      P("[STATUS "); outStr(t); P("]  frame "); outUL(fc);
      P(" = volume "); outUL(vc); P(", frame "); outUL(fiv); P(" of "); outUL(fpv);
      break;
    }
    case 1: {
      // Frame rate is measured directly. Volume rate is derived from it rather than from the
      // completed-volume count, which is biased low early in a run by the first partial
      // volume. Stim rate is measured between the first and last delivery, so a long
      // gate-off baseline does not drag it down.
      unsigned long fHz = centiRate(fc, span);
      P("  rates: frame "); outCenti(fHz);
      P(" Hz | volume "); outCenti(fpv > 0UL ? fHz / fpv : 0UL);
      P(" Hz | stim ");
      if (deliveriesSeen >= 2UL && lastDeliveryMs > firstDeliveryMs)
        outCenti(centiRate(deliveriesSeen - 1UL, lastDeliveryMs - firstDeliveryMs));
      else P("--");
      P(" Hz");
      break;
    }
    case 2: {
      P("  opto: "); outStrP(optoStateNameP());
      P(" | every "); outUL(atomicGet32(&activeDivisor)); P(" vol");
      P(" | pulses "); outUL(dv);
      if (optoState == OPTO_RAMP || optoState == OPTO_FREERUN_RAMP) {
        P(" | ramp step "); outUL(rampStepNow + 1UL); P("/"); outUL(rampSteps);
        unsigned long el = nowMs - rampStartMs;
        P(", "); outUL((rampDurationMs > el ? rampDurationMs - el : 0UL) / 1000UL);
        P(" s left");
      }
      break;
    }
    case 3: {
      unsigned long mn = atomicGet32(&minIntervalUs);
      unsigned long mx = atomicGet32(&maxIntervalUs);
      P("  interval: ");
      if (mx == 0UL) { P("(no data)"); }
      else {
        outCenti(usToCentiMs(mn)); P(" / ");
        outCenti(usToCentiMs(atomicGet32(&lastIntervalUs))); P(" / ");
        outCenti(usToCentiMs(mx)); P(" ms  min/last/max");
        if (nominalLocked) { P("  nominal "); outCenti(usToCentiMs(atomicGet32(&nominalUs)));
                             P(" ms"); }
        else               { P("  (calibrating "); outUL(calibCount); P("/32)"); }
      }
      break;
    }
    case 4: {
      P("  SLIP: "); outUL(framesMissed); P(" frame(s) missed, ");
      outUL(framesSpurious); P(" spurious");
      if (framesMissed == 0UL && framesSpurious == 0UL) P("   [grid in sync]");
      break;
    }
    case 5: {
      P("  checks: grid "); outUL(spacingMismatches);
      P(" | long "); outUL(atomicGet32(&longIntervals));
      P(" | short "); outUL(atomicGet32(&shortIntervals));
      P(" | bounce "); outUL(atomicGet32(&refractoryDrops));
      P(" | overlap "); outUL(atomicGet32(&overlapRestarts));
      P(" | stalls "); outUL(stallEvents);
      if (outDropped) { P(" | TXDROP "); outUL(outDropped); }
      break;
    }
    default: blockKind = BLK_NONE; return;
  }
  outNL();
}

void emitSummaryLine(uint8_t i) {
  unsigned long nowMs = millis();
  unsigned long fc  = atomicGet32(&frameCount);
  unsigned long vc  = atomicGet32(&volumeCount);
  unsigned long dv  = atomicGet32(&deliveries);
  unsigned long span = haveFirstEdgeMs ? (nowMs - firstEdgeMs) : 0UL;
  unsigned long fpv = atomicGet32(&framesPerVolume);

  switch (i) {
    case 0: P("============== RUN SUMMARY =============="); break;
    case 1: {
      char t[12];
      formatHMS(runStartMs ? (nowMs - runStartMs) : 0UL, t, sizeof(t));
      P("  duration            "); outStr(t);
      break;
    }
    case 2: P("  frames counted      "); outUL(fc); break;
    case 3: P("  volumes completed   "); outUL(vc > 0UL ? vc - 1UL : 0UL);
            P("  (partial: frame "); outUL(atomicGet32(&frameInVolume));
            P(" of "); outUL(fpv); P(")"); break;
    case 4: P("  pulses delivered    "); outUL(dv); break;
    case 5: {
      // Expected deliveries over the frames actually counted, for a quick sanity check.
      unsigned long per = fpv * atomicGet32(&activeDivisor);
      unsigned long expd = (per > 0UL) ? (fc / per) : 0UL;
      P("  pulses expected     ~"); outUL(expd);
      P("  (upper bound; gate-off baseline reduces this)");
      break;
    }
    case 6: P("  manual test pulses  "); outUL(manualPulses);
            P("   free-run pulses "); outUL(freeRunPulses); break;
    case 7: P("--- measured rates ---"); break;
    case 8: P("  frame rate          "); outCenti(centiRate(fc, span)); P(" Hz"); break;
    case 9: P("  volume rate         ");
            outCenti(fpv > 0UL ? centiRate(fc, span) / fpv : 0UL);
            P(" Hz   (frame rate / "); outUL(fpv); P(")"); break;
    case 10: P("  stim rate           ");
             if (deliveriesSeen >= 2UL && lastDeliveryMs > firstDeliveryMs)
               outCenti(centiRate(deliveriesSeen - 1UL, lastDeliveryMs - firstDeliveryMs));
             else P("--");
             P(" Hz   (between first and last delivery)"); break;
    case 11: P("--- integrity ---"); break;
    case 12: P("  PHASE SLIP          "); outUL(framesMissed);
             P(" frame(s) missed, "); outUL(framesSpurious); P(" spurious");
             if (framesMissed == 0UL && framesSpurious == 0UL)
               P("   <-- grid stayed in sync");
             else
               P("   <-- the volume grid drifted from the microscope by this much");
             break;
    case 13: P("  long intervals      "); outUL(atomicGet32(&longIntervals));
             P("   (events; each one is a suspected MISSED edge)"); break;
    case 14: P("  short intervals     "); outUL(atomicGet32(&shortIntervals));
             P("   (events; suspected SPURIOUS edges)"); break;
    case 15: P("  bounce rejected     "); outUL(atomicGet32(&refractoryDrops));
             P("   (10 ms refractory lockout)"); break;
    case 16: P("  irregular spacing   "); outUL(spacingMismatches);
             P("   (deliveries not "); outUL(fpv * atomicGet32(&activeDivisor));
             P(" frames apart; expected across ramp steps and gate gaps)"); break;
    case 17: P("  overlap restarts    "); outUL(atomicGet32(&overlapRestarts));
             P("   clock stalls "); outUL(stallEvents); break;
    case 18: {
      unsigned long mn = atomicGet32(&minIntervalUs);
      unsigned long mx = atomicGet32(&maxIntervalUs);
      P("  interval min/max    ");
      if (mx == 0UL) P("(no data)");
      else { outCenti(usToCentiMs(mn)); P(" / "); outCenti(usToCentiMs(mx)); P(" ms"); }
      if (nominalLocked) { P("   nominal "); outCenti(usToCentiMs(atomicGet32(&nominalUs)));
                           P(" ms"); }
      break;
    }
    case 19: if (outDropped) { P("  serial bytes dropped "); outUL(outDropped); break; }
             else { P("========================================="); blockThen = BLK_NONE;
                    blockKind = BLK_NONE; outNL(); return; }
    case 20: P("========================================="); break;
    default: blockKind = BLK_NONE; return;
  }
  outNL();
}

// One line per loop pass, and only when the ring has room for a full line.
void serviceBlocks() {
  if (blockKind == BLK_NONE) return;
  if (outFree() < 110) return;

  uint8_t i = blockLine++;
  BlockKind k = blockKind;
  switch (k) {
    case BLK_HELP:     emitHelpLine(i);     break;
    case BLK_SETTINGS: emitSettingsLine(i); break;
    case BLK_STATUS:   emitStatusLine(i);   break;
    case BLK_SUMMARY:  emitSummaryLine(i);  break;
    default: blockKind = BLK_NONE; return;
  }
  if (blockKind == BLK_NONE && blockThen != BLK_NONE) {
    BlockKind t = blockThen;
    blockThen = BLK_NONE;
    requestBlock(t);
  }
}


// ============================================================================================
//   SERIAL COMMAND PARSING
//   Non-blocking character accumulator. readStringUntil() is deliberately avoided: it relies
//   on a 1000 ms Stream timeout, so with the Serial Monitor set to "No Line Ending" every
//   command would stall the loop for a full second. A fixed char buffer also keeps String off
//   the heap, which matters on a 2 KB part.
// ============================================================================================
const uint8_t CMDBUF_SIZE = 40;
char    cmdBuf[CMDBUF_SIZE];
uint8_t cmdLen = 0;

// Commas and spaces are interchangeable separators.
bool isSep(char c) { return c == ' ' || c == ',' || c == '\t' || c == ';'; }

// Read the nth (0-based) numeric field after the command letter. Returns false if absent.
bool argN(const char *s, uint8_t n, long *out) {
  const char *p = s + 1;                       // skip the command letter
  for (uint8_t f = 0; ; f++) {
    while (*p && isSep(*p)) p++;               // skip separators
    if (!*p) return false;
    const char *start = p;
    while (*p && !isSep(*p)) p++;
    if (f == n) {
      if (!(isdigit((unsigned char)*start) ||
            ((*start == '-' || *start == '+') && isdigit((unsigned char)start[1])))) return false;
      *out = atol(start);
      return true;
    }
  }
}

bool lockedWhileRunning(const char *nameP) {
  if (!running) return false;
  P("!! "); outStrP(nameP);
  P(" is locked while a run is in progress. Send 'x' first."); outNL();
  return true;
}

void warnIfDurationTooLong() {
  unsigned long per = atomicGet32(&framesPerVolume) * atomicGet32(&activeDivisor);
  unsigned long nom = nominalLocked ? atomicGet32(&nominalUs) : 0UL;
  if (nom == 0UL || per == 0UL) return;
  unsigned long interStimMs = (per * nom) / 1000UL;
  if (pulseDurationMs >= interStimMs) {
    P("!! WARNING: d = "); outUL(pulseDurationMs);
    P(" ms >= measured inter-stim interval "); outUL(interStimMs);
    P(" ms. Every fire point will land mid-pulse and, with restart-on-overlap, the output "
      "will be effectively continuous."); outNL();
  }
}

void handleCommand(char *s) {
  // lower-case the command letter only; arguments are numeric
  char c = (char)tolower((unsigned char)s[0]);
  long v = 0, v2 = 0;
  unsigned long nowMs = millis();

  switch (c) {

    case 's': cmdStart(nowMs); return;

    case 'x': cmdStop();       return;

    case '?': requestBlock(BLK_HELP, BLK_SETTINGS); return;

    case 'v': requestBlock(BLK_STATUS); return;

    // ---- ramp down now ----
    case 'q':
      if (optoState == OPTO_FREERUN)      { beginRampFreeRun(nowMs); }
      else if (optoState == OPTO_ACTIVE)  { requestRampDown(nowMs, PSTR("Command 'q'")); }
      else if (optoState == OPTO_RAMP || optoState == OPTO_FREERUN_RAMP) {
        P(">> Already ramping down."); outNL();
      } else {
        P(">> Nothing to ramp down (opto is not active)."); outNL();
      }
      return;

    // ---- pulse duration: the only live-editable parameter ----
    case 'd':
      if (!argN(s, 0, &v) || v < 1 || v > (long)PULSE_MS_MAX) {
        P("!! Use: d 295   (1-"); outUL(PULSE_MS_MAX);
        P(" ms; the ceiling is Timer1 at /1024)"); outNL();
        return;
      }
      pulseDurationMs = (unsigned long)v;
      // Applies from the NEXT pulse; a pulse already in flight keeps its original length.
      pulseTicks = MS_TO_TICKS(pulseDurationMs);
      P(">> d = "); outUL(pulseDurationMs); P(" ms (from the next pulse)"); outNL();
      warnIfDurationTooLong();
      return;

    // ---- structural parameters: locked during a run ----
    case 'n':
      if (lockedWhileRunning(PSTR("n (planes/volume)"))) return;
      if (!argN(s, 0, &v) || v < 1) { P("!! Use: n 5"); outNL(); return; }
      planesPerVolume = (unsigned long)v; recomputeGrid();
      P(">> n = "); outUL(planesPerVolume); P(" planes; ");
      outUL(framesPerVolume); P(" frames/volume; fire on frame ");
      outUL(firePointFrame); outNL();
      return;

    case 'y':
      if (lockedWhileRunning(PSTR("y (flybacks/volume)"))) return;
      if (!argN(s, 0, &v) || v < 1) {
        P("!! Use: y 2   (must be >= 1 so a fire point exists)"); outNL(); return;
      }
      flybackFrames = (unsigned long)v; recomputeGrid();
      P(">> y = "); outUL(flybackFrames); P("; ");
      outUL(framesPerVolume); P(" frames/volume"); outNL();
      return;

    case 'e':
      if (lockedWhileRunning(PSTR("e (stim every Nth volume)"))) return;
      if (!argN(s, 0, &v) || v < 1) { P("!! Use: e 2"); outNL(); return; }
      stimEveryNthVolume = (unsigned long)v; recomputeGrid();
      P(">> e = "); outUL(stimEveryNthVolume); P(" volumes = every ");
      outUL(framesPerVolume * stimEveryNthVolume); P(" frames"); outNL();
      return;

    case 'a':
      if (lockedWhileRunning(PSTR("a (phase offset)"))) return;
      if (!argN(s, 0, &v) || v < 0) { P("!! Use: a 0"); outNL(); return; }
      phaseOffsetFrames = (unsigned long)v;
      P(">> a = "); outUL(phaseOffsetFrames);
      P(" frames; the first counted edge will be frame ");
      outUL(phaseOffsetFrames + 1UL); outNL();
      return;

    case 't':
      if (lockedWhileRunning(PSTR("t (ramp duration)"))) return;
      if (!argN(s, 0, &v) || v < 1) { P("!! Use: t 300000"); outNL(); return; }
      rampDurationMs = (unsigned long)v;
      P(">> t = "); outUL(rampDurationMs); P(" ms; step = ");
      outUL(rampStepDurMs()); P(" ms"); outNL();
      return;

    case 'k':
      if (lockedWhileRunning(PSTR("k (ramp steps)"))) return;
      if (!argN(s, 0, &v) || v < 1) { P("!! Use: k 5"); outNL(); return; }
      rampSteps = (unsigned long)v;
      P(">> k = "); outUL(rampSteps); P(" steps; step = ");
      outUL(rampStepDurMs()); P(" ms"); outNL();
      return;

    case 'm':
      if (lockedWhileRunning(PSTR("m (ramp multiplier)"))) return;
      if (!argN(s, 0, &v) || v < 2) {
        P("!! Use: m 2   (>= 2, or the ramp would not reduce the rate)"); outNL(); return;
      }
      rampMultiplier = (unsigned long)v;
      P(">> m = "); outUL(rampMultiplier); outNL();
      return;

    // ---- gate override ----
    case 'g': {
      const char *p = s + 1;
      while (*p && isSep(*p)) p++;
      char g = (char)tolower((unsigned char)*p);
      if (g == 'a') {
        gateOverride = GATE_AUTO;
        P(">> Gate override AUTO (follows pin 9, now ");
        outStrP(gatePinStable == HIGH ? PSTR("HIGH") : PSTR("LOW")); P(")"); outNL();
        // Apply the pin level immediately.
        if (running) {
          if (gatePinStable == HIGH) {
            if (!cancelRampAnyForm(nowMs, PSTR("g a with pin 9 HIGH"))
                && optoState == OPTO_ARMED) { optoState = OPTO_ACTIVE; stimArmed = true; }
          } else if (optoState == OPTO_ACTIVE) {
            requestRampDown(nowMs, PSTR("g a with pin 9 LOW"));
          }
        }
      } else if (g == '1') {
        gateOverride = GATE_FORCE_ON;
        P(">> Gate FORCED ON (pin 9 ignored)"); outNL();
        if (running) {
          if (!cancelRampAnyForm(nowMs, PSTR("g 1")) && optoState == OPTO_ARMED) {
            optoState = OPTO_ACTIVE; stimArmed = true;
            P("[GATE] ACTIVE from the next scheduled fire point."); outNL();
          }
        }
      } else if (g == '0') {
        gateOverride = GATE_FORCE_OFF;
        P(">> Gate FORCED OFF -> ramping down (only 'x' stops everything outright)"); outNL();
        if (running && optoState == OPTO_ACTIVE) requestRampDown(nowMs, PSTR("g 0"));
      } else {
        P("!! Use: g a | g 1 | g 0"); outNL();
      }
      return;
    }

    // ---- manual opto test ----
    case 'o': {
      if (optoState == OPTO_RAMP || optoState == OPTO_FREERUN_RAMP) {
        P("!! Rejected: a ramp-down is in progress."); outNL(); return;
      }
      if (optoState == OPTO_FREERUN) {
        P("!! Rejected: free-run is active. Send 'x' or 'q' first."); outNL(); return;
      }
      if (optoState == OPTO_ACTIVE) {
        P("!! Rejected: scheduled stimulation is ACTIVE. A manual pulse here would land "
          "off-grid. Test during baseline (pin 9 LOW) or after 'x'."); outNL();
        return;
      }
      bool inFlight;
      uint8_t sr = SREG; cli(); inFlight = pulseActive || manualHold; SREG = sr;
      if (inFlight) { P("!! Rejected: a pulse is already in flight."); outNL(); return; }

      if (!argN(s, 0, &v)) {
        // bare 'o' -> one pulse of d ms
        sr = SREG; cli(); pulseStart(MS_TO_TICKS(pulseDurationMs)); SREG = sr;
        manualPulses++;
        P("[TEST] Single pulse, "); outUL(pulseDurationMs); P(" ms."); outNL();
        return;
      }
      if (v < 1 || v > 300) { P("!! Use: o   or  o5   (1-300 s)"); outNL(); return; }
      manualUntilMs = nowMs + (unsigned long)v * 1000UL;
      pulseHoldOn();
      manualPulses++;
      P("[TEST] Manual latch ON for "); outUL((unsigned long)v);
      P(" s (or send 'x')."); outNL();
      return;
    }

    // ---- free-run ----
    case 'r': {
      if (running) {
        P("!! Rejected: free-run would corrupt the trial structure. Send 'x' first."); outNL();
        return;
      }
      if (!argN(s, 0, &v) || !argN(s, 1, &v2) || v < 1 || v2 < 1) {
        P("!! Use: r 2,250   or  r 2 250   (<hz>, <on-ms>)"); outNL(); return;
      }
      if (v > 500) { P("!! Max 500 Hz."); outNL(); return; }
      freeRunBasePerMs = 1000UL / (unsigned long)v;
      if (freeRunBasePerMs < 1UL) freeRunBasePerMs = 1UL;
      freeRunPeriodMs  = freeRunBasePerMs;
      freeRunOnMs      = (unsigned long)v2;
      freeRunNextMs    = nowMs;
      freeRunPulses    = 0UL;
      optoState        = OPTO_FREERUN;
      pulseAbort();
      P("[FREE] "); outUL((unsigned long)v); P(" Hz, period ");
      outUL(freeRunPeriodMs); P(" ms, ON "); outUL(freeRunOnMs); P(" ms. ");
      if (freeRunOnMs >= freeRunPeriodMs) {
        P("100% duty -> output held CONTINUOUSLY ON. ");
      }
      P("'q' ramps down, 'x' stops."); outNL();
      return;
    }
  }

  P("!! Unknown command '"); outStr(s); P("'. Send '?' for the command list."); outNL();
}

void serviceSerialIn() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; handleCommand(cmdBuf); cmdLen = 0; }
      continue;
    }
    if (cmdLen < CMDBUF_SIZE - 1) cmdBuf[cmdLen++] = c;
  }
}


// ============================================================================================
//   SETUP
// ============================================================================================
void setup() {
  // Pin 13 LOW before anything else, so the output is defined as early as possible.
  pinMode(PIN_OPTO, OUTPUT);
  OPTO_LOW();

  // Plain INPUT on both inputs, matching the previously working VR_Opto_4 configuration.
  pinMode(PIN_FRAMECLK, INPUT);
  pinMode(PIN_ENABLE,   INPUT);

  // Timer1 fully disarmed until a pulse needs it. TCCR1A = 0 leaves COM1A cleared, which is
  // what keeps OC1A (Arduino pin 9, our enable input) undriven.
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;

  Serial.begin(SERIAL_BAUD);

  gateRawLast   = digitalRead(PIN_ENABLE);
  gatePinStable = gateRawLast;
  gateChangedMs = millis();
  gateOverride  = GATE_AUTO;          // always AUTO after reset / upload

  recomputeGrid();
  pulseTicks = MS_TO_TICKS(pulseDurationMs);

  P("\r\n============================================================"); outNL();
  P("  VR_Opto_v5 - volume-locked optogenetic stimulation"); outNL();
  P("  Arduino Uno | frame clk pin 6 | enable pin 9 | opto pin 13"); outNL();
  P("  SERIAL MONITOR MUST BE AT 250000 BAUD"); outNL();
  P("============================================================"); outNL();
  requestBlock(BLK_HELP, BLK_SETTINGS);
}


// ============================================================================================
//   MAIN LOOP
//   Nothing here is timing-critical: frame counting and pulse start live in PCINT2, pulse end
//   lives in TIMER1_COMPA. The loop only does bookkeeping, gate debouncing, ramp progression
//   and serial I/O, so it can be delayed without affecting the stimulus at all.
// ============================================================================================
void loop() {
  unsigned long nowMs = millis();

  // 1. Serial input (non-blocking accumulator)
  serviceSerialIn();

  // 2. Enable-pin debounce and edge handling
  serviceGate(nowMs);

  // 3. A ramp that was deferred because a pulse was in flight
  if (pendingRamp) {
    bool inFlight;
    uint8_t s = SREG; cli(); inFlight = pulseActive || manualHold; SREG = s;
    if (!inFlight) { pendingRamp = false; beginRampFrameLocked(nowMs); }
  }

  // 4. Ramp progression
  serviceRamp(nowMs);

  // 5. Free-run and manual-latch modes
  serviceFreeRun(nowMs);
  serviceManual(nowMs);

  // 6. Nominal-interval calibration, once 32 intervals are in
  if (running) tryLockNominal();

  // 7. Anomaly bookkeeping (phase slip, stalls, rate anchor)
  if (running) serviceAnomalies(nowMs);

  // 8. Periodic status report
  if (running && (nowMs - lastStatusMs) >= STATUS_INTERVAL_MS) {
    lastStatusMs = nowMs;
    if (blockKind == BLK_NONE) requestBlock(BLK_STATUS);
  }

  // 9. Emit at most one queued block line, then drain the ring to the UART.
  serviceBlocks();
  serviceSerialOut();
}
