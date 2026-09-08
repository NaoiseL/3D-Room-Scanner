/*
  Tilt axis test firmware - DRV8825 + NEMA 17 + Arduino Uno

  Gearing: 20T pinion / 48T gear = 2.4:1
  Motor:   1.8 deg/step, 200 full steps/rev
  Driver:  1/8 microstepping

  One full step  = 1.8 / 2.4          = 0.75 deg of tilt
  One microstep  = 0.75 / 8           = 0.09375 deg
  Full sweep     = 180 deg            = 1920 microsteps

  Always stop on a multiple of 8 microsteps so the axis lands on a
  real full-step detent. The DRV8825's intermediate microsteps are
  not evenly spaced, so positions between detents are not trustworthy.

  Serial commands at 115200:
    ?        status
    E1 / E0  enable / disable driver
    D1 / D0  direction
    S<n>     step n microsteps in the current direction
    F<n>     step n FULL steps (n*8 microsteps) - prefer this
    H        home against an endstop switch
    C        crash home against the mechanical hard stop
    Z        zero here, manual datum for when no endstop is fitted
    T        self test: quarter turn out and back
*/

const uint8_t PIN_STEP    = 3;
const uint8_t PIN_DIR     = 4;
const uint8_t PIN_ENABLE  = 5;   // active LOW
const uint8_t PIN_M0      = 8;
const uint8_t PIN_M1      = 9;
const uint8_t PIN_M2      = 10;
const uint8_t PIN_ENDSTOP = 2;   // to GND, uses internal pullup

const uint8_t  MICROSTEPS      = 8;
const int      PULSE_US        = 3;      // DRV8825 needs >1.9us
const unsigned STEP_INTERVAL   = 600;    // us between microsteps
const unsigned HOME_INTERVAL   = 1500;   // slower approach when homing
const long     MICROSTEPS_180  = 1920;

long position = 0;          // microsteps from home
bool homed    = false;
bool dirFwd   = true;

void setMicrostepping() {
  // DRV8825: 1/8 step is M0 HIGH, M1 HIGH, M2 LOW
  digitalWrite(PIN_M0, HIGH);
  digitalWrite(PIN_M1, HIGH);
  digitalWrite(PIN_M2, LOW);
}

void enableDriver(bool on) {
  digitalWrite(PIN_ENABLE, on ? LOW : HIGH);
}

bool endstopHit() {
  return digitalRead(PIN_ENDSTOP) == LOW;
}

void setDirection(bool forward) {
  dirFwd = forward;
  digitalWrite(PIN_DIR, forward ? HIGH : LOW);
  delayMicroseconds(5);      // DIR setup time before the next STEP edge
}

void oneStep(unsigned intervalUs) {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(intervalUs);
  position += dirFwd ? 1 : -1;
}

void move(long microsteps) {
  setDirection(microsteps >= 0);
  long n = microsteps < 0 ? -microsteps : microsteps;
  for (long i = 0; i < n; i++) {
    if (!dirFwd && endstopHit()) {
      Serial.println(F("stopped: endstop"));
      return;
    }
    oneStep(STEP_INTERVAL);
  }
}

bool home() {
  Serial.println(F("homing..."));
  setDirection(false);

  // Back off first in case we are already sitting on the switch.
  if (endstopHit()) {
    setDirection(true);
    for (int i = 0; i < 40 * MICROSTEPS; i++) oneStep(HOME_INTERVAL);
    setDirection(false);
  }

  long guard = MICROSTEPS_180 * 2;
  while (!endstopHit() && guard-- > 0) oneStep(HOME_INTERVAL);

  if (guard <= 0) {
    Serial.println(F("FAILED: no endstop found"));
    homed = false;
    return false;
  }

  // Back off and approach again slowly for a repeatable datum.
  setDirection(true);
  for (int i = 0; i < 16 * MICROSTEPS; i++) oneStep(HOME_INTERVAL);
  setDirection(false);
  while (!endstopHit()) oneStep(HOME_INTERVAL * 3);

  position = 0;
  homed = true;
  Serial.println(F("homed"));
  return true;
}

bool crashHome() {
  // Homing with no switch: drive into the mechanical hard stop and let the
  // motor skip steps. The DRV8825 holds current constant regardless of load,
  // so a stall is not detectable electrically. We simply command more travel
  // than the axis has and trust the stop to absorb it.
  //
  // Repeatable to roughly one full step. Good enough for bring-up, and a
  // constant offset gets absorbed by extrinsic calibration later, but a
  // contact switch is meaningfully better.
  Serial.println(F("crash homing..."));
  setDirection(false);

  const long overtravel = MICROSTEPS_180 + 8L * MICROSTEPS;
  for (long i = 0; i < overtravel; i++) oneStep(HOME_INTERVAL);

  // Ease off so the gears are not left loaded against the stop.
  setDirection(true);
  for (int i = 0; i < 2 * MICROSTEPS; i++) oneStep(HOME_INTERVAL);

  position = 0;
  homed = true;
  Serial.println(F("at hard stop, zeroed"));
  return true;
}

void status() {
  Serial.print(F("pos "));       Serial.print(position);
  Serial.print(F("  deg "));     Serial.print(position * 0.09375, 3);
  Serial.print(F("  homed "));   Serial.print(homed ? 'y' : 'n');
  Serial.print(F("  endstop ")); Serial.print(endstopHit() ? "HIT" : "open");
  Serial.print(F("  dir "));     Serial.println(dirFwd ? '+' : '-');
}

void setup() {
  pinMode(PIN_STEP,   OUTPUT);
  pinMode(PIN_DIR,    OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_M0,     OUTPUT);
  pinMode(PIN_M1,     OUTPUT);
  pinMode(PIN_M2,     OUTPUT);
  pinMode(PIN_ENDSTOP, INPUT_PULLUP);

  enableDriver(false);
  setMicrostepping();
  setDirection(true);

  Serial.begin(115200);
  while (!Serial) { }
  Serial.println(F("tilt axis ready. commands: ? E1 E0 D1 D0 S<n> F<n> H C Z T"));
  enableDriver(true);
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  switch (c) {
    case '?': status(); break;

    case 'E': {
      char v = Serial.read();
      enableDriver(v == '1');
      Serial.println(v == '1' ? F("enabled") : F("disabled"));
      break;
    }

    case 'D': {
      char v = Serial.read();
      setDirection(v == '1');
      Serial.println(dirFwd ? F("dir +") : F("dir -"));
      break;
    }

    case 'S': {
      long n = Serial.parseInt();
      move(dirFwd ? n : -n);
      status();
      break;
    }

    case 'F': {
      long n = Serial.parseInt();
      move((dirFwd ? n : -n) * MICROSTEPS);
      status();
      break;
    }

    case 'H': home(); status(); break;

    case 'C': crashHome(); status(); break;

    case 'Z':
      // Manual datum. Park the axis where you want zero by eye, then send Z.
      // Use this until a real endstop exists; it is repeatable only as far
      // as your eye is, so it is for bring-up, not for real scans.
      position = 0;
      homed = true;
      Serial.println(F("zeroed here (manual datum)"));
      break;

    case 'T': {
      Serial.println(F("self test: 60 full steps out and back"));
      move(60L * MICROSTEPS);
      delay(400);
      move(-60L * MICROSTEPS);
      status();
      break;
    }

    default: break;   // ignore newlines and stray characters
  }
}
