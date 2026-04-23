#include <Arduino.h>
#include <IntervalTimer.h>

constexpr uint8_t DM_OFF = HIGH;
constexpr uint8_t DM_ON  = LOW;

IntervalTimer stepTimer;
volatile bool stepState = false;
volatile bool steppingEnabled = false;

volatile long currentSteps = 0;
volatile long targetSteps = 0;
volatile bool currentDirForward = true;

volatile float targetSPS = 0.0f;

constexpr long MIN_STEPS = 0;
constexpr long MAX_STEPS = 1200;
constexpr float STEPS_PER_DEG = 10.0f;

enum GaitPhase {
  PHASE_STANCE_LOADING,
  PHASE_MID_STANCE,
  PHASE_TERMINAL_STANCE,
  PHASE_PRE_SWING,
  PHASE_INITIAL_SWING,
  PHASE_MID_SWING,
  PHASE_TERMINAL_SWING,
  PHASE_UNKNOWN
};

struct SensorData {
  float footLoadNorm;
  float shankAngleDeg;
  float shankVelDegPerSec;
};

struct PhaseOutput {
  GaitPhase phase;
  float theoreticalKneeAngleDeg;
};

struct MotorTestCommand {
  long targetStepPosition;
  float speedSPS;
  const char* modeLabel;
};

unsigned long lastSampleMs = 0;
float lastAngleDeg = 0.0f;

float filteredLoad = 0.0f;
constexpr float LOAD_ALPHA = 0.15f;

GaitPhase lastPhase = PHASE_UNKNOWN;

void stepISR() {
  if (!steppingEnabled) return;

  long cs = currentSteps;
  long ts = targetSteps;

  if (cs == ts) {
    steppingEnabled = false;
    stepState = false;
    digitalWriteFast(STEP_N_PIN, LOW);
    return;
  }

  stepState = !stepState;
  digitalWriteFast(STEP_N_PIN, stepState ? HIGH : LOW);

  if (stepState) {
    if (currentDirForward) {
      currentSteps++;
    } else {
      currentSteps--;
    }
  }
}

void enableDriver(bool enable) {
  digitalWriteFast(EN_N_PIN, enable ? DM_ON : DM_OFF);
}

void setDirection(bool forward) {
  digitalWriteFast(DIR_N_PIN, forward ? DM_ON : DM_OFF);
}

void setSpeedSPS(float sps) {
  targetSPS = sps;

  if (sps <= 0.0f) {
    stepTimer.end();
    noInterrupts();
    steppingEnabled = false;
    stepState = false;
    interrupts();
    digitalWriteFast(STEP_N_PIN, LOW);
    return;
  }

  float isrFreq = sps * 2.0f;  
  uint32_t period_us = (uint32_t)(1e6f / isrFreq);
  stepTimer.begin(stepISR, period_us);
}

void moveToSteps(long newTargetSteps, float sps) {
  newTargetSteps = constrain(newTargetSteps, MIN_STEPS, MAX_STEPS);

  long cs;
  noInterrupts();
  cs = currentSteps;
  interrupts();

  bool needMove = (newTargetSteps != cs);
  bool dirForward = (newTargetSteps > cs);

  if (needMove) {
    setDirection(dirForward);
  }

  noInterrupts();
  targetSteps = newTargetSteps;
  currentDirForward = dirForward;
  steppingEnabled = needMove;
  interrupts();

  setSpeedSPS(needMove ? sps : 0.0f);
}

float readMockShankAngleDeg() {  //Placeholder sensor stuff
  float t = millis() / 1000.0f;
  return 12.0f * sinf(2.0f * 3.14159f * 0.8f * t);
}

float readPressureNorm() {
  int raw = analogRead(PRESSURE_PIN);  
  float norm = constrain(raw / 1023.0f, 0.0f, 1.0f);

  filteredLoad = LOAD_ALPHA * norm + (1.0f - LOAD_ALPHA) * filteredLoad;
  return filteredLoad;
}

SensorData readSensors() {
  String data_string = read_sensors(); // From consolidated_tps.ino
  char *strings[4]; // an array of pointers to the pieces of the above array after strtok()
  char *ptr = NULL;
  byte index = 0;
  ptr = strtok(array, ",");  // delimiter
  while (ptr != NULL)
  {
    strings[index] = ptr;
    index++;
    ptr = strtok(NULL, ",");
  }
  s.footLoadNorm = atof(strings[0]);
  s.shankAngleDeg = atof(strings[2]);



  """
  SensorData s{};

  s.footLoadNorm = readPressureNorm();
  s.shankAngleDeg = readMockShankAngleDeg();
  """

  unsigned long now = millis();
  float dt = (now - lastSampleMs) / 1000.0f;

  if (dt > 0.001f) {
    s.shankVelDegPerSec = (s.shankAngleDeg - lastAngleDeg) / dt;
  } else {
    s.shankVelDegPerSec = 0.0f;
  }

  lastSampleMs = now;
  lastAngleDeg = s.shankAngleDeg;

  return s;
}

const char* phaseToString(GaitPhase p) {
  switch (p) {
    case PHASE_STANCE_LOADING: return "STANCE_LOADING";
    case PHASE_MID_STANCE: return "MID_STANCE";
    case PHASE_TERMINAL_STANCE: return "TERMINAL_STANCE";
    case PHASE_PRE_SWING: return "PRE_SWING";
    case PHASE_INITIAL_SWING: return "INITIAL_SWING";
    case PHASE_MID_SWING: return "MID_SWING";
    case PHASE_TERMINAL_SWING: return "TERMINAL_SWING";
    default: return "UNKNOWN";
  }
}

GaitPhase detectPhase(const SensorData& s) {
  bool loaded = s.footLoadNorm > 0.35f;
  bool unloading = s.footLoadNorm < 0.20f;
  bool shankForward = s.shankVelDegPerSec > 8.0f;
  bool shankBackward = s.shankVelDegPerSec < -8.0f;

  if (loaded) {
    if (s.shankAngleDeg < -4.0f) return PHASE_STANCE_LOADING;
    if (s.shankAngleDeg < 4.0f)  return PHASE_MID_STANCE;
    return PHASE_TERMINAL_STANCE;
  }

  if (!loaded && s.footLoadNorm > 0.10f) {
    return PHASE_PRE_SWING;
  }

  if (unloading) {
    if (shankForward && s.shankAngleDeg < 5.0f)  return PHASE_INITIAL_SWING;
    if (shankForward && s.shankAngleDeg >= 5.0f) return PHASE_MID_SWING;
    if (shankBackward || s.shankVelDegPerSec <= 3.0f) return PHASE_TERMINAL_SWING;
  }

  return PHASE_UNKNOWN;
}

PhaseOutput computePhaseOutput(const SensorData& s) {
  PhaseOutput out{};
  out.phase = detectPhase(s);

  switch (out.phase) {
    case PHASE_STANCE_LOADING:  out.theoreticalKneeAngleDeg = 10.0f; break;
    case PHASE_MID_STANCE:      out.theoreticalKneeAngleDeg = 5.0f;  break;
    case PHASE_TERMINAL_STANCE: out.theoreticalKneeAngleDeg = 0.0f;  break;
    case PHASE_PRE_SWING:       out.theoreticalKneeAngleDeg = 20.0f; break;
    case PHASE_INITIAL_SWING:   out.theoreticalKneeAngleDeg = 40.0f; break;
    case PHASE_MID_SWING:       out.theoreticalKneeAngleDeg = 60.0f; break;
    case PHASE_TERMINAL_SWING:  out.theoreticalKneeAngleDeg = 15.0f; break;
    default:                    out.theoreticalKneeAngleDeg = 0.0f;  break;
  }

  return out;
}

long angleDegToSteps(float angleDeg) {
  long steps = (long)(angleDeg * STEPS_PER_DEG);
  return constrain(steps, MIN_STEPS, MAX_STEPS);
}

MotorTestCommand buildMotorTestCommand(const PhaseOutput& out) {
  MotorTestCommand cmd{};

  switch (out.phase) {
    case PHASE_STANCE_LOADING:
      cmd.targetStepPosition = angleDegToSteps(10.0f);
      cmd.speedSPS = 250.0f;
      cmd.modeLabel = "LOAD_RESPONSE_TEST";
      break;

    case PHASE_MID_STANCE:
      cmd.targetStepPosition = angleDegToSteps(5.0f);
      cmd.speedSPS = 180.0f;
      cmd.modeLabel = "MID_STANCE_HOLD_TEST";
      break;

    case PHASE_TERMINAL_STANCE:
      cmd.targetStepPosition = angleDegToSteps(0.0f);
      cmd.speedSPS = 220.0f;
      cmd.modeLabel = "STANCE_EXTENSION_TEST";
      break;

    case PHASE_PRE_SWING:
      cmd.targetStepPosition = angleDegToSteps(20.0f);
      cmd.speedSPS = 350.0f;
      cmd.modeLabel = "PRE_SWING_FLEX_TEST";
      break;

    case PHASE_INITIAL_SWING:
      cmd.targetStepPosition = angleDegToSteps(40.0f);
      cmd.speedSPS = 500.0f;
      cmd.modeLabel = "INITIAL_SWING_DRIVE_TEST";
      break;

    case PHASE_MID_SWING:
      cmd.targetStepPosition = angleDegToSteps(60.0f);
      cmd.speedSPS = 550.0f;
      cmd.modeLabel = "MID_SWING_PEAK_FLEX_TEST";
      break;

    case PHASE_TERMINAL_SWING:
      cmd.targetStepPosition = angleDegToSteps(15.0f);
      cmd.speedSPS = 300.0f;
      cmd.modeLabel = "TERMINAL_SWING_RETURN_TEST";
      break;

    default:
      cmd.targetStepPosition = angleDegToSteps(0.0f);
      cmd.speedSPS = 120.0f;
      cmd.modeLabel = "SAFE_HOME_TEST";
      break;
  }

  return cmd;
}

void runMotorTestFromPhase(const PhaseOutput& out) {
  MotorTestCommand cmd = buildMotorTestCommand(out);
  moveToSteps(cmd.targetStepPosition, cmd.speedSPS);
}

void setup() {
  pinMode(PRESSURE_PIN, INPUT);

  pinMode(STEP_N_PIN, OUTPUT);
  pinMode(DIR_N_PIN, OUTPUT);
  pinMode(EN_N_PIN, OUTPUT);

  digitalWriteFast(STEP_N_PIN, LOW);
  digitalWriteFast(DIR_N_PIN, LOW);
  digitalWriteFast(EN_N_PIN, DM_OFF);

  analogReadResolution(10);

  Serial.begin(115200);
  delay(300);

  enableDriver(true);

  lastSampleMs = millis();
  lastAngleDeg = 0.0f;

  Serial.println("Bench gait-phase motor test started");
  setup_sensors();
}

void loop() {
  SensorData s = readSensors();
  PhaseOutput out = computePhaseOutput(s);

  runMotorTestFromPhase(out);

  long cs, ts;
  noInterrupts();
  cs = currentSteps;
  ts = targetSteps;
  interrupts();

  MotorTestCommand activeCmd = buildMotorTestCommand(out);

  if (out.phase != lastPhase) {
    Serial.println("---- PHASE CHANGE ----");
    lastPhase = out.phase;
  }

  Serial.print("load=");
  Serial.print(s.footLoadNorm, 3);
  Serial.print(", angle=");
  Serial.print(s.shankAngleDeg, 2);
  Serial.print(", vel=");
  Serial.print(s.shankVelDegPerSec, 2);
  Serial.print(", phase=");
  Serial.print(phaseToString(out.phase));
  Serial.print(", motorMode=");
  Serial.print(activeCmd.modeLabel);
  Serial.print(", targetDeg=");
  Serial.print(out.theoreticalKneeAngleDeg, 1);
  Serial.print(", cmdSteps=");
  Serial.print(activeCmd.targetStepPosition);
  Serial.print(", cmdSPS=");
  Serial.print(activeCmd.speedSPS, 1);
  Serial.print(", currentSteps=");
  Serial.print(cs);
  Serial.print(", targetSteps=");
  Serial.println(ts);

  delay(20);
}