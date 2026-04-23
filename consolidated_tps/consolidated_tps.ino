#include <Arduino.h>
#include <IntervalTimer.h>
#include "ICM_20948.h" // http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "HX711.h"

// ==========================================
// 1. HARDWARE PINS & MOTOR CONFIGURATION
// ==========================================
constexpr uint8_t STEP_N_PIN = 5;
constexpr uint8_t DIR_N_PIN  = 6;
constexpr uint8_t EN_N_PIN   = 7;

constexpr uint8_t DATA_PIN = 2; // HX711 DT
constexpr uint8_t CLOCK_PIN = 3; // HX711 SCK

constexpr uint8_t DM_OFF = HIGH;
constexpr uint8_t DM_ON  = LOW;

IntervalTimer stepTimer;
constexpr uint32_t STEP_PULSE_LOW_US = 5;

#define WIRE_PORT Wire 
#define AD0_VAL 1
ICM_20948_I2C myICM; 
HX711 pressure_sensor; 

// ==========================================
// 2. SIGNAL PROCESSING & FILTERS
// ==========================================
class MovingAverageFilter {
    private:
    float* buffer;
    int windowSize;
    int index = 0;
    float currentSum = 0;

    public:
    MovingAverageFilter(int size) : windowSize(size) {
        buffer = new float[size]{0}; 
    }

    float process(float newValue) {
        currentSum -= buffer[index];       
        buffer[index] = newValue;          
        currentSum += buffer[index];       
        index = (index + 1) % windowSize;  
        return currentSum / windowSize;    
    }
};

MovingAverageFilter weightFilter(10); 
MovingAverageFilter gyroFilter(5); 

float filteredAngle = 0.0;
unsigned long lastTime = 0;

// ==========================================
// 3. GAIT PHASE STATE MACHINE & THRESHOLDS
// ==========================================
enum GaitPhase {
  INITIAL_CONTACT, 
  LOADING_RESPONSE, 
  MID_STANCE,      
  TERMINAL_STANCE, 
  PRE_SWING,       
  INITIAL_SWING,   
  MID_SWING,       
  TERMINAL_SWING   
};

GaitPhase currentPhase = MID_STANCE; 

// Dynamic Variables
float baselineAngle = 0.0;
float userPaceFactor = 1.0; 
float peakSwingSpeed = 0.0; 

// Generic Bench-Testing Constants
// Any object heavier than this noise floor will trigger stance phases
constexpr float NOISE_FLOOR_UNITS = 15.0; 
constexpr float MAX_SWING_ANGLE = 65.0;    
constexpr float BASE_SWING_SPS = 600.0;    

float currentMotorSpeed = 0.0; 
float currentRequiredTorque = 0.0;

// ==========================================
// 4. MOTOR CONTROL FUNCTIONS
// ==========================================
void stepISR() {
  digitalWriteFast(STEP_N_PIN, DM_ON);
  delayMicroseconds(STEP_PULSE_LOW_US);
  digitalWriteFast(STEP_N_PIN, DM_OFF);
}

void setSpeedSPS(float sps) {
  if (sps <= 0) {
    stepTimer.end();
    digitalWriteFast(STEP_N_PIN, DM_OFF);
    return;
  }
  float period_us = 1e6f / sps;
  stepTimer.begin(stepISR, period_us);
}

void enableDriver(bool enable) {
  digitalWriteFast(EN_N_PIN, enable ? DM_ON : DM_OFF);
}

void setDirection(bool forward) {
  digitalWriteFast(DIR_N_PIN, forward ? DM_ON : DM_OFF);
}

// ==========================================
// 5. KNEE ACTUATION LOGIC
// ==========================================
void calculateMotorTarget(GaitPhase phase, float &targetSpeedSPS, float &targetTorque) {
    switch(phase) {
        case INITIAL_CONTACT:
        case LOADING_RESPONSE:
            targetTorque = 80.0; 
            targetSpeedSPS = -100.0 * userPaceFactor; // Yielding slightly
            break;
            
        case MID_STANCE:
            targetTorque = 100.0; // Max support
            targetSpeedSPS = 0.0; // Locked
            break;
            
        case TERMINAL_STANCE:
        case PRE_SWING:
            targetTorque = 90.0; 
            targetSpeedSPS = BASE_SWING_SPS * 0.5 * userPaceFactor; // Begin push-off
            break;
            
        case INITIAL_SWING:
        case MID_SWING:
            targetTorque = 30.0; // Low resistance
            targetSpeedSPS = BASE_SWING_SPS * userPaceFactor; // Dynamic swing speed
            break;
            
        case TERMINAL_SWING:
            targetTorque = 60.0; // Braking
            targetSpeedSPS = -300.0 * userPaceFactor; // Decelerate to extend
            break;
    }
}

void actuateMotor(float speedSPS, float torque) {
    if (torque > 5.0) {
        enableDriver(true);
    } else {
        enableDriver(false);
    }

    if (speedSPS > 0) {
        setDirection(true); 
        setSpeedSPS(speedSPS);
    } else if (speedSPS < 0) {
        setDirection(false); 
        setSpeedSPS(abs(speedSPS)); 
    } else {
        setSpeedSPS(0);
    }
}

// ==========================================
// 6. PHASE DETECTION LOGIC
// ==========================================
void detectPhase(float weight, float angle, float velocity) {
  // Generic Object Detection based on Noise Floor
  bool isWeightBearing = weight > NOISE_FLOOR_UNITS;
  bool isLiftedOff = weight < (NOISE_FLOOR_UNITS * 0.4); // Hysteresis to prevent flickering

  switch (currentPhase) {
    case TERMINAL_SWING:
      if (isWeightBearing) currentPhase = INITIAL_CONTACT;
      break;

    case INITIAL_CONTACT:
      if (abs(angle - baselineAngle) > 2.0) currentPhase = LOADING_RESPONSE;
      break;

    case LOADING_RESPONSE:
      if (velocity < 5.0 && isWeightBearing) currentPhase = MID_STANCE;
      break;

    case MID_STANCE:
      if ((angle - baselineAngle) < -5.0) currentPhase = TERMINAL_STANCE;
      break;

    case TERMINAL_STANCE:
      if (isLiftedOff) {
          currentPhase = PRE_SWING;
          peakSwingSpeed = 0.0; // Reset for new swing
      } 
      break;

    case PRE_SWING:
      if (velocity > 30.0) currentPhase = INITIAL_SWING;
      break;

    case INITIAL_SWING:
      if (velocity > peakSwingSpeed) peakSwingSpeed = velocity;
      
      // Adapt user pace multiplier based on max velocity
      userPaceFactor = constrain(peakSwingSpeed / 100.0, 0.5, 1.5);
      
      if ((angle - baselineAngle) > (MAX_SWING_ANGLE * 0.8)) currentPhase = MID_SWING;
      break;

    case MID_SWING:
      if (velocity < 10.0) currentPhase = TERMINAL_SWING; 
      break;
  }
}

// ==========================================
// 7. SENSOR CALIBRATION
// ==========================================
void calibrateSensors() {
  Serial.println("--- CALIBRATING ---");
  Serial.println("DO NOT TOUCH SENSORS. KEEP PLATE EMPTY. KEEP IMU STILL.");
  delay(2000); 

  float tempAngle = 0;
  int samples = 50;

  // Initialize HX711 with your requested settings
  pressure_sensor.begin(DATA_PIN, CLOCK_PIN);
  
  // pressure_sensor.set_offset(8235729); // Commented out: tare() overrides this!
  pressure_sensor.set_scale(-12.48);
  
  // Tare zeros out the weight of the empty plate
  pressure_sensor.tare(); 
  delay(500);

  for (int i = 0; i < samples; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      float rawPitch = atan2(myICM.accX(), myICM.accZ()) * 57.2958;
      tempAngle += rawPitch; 
      delay(20);
    }
  }

  baselineAngle = tempAngle / samples;
  filteredAngle = baselineAngle; 
  
  Serial.println("--- CALIBRATION COMPLETE ---");
  Serial.print("Baseline Angle: "); Serial.println(baselineAngle);
  
  lastTime = millis(); 
  delay(1000);
}

// ==========================================
// 8. SYSTEM SETUP
// ==========================================
void setup_sensors() {
  Serial.begin(115200);

  pinMode(STEP_N_PIN, OUTPUT);
  pinMode(DIR_N_PIN, OUTPUT);
  pinMode(EN_N_PIN, OUTPUT);

  digitalWriteFast(STEP_N_PIN, DM_OFF);
  digitalWriteFast(DIR_N_PIN, DM_OFF);
  digitalWriteFast(EN_N_PIN, DM_OFF);
  
  enableDriver(true);
  
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {
      myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      delay(500);
    } else {
      initialized = true;
    }
  }

  calibrateSensors();
}

// ==========================================
// 9. MAIN LOOP
// ==========================================
String read_sensors() {
  if (myICM.dataReady() && pressure_sensor.is_ready()) {
    myICM.getAGMT(); 
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; 
    lastTime = currentTime;

    float accPitch = atan2(myICM.accX(), myICM.accZ()) * 57.2958;
    float currentGyroY = gyroFilter.process(myICM.gyrY()); 
    filteredAngle = 0.96 * (filteredAngle + currentGyroY * dt) + 0.04 * accPitch;
    
    float rawWeight = pressure_sensor.get_units(1);
    float smoothWeight = weightFilter.process(rawWeight);

    detectPhase(smoothWeight, filteredAngle, currentGyroY);
    calculateMotorTarget(currentPhase, currentMotorSpeed, currentRequiredTorque);
    actuateMotor(currentMotorSpeed, currentRequiredTorque);

    return String(smoothWeight)+","+String(currentPhase * 10)+","+String(filteredAngle)+","+String(currentMotorSpeed);
    
  } else {
    delay(2); 
    return null;
  }
}
