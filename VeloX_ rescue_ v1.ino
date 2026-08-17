#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// ==================== PIN DEFINITIONS ====================
const int BUZZER_PIN = 2;
const int GREEN_LED   = 4;
const int RED_LED     = 10;

// ESC / Motor PWM Pins (Quad-X layout)
// M1 = front-right, M2 = rear-right, M3 = rear-left, M4 = front-left
const int MOTOR_1_PIN = 3;
const int MOTOR_2_PIN = 5;
const int MOTOR_3_PIN = 6;
const int MOTOR_4_PIN = 9;

// NRF24L01 CE & CSN Pins
const int NRF_CE_PIN  = 7;
const int NRF_CSN_PIN = 8;

// ==================== FRAME GEOMETRY (Iter. 09) ====================
// Recorded here so the safety margin is visible in firmware, not just CAD.
const float WHEELBASE_MM         = 220.0;  // motor-center to motor-center
const float BUMPER_DIAMETER_MM   = 139.0;  // outer protective duct/bumper ring
const float TOTAL_FOOTPRINT_MM   = WHEELBASE_MM + BUMPER_DIAMETER_MM; // 359mm tip-to-tip
const float USAR_BREACH_HOLE_MM  = 450.0;  // standard USAR concrete breach diameter
const float CLEARANCE_MARGIN_MM  = USAR_BREACH_HOLE_MM - TOTAL_FOOTPRINT_MM; // 91mm

// ==================== CONTROL TUNING ====================
const int STICK_CENTER        = 512;   // glove joystick/gesture center value
const int STICK_RANGE         = 512;   // +/- range around center
const int MAX_ATTITUDE_OFFSET = 300;   // max µs the mixer can add/subtract per axis
const float KP_STABILIZE      = 4.0;   // proportional self-level gain (µs per degree error) — tune in bench test
const float MAX_TILT_DEG      = 30.0;  // max commanded lean angle from stick input

// ==================== FAILSAFE TUNING ====================
const unsigned long SIGNAL_TIMEOUT   = 800;   // ms — was 3000ms, tightened for tight-rubble ops
const unsigned long DESCENT_RAMP_MS  = 2000;  // ms to ramp from last throttle down to floor
const int DESCENT_FLOOR_PWM          = 1100;  // gentle-descent PWM floor — tune to your thrust/weight

// ==================== OBJECTS & VARIABLES ====================
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Adafruit_MPU6050 mpu;
Servo motor1, motor2, motor3, motor4;

const byte rxAddress[6] = "VLOX1";

struct ControlPacket {
  int throttle; // 0 - 1024
  int pitch;    // 0 - 1024, center = 512
  int roll;     // 0 - 1024, center = 512
  int yaw;      // 0 - 1024, center = 512
};

ControlPacket receivedData;
unsigned long lastHeartbeat = 0;

// Attitude estimation state (complementary filter)
float pitchAngle = 0.0;
float rollAngle = 0.0;
unsigned long lastFilterTimeUs = 0;

// Failsafe ramp state
bool failsafeActive = false;
unsigned long failsafeStartTime = 0;
int throttleAtFailsafeStart = 1000;
int lastCommandedThrottlePWM = 1000;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  motor1.attach(MOTOR_1_PIN, 1000, 2000);
  motor2.attach(MOTOR_2_PIN, 1000, 2000);
  motor3.attach(MOTOR_3_PIN, 1000, 2000);
  motor4.attach(MOTOR_4_PIN, 1000, 2000);

  stopMotors(); // Initial Motor Safety Lock

  if (!mpu.begin()) {
    Serial.println(F("[ERROR] MPU6050 Sensor Fault!"));
    while (1) {
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
      delay(100);
    }
  }

  if (radio.begin()) {
    radio.openReadingPipe(0, rxAddress);
    radio.setPALevel(RF24_PA_MAX);
    radio.startListening();
    Serial.println(F("[RF LINK] NRF24L01 Receiver Online!"));
  } else {
    Serial.println(F("[ERROR] NRF24L01 Hardware Not Found!"));
  }

  lastFilterTimeUs = micros();

  Serial.println(F("=============================================="));
  Serial.println(F("  VeloX-Rescue V1 - Firmware Fully Initialized "));
  Serial.print(F("  Frame footprint: "));
  Serial.print(TOTAL_FOOTPRINT_MM);
  Serial.print(F("mm | USAR clearance margin: "));
  Serial.print(CLEARANCE_MARGIN_MM);
  Serial.println(F("mm"));
  Serial.println(F("=============================================="));
}

// ==================== MAIN LOOP ====================
void loop() {
    // 1. Check Wireless Packet from Gesture Glove
    if (radio.available()) {
        radio.read(&receivedData, sizeof(ControlPacket));
        lastHeartbeat = millis();
    }

    // 2. Serial Monitor Simulation Fallback (bench testing only - remove before field flights)
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            lastHeartbeat = millis();
        }
        receivedData.throttle = 500; 
        receivedData.pitch = STICK_CENTER;
        receivedData.roll = STICK_CENTER;
        receivedData.yaw = STICK_CENTER;
    } else if (receivedData.throttle == 0) {
        lastHeartbeat = 0; // Trigger Instant Timeout if throttle is cleared
    }

    // 3. Read MPU6050 Telemetry & Process Orientation Filter
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    updateAttitude(a, g);

    // 4. Check Link Timeout for Emergency Failsafe
    if (millis() - lastHeartbeat > SIGNAL_TIMEOUT) {
        triggerFailsafeMode();
    } else {
        failsafeActive = false; // link is good, clear any prior failsafe ramp state
        executeFlightControl(receivedData.throttle, receivedData.pitch, receivedData.roll, receivedData.yaw);
    }

    // 5. Precise 50 Hz Control Loop Sync (Replaces unsafe delay command)
    while (micros() - lastFilterTimeUs < 20000) {
        // Strict deterministic wait window to ensure exactly 20ms cycles
    }
}


// ==================== ATTITUDE ESTIMATION ====================
void updateAttitude(sensors_event_t &a, sensors_event_t &g) {
    unsigned long nowUs = micros();
    float dt = (nowUs - lastFilterTimeUs) / 1000000.0;
    lastFilterTimeUs = nowUs; // Lock timestamp immediately for accurate integration

    if (dt <= 0 || dt > 0.5) return; // guard against first-run or overflow glitches

    // Calculate Pitch and Roll from Accelerometer data
    float accelPitch = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
    float accelRoll  = atan2(-a.acceleration.x, a.acceleration.z) * 180.0 / PI;

    // Complementary Filter: High-pass Gyro (converted to deg/s) + Low-pass Accelerometer
    pitchAngle = 0.98 * (pitchAngle + (g.gyro.x * 180.0 / PI) * dt) + 0.02 * accelPitch;
    rollAngle  = 0.98 * (rollAngle  + (g.gyro.y * 180.0 / PI) * dt) + 0.02 * accelRoll;
}


// ==================== FLIGHT CONTROL / MOTOR MIXER ====================
void executeFlightControl(int baseThrottle, int pitchRaw, int rollRaw, int yawRaw) {
  noTone(BUZZER_PIN);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, LOW);

  int throttlePWM = map(baseThrottle, 0, 1024, 1000, 2000);
  lastCommandedThrottlePWM = throttlePWM;

  // Pilot's commanded lean angle from stick deflection
  float desiredPitch = map(pitchRaw, 0, 1024, -MAX_TILT_DEG * 100, MAX_TILT_DEG * 100) / 100.0;
  float desiredRoll  = map(rollRaw,  0, 1024, -MAX_TILT_DEG * 100, MAX_TILT_DEG * 100) / 100.0;

  // Proportional self-leveling: correction pushes actual angle toward desired angle
  int stabPitch = constrain((int)((desiredPitch - pitchAngle) * KP_STABILIZE), -MAX_ATTITUDE_OFFSET, MAX_ATTITUDE_OFFSET);
  int stabRoll  = constrain((int)((desiredRoll  - rollAngle)  * KP_STABILIZE), -MAX_ATTITUDE_OFFSET, MAX_ATTITUDE_OFFSET);

  int yawOffset = map(yawRaw - STICK_CENTER, -STICK_RANGE, STICK_RANGE, -MAX_ATTITUDE_OFFSET, MAX_ATTITUDE_OFFSET);

  mixAndDrive(throttlePWM, stabPitch, stabRoll, yawOffset);

  Serial.print(F("[FLIGHT] Thr:"));
  Serial.print(throttlePWM);
  Serial.print(F(" Pitch:"));
  Serial.print(pitchAngle);
  Serial.print(F(" Roll:"));
  Serial.println(rollAngle);
}

// Quad-X mixer: M1=FR, M2=RR, M3=RL, M4=FL
void mixAndDrive(int throttlePWM, int P, int R, int Y) {
  int m1 = constrain(throttlePWM + P + R + Y, 1000, 2000);
  int m2 = constrain(throttlePWM - P + R - Y, 1000, 2000);
  int m3 = constrain(throttlePWM - P - R + Y, 1000, 2000);
  int m4 = constrain(throttlePWM + P - R - Y, 1000, 2000);

  motor1.writeMicroseconds(m1);
  motor2.writeMicroseconds(m2);
  motor3.writeMicroseconds(m3);
  motor4.writeMicroseconds(m4);
}

// ==================== FAILSAFE ====================
void triggerFailsafeMode() {
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  tone(BUZZER_PIN, 1000); // acoustic locator beacon — helps find the unit under rubble

  if (!failsafeActive) {
    failsafeActive = true;
    failsafeStartTime = millis();
    throttleAtFailsafeStart = lastCommandedThrottlePWM;
  }

  unsigned long elapsed = millis() - failsafeStartTime;
  float rampFrac = min(1.0f, (float)elapsed / (float)DESCENT_RAMP_MS);
  int descentPWM = throttleAtFailsafeStart -
                    (int)(rampFrac * (throttleAtFailsafeStart - DESCENT_FLOOR_PWM));
  descentPWM = constrain(descentPWM, DESCENT_FLOOR_PWM, 2000);

  // Level attitude (P=R=Y=0) during descent — no pilot input to trust, so fly level
  mixAndDrive(descentPWM, 0, 0, 0);

  Serial.print(F("[FAILSAFE] Ramped descent PWM: "));
  Serial.println(descentPWM);
}

void stopMotors() {
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
}