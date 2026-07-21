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

// ESC / Motor PWM Pins
const int MOTOR_1_PIN = 3;
const int MOTOR_2_PIN = 5;
const int MOTOR_3_PIN = 6;
const int MOTOR_4_PIN = 9;

// NRF24L01 CE & CSN Pins
const int NRF_CE_PIN  = 7;
const int NRF_CSN_PIN = 8;

// ==================== OBJECTS & VARIABLES ====================
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Adafruit_MPU6050 mpu;

// ESC Servo Objects
Servo motor1, motor2, motor3, motor4;

const byte rxAddress[6] = "VLOX1"; // Dynamic RF Pipe Address

// Incoming Radio Data Structure from Glove Transmitter
struct ControlPacket {
  int throttle; // 0 - 1024
  int pitch;    // Angle / Joystick
  int roll;     // Angle / Joystick
  int yaw;      // Rotation
};

ControlPacket receivedData;
unsigned long lastHeartbeat = 0;
const unsigned long SIGNAL_TIMEOUT = 3000; // 3 Seconds Failsafe Timeout

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Attach Motors to PWM Pins
  motor1.attach(MOTOR_1_PIN, 1000, 2000);
  motor2.attach(MOTOR_2_PIN, 1000, 2000);
  motor3.attach(MOTOR_3_PIN, 1000, 2000);
  motor4.attach(MOTOR_4_PIN, 1000, 2000);

  stopMotors(); // Initial Motor Safety Lock

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println(F("[ERROR] MPU6050 Sensor Fault!"));
    while (1) {
      digitalWrite(RED_LED, HIGH);
      delay(100);
      digitalWrite(RED_LED, LOW);
      delay(100);
    }
  }

  // Initialize NRF24L01 Transceiver
  if (radio.begin()) {
    radio.openReadingPipe(0, rxAddress);
    radio.setPALevel(RF24_PA_MAX);
    radio.startListening();
    Serial.println(F("[RF LINK] NRF24L01 Receiver Online!"));
  } else {
    Serial.println(F("[ERROR] NRF24L01 Hardware Not Found!"));
  }

  Serial.println(F("=============================================="));
  Serial.println(F("  VeloX-Rescue V1 - Firmware Fully Initialized "));
  Serial.println(F("=============================================="));
}

// ==================== MAIN LOOP ====================
void loop() {
  // Check Wireless Packet from Gesture Glove
  if (radio.available()) {
    radio.read(&receivedData, sizeof(ControlPacket));
    lastHeartbeat = millis();
  }

  // Serial Monitor Simulation Fallback
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      lastHeartbeat = millis();
      receivedData.throttle = 500; // Simulated Idle Hover Throttle
    } else if (cmd == 'd' || cmd == 'D') {
      lastHeartbeat = 0; // Trigger Instant Timeout
    }
  }

  // Read MPU6050 Telemetry
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Check Link Timeout for Emergency Failsafe
  if (millis() - lastHeartbeat > SIGNAL_TIMEOUT) {
    triggerFailsafeMode(a.acceleration.x, a.acceleration.y);
  } else {
    executeFlightControl(receivedData.throttle, a.acceleration.x, a.acceleration.y);
  }

  delay(20); // 50 Hz Control Loop Sync
}

// ==================== HELPER FUNCTIONS ====================

void executeFlightControl(int baseThrottle, float ax, float ay) {
  noTone(BUZZER_PIN);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, LOW);

  // Map 0-1024 input to PWM Servo range (1000us to 2000us)
  int motorSpeed = map(baseThrottle, 0, 1024, 1000, 2000);
  
  motor1.writeMicroseconds(motorSpeed);
  motor2.writeMicroseconds(motorSpeed);
  motor3.writeMicroseconds(motorSpeed);
  motor4.writeMicroseconds(motorSpeed);

  Serial.print(F("[FLIGHT RUNNING] Throttle: "));
  Serial.print(motorSpeed);
  Serial.print(F(" | Tilt X: "));
  Serial.println(ax);
}

void triggerFailsafeMode(float ax, float ay) {
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  tone(BUZZER_PIN, 1000); // Acoustic Warning

  // Emergency Hover / Gradual Descent Throttle Safety Limit (1150us)
  motor1.writeMicroseconds(1150);
  motor2.writeMicroseconds(1150);
  motor3.writeMicroseconds(1150);
  motor4.writeMicroseconds(1150);

  Serial.print(F("[FAILSAFE ENGAGED] Emergency Descent Active | Angle: "));
  Serial.println(ax);
}

void stopMotors() {
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
}
