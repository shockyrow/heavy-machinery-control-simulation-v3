#include <Servo.h>

struct PlantState {
  float angle;
  float speed;
};

// --- Heavy Machinery Parameters ---
float maxMotorTorque = 150.0;
float inertia = 5.0;
float friction = 0.8;
float maxSpeed = maxMotorTorque / friction;

PlantState machine = {0.0, 0.0};
unsigned long lastTime = 0;
Servo myServo;

void setup() {
  pinMode(5, OUTPUT); // Sends Angle Feedback (-90..+90) to Right Arduino
  pinMode(6, OUTPUT); // Sends Speed Feedback (-187.5..+187.5) to Right Arduino
  myServo.attach(9, 500, 2500);
  lastTime = millis();
}

struct PlantState simulatePlant(float controlSignal, float dt) {
  float appliedTorque = (controlSignal / 100.0) * maxMotorTorque;
  float dragTorque = friction * machine.speed;

  float acceleration = (appliedTorque - dragTorque) / inertia;
  machine.speed += acceleration * dt;
  machine.angle += machine.speed * dt;

  if (machine.angle > 90.0) {
    machine.angle = 90.0;
    machine.speed = 0.0;
  } else if (machine.angle < -90.0) {
    machine.angle = -90.0;
    machine.speed = 0.0;
  }

  return machine;
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  if (dt >= 0.02) { // 50 Hz loop
    lastTime = now;

    // 1. Read Control Signal from Right Arduino (A0) -> -100% to +100%
    float controlSignal = ((analogRead(A0) / 1024.0) * 200.0) - 100.0;

    // Small deadband so electrical noise doesn't drift the machine at rest
    if (abs(controlSignal) < 1.5) {
      //controlSignal = 0.0;
    }

    // 2. Run Physics Simulation
    machine = simulatePlant(controlSignal, dt);

    // 3. Move Servo visually
    myServo.write(map((int)machine.angle, -90, 90, 0, 180));

    // 4. Send Angle Feedback out as PWM (0..255) on Pin 5
    int pwmAngleFeedback = (int)(((machine.angle + 90.0) / 180.0) * 255.0);
    analogWrite(5, constrain(pwmAngleFeedback, 0, 255));
    
    // 5. Send Speed Feedback out as PWM (0..255) on Pin 6
    int pwmSpeedFeedback = (int)(((machine.speed + maxSpeed) / (2 * maxSpeed)) * 255.0);
    analogWrite(6, constrain(pwmSpeedFeedback, 0, 255));
  }
}
