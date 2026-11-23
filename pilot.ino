// --- Controller Parameters ---
float Kp = 2.0;
float Ki = 0.01;
float Kd = 3.0;
float integralError = 0.0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(5, OUTPUT); // Sends Control Signal (-100%..+100%) to Left Arduino
  lastTime = millis();
}

float computeController(float targetAngle, float measuredAngle, float measuredSpeed, float dt) {
  float error = targetAngle - measuredAngle;
  integralError = constrain(integralError + error * dt, -100.0, 100.0);
  
  float rawOutput = (Kp * error) + (Ki * integralError) - (Kd * measuredSpeed);
  return constrain(rawOutput, -100.0, 100.0);
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  if (dt >= 0.02) { // 50 Hz loop
    lastTime = now;

    // 1. Read Pilot Knob (A0) -> Target Angle (-90 to +90)
    float targetAngle = ((analogRead(A0) / 1024.0) * 180.0) - 90.0;

    // 2. Read Machine Angle from Left Arduino (A1) -> Measured Angle (-90 to +90)
    float maxAngle = 90;
    float measuredAngle = ((analogRead(A1) / 1024.0) * (2 * maxAngle)) - maxAngle;

    // 3. Read Machine Speed from Left Arduino (A1) -> Measured Speed (-187.5 to +187.5)
    float maxSpeed = 187.5;
    float measuredSpeed = ((analogRead(A2) / 1024.0) * (2 * maxSpeed)) - maxSpeed;
	
    // 4. Compute PID Control Signal (-100% to +100%)
    float controlSignal = computeController(targetAngle, measuredAngle, measuredSpeed, dt);

    // 5. Send Control Signal out as PWM (0..255) on Pin 5
    int pwmOut = (int)(((controlSignal + 100.0) / 200.0) * 255.0);
    analogWrite(5, constrain(pwmOut, 0, 255));

    // 6. Print numbers only for Tinkercad Serial Plotter (Target, Control, Angle)
    Serial.print(targetAngle);
    Serial.print(",");
    Serial.print(controlSignal);
    Serial.print(",");
    Serial.print(measuredAngle);
    Serial.print(",");
    Serial.println(measuredSpeed);
  }
}
