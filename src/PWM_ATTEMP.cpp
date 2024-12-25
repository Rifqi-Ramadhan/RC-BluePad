// Defining Pin Used for Servo Tilt - Pan
#define Servo_Pin_1 33 // Set as PWM
#define Servo_Pin_2 32 // Set as PWM

#define PWM_FREQ 5000          // Frequency of PWM in Hz
#define PWM_RESOLUTION 8       // Resolution (8 bits = 0-255 duty cycle)
#define PWM_CHANNEL_1 0        // LEDC channel for Servo_Pin_1
#define PWM_CHANNEL_2 1        // LEDC channel for Servo_Pin_2

void setup() {
  Serial.begin(9600);
  // Attach pins to LEDC PWM channels with frequency and resolution
  if (!ledcAttachChannel(Servo_Pin_1, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_1)) {
    Serial.println("Failed to configure PWM for Servo_Pin_1!");
  }
  if (!ledcAttachChannel(Servo_Pin_2, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_2)) {
    Serial.println("Failed to configure PWM for Servo_Pin_2!");
  }
  Serial.println("PWM setup complete");
}

void loop() {
  // Cycle PWM duty cycle on Servo_Pin_1 and Servo_Pin_2
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    ledcWrite(Servo_Pin_1, dutyCycle); // Gradually increase PWM on pin 33
    ledcWrite(Servo_Pin_2, dutyCycle); // Gradually increase PWM on pin 32
    delay(2);
  }
  for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
    ledcWrite(Servo_Pin_1, dutyCycle); // Gradually decrease PWM on pin 33
    ledcWrite(Servo_Pin_2, dutyCycle); // Gradually decrease PWM on pin 32
    delay(2);
  }
  delay(200);
}
