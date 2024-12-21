#include <Arduino.h>


//Defining Pin Used for FrontWheel - Motor A
#define FrontWheel_EN_A 4 // Set this asw PWM
#define FrontWheel_INT1_A 2
#define FrontWheel_INT2_A 15
//Defining Pin Used for FrontWheel - Motor B
#define FrontWheel_EN_B 12 // Set this as PWM
#define FrontWheel_INT1_B 13
#define FrontWheel_INT2_B 14

//Defining Variables for PWM Action
const int pwmChannel = 0;      // PWM Channel
const int pwmFrequency = 5000; // Frequency in Hz
const int pwmResolution = 8;   // Resolution: 8 bits (0-255)

int MotorSpeed = 255;

void Function_SetSpeed(){
  if (Serial.available() > 0){
    Serial.println("Enter The Percentage for the Speed :");
    String Speed_Input = Serial.readStringUntil('\n');
    int MotorSpeed_Input = Speed_Input.toInt();
    int MotorSpeed_PercentToInt = map(MotorSpeed_Input, 0, 100, 0, 255);

    Serial.println("Inputted Speed Percentage is : " );
    Serial.println(MotorSpeed_Input);
    Serial.println("%");
    if(MotorSpeed_Input <= 100){
      MotorSpeed = MotorSpeed_PercentToInt;
      ledcWrite(pwmChannel, MotorSpeed);
    }
  }
}

void Function_SetAllHigh(){
    digitalWrite(FrontWheel_INT1_A, HIGH);
    digitalWrite(FrontWheel_INT2_A, LOW);
    digitalWrite(FrontWheel_INT1_B, HIGH);
    digitalWrite(FrontWheel_INT2_B, LOW);
}

void setup() {
  
  Serial.begin(9600);
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(FrontWheel_EN_A, pwmChannel);
  ledcAttachPin(FrontWheel_EN_B, pwmChannel);
  ledcWrite(pwmChannel, MotorSpeed);

  pinMode(FrontWheel_INT1_A, OUTPUT);
  pinMode(FrontWheel_INT2_A, OUTPUT);
  pinMode(FrontWheel_INT1_B, OUTPUT);
  pinMode(FrontWheel_INT2_B, OUTPUT);

  Function_SetAllHigh();
}

void loop() {
  Function_SetSpeed();
  delay(100);
}

// put function definitions here:
