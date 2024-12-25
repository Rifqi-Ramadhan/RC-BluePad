#include <Arduino.h> // Library for the writing Pin
#include <Bluepad32.h> // Library for the Gamepad Controller handler

// Defining the ControllerPtr to refrence an array to the myControllers
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Defining Pin Used for FrontWheel - Motor A
#define FrontWheel_EN_A 4 // Set this as PWM
#define FrontWheel_INT1_A 2
#define FrontWheel_INT2_A 15
// Defining Pin Used for FrontWheel - Motor B
#define FrontWheel_EN_B 12 // Set this as PWM
#define FrontWheel_INT1_B 13
#define FrontWheel_INT2_B 14
// Defining Pin Used for BackWheel - Motor A
#define BackWheel_EN_A 5 // Set this as PWM
#define BackWheel_INT1_A 22
#define BackWheel_INT2_A 23
// Defining Pin Used for BackWheel - Motor B
#define BackWheel_EN_B 27 // Set this as PWM
#define BackWheel_INT1_B 26
#define BackWheel_INT2_B 25
// Defining Pin Used for WaterPump - Motor A
#define MotorPump_EN_A 18 // Set this as PWM
#define MotorPump_INT1_A 19
#define MotorPump_INT2_A 21
// Defining Pin Used for Servo Tilt - Pan
#define Servo_Pin_1 33 // Set As PWM
#define Servo_Pin_2 32 // Set As PWM

// Defining Variables for PWM Action
const int pwmFrequency = 5000; // Frequency in Hz
const int pwmResolution = 8;   // Resolution: 8 bits (0-255)

// set the MotorSpeed as 255 at the beginning
int MotorSpeed = 255;

// array to save the motor pins, to be done in for loop
const int motorPins[] = {
    FrontWheel_INT1_A,
    FrontWheel_INT2_A,
    FrontWheel_INT1_B,
    FrontWheel_INT2_B,
    BackWheel_INT1_A,
    BackWheel_INT2_A,
    BackWheel_INT1_B,
    BackWheel_INT2_B,
    MotorPump_EN_A,
    MotorPump_INT1_A,
    MotorPump_INT2_A
};

// Assign unique channels for each PWM pin
const int pwmChannels[] = {0, 1, 2, 3};

// callback for Connected Controller
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false; // Boolean for checking the empty slot
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) { // Looping for all the array saving controller
        if (myControllers[i] == nullptr) { // check if the index array is null, if yes
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i); // then show the index of the controller
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties(); // get the properties of the controller, this is from the library
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id); // print for the data connected
            myControllers[i] = ctl; // set the controller pointer to be ctl from the functions arguments
            foundEmptySlot = true; // make the empty slot to true, indicates that it was found
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot"); // error handling message, controller found but array to save is full
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false; // change the found ciontroller false

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i); // Serial print for disconnect message
            myControllers[i] = nullptr; // make the array that save the controller to null
            foundController = true; // change boolean to true
            break; // break it 
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers"); // error handling when a new controller is found
    }
}

void dumpGamepad(ControllerPtr ctl) { // Dump the Info
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S, Stadia support rumble.
        // It is possible to set it by calling:
        // Some controllers have two motors: "strong motor", "weak motor".
        // It is possible to control them independently.
        ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */,
                            0x40 /* strongMagnitude */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);
}

void processControllers() { // Function to check what type of controller
    for (auto myController : myControllers) {  // auto here be used to automatically determine the myControllers variables
        if (myController && myController->isConnected() && myController->hasData()) { // Checker if the controiller is connected and if it has a data
            if (myController->isGamepad()) { // Check if the controller is a gamepad
                processGamepad(myController); // Run the MyController Function
            } else {
                Serial.println("Unsupported controller"); // Only supports controller :)
            }
        }
    }
}

void Function_SetSpeed() {
  if (Serial.available() > 0) {
    Serial.println("Enter The Percentage for the Speed :");
    String Speed_Input = Serial.readStringUntil('\n');
    Speed_Input.trim(); // Remove any trailing newlines or spaces
    int MotorSpeed_Input = Speed_Input.toInt(); // Input a percentage
    int MotorSpeed_PercentToInt = map(MotorSpeed_Input, 0, 100, 0, 255); // Map for converting percentage to a 8 bit data
    // Prompt for input
    Serial.print("Inputted Speed Percentage is: ");
    Serial.print(MotorSpeed_Input);
    Serial.println("%");

    if (MotorSpeed_Input >= 0 && MotorSpeed_Input <= 100) { // Checker for input speed
      MotorSpeed = MotorSpeed_PercentToInt; // Set MotorSpeed with motorSpeed input
      for (int i = 0; i < 4; i++) { // Write data to all of the pwm channel
        ledcWrite(pwmChannels[i], MotorSpeed); // ledc Write to the pwm channels
      }
    } else {
      Serial.println("Invalid speed percentage. Please enter a value between 0 and 100."); // Failure Prompt
    }
  }
}

void Function_SetAllHigh() {
  // Setup Pin For FrontWheels
  digitalWrite(FrontWheel_INT1_A, HIGH); // Set this as High to Go Forward
  digitalWrite(FrontWheel_INT2_A, LOW);  // Set this as High to Go Backward
  digitalWrite(FrontWheel_INT1_B, LOW);  // Set this as High to Go Backward
  digitalWrite(FrontWheel_INT2_B, HIGH); // Set this as High to Go Forward
  // Setup Pin For BackWheels
  digitalWrite(BackWheel_INT1_A, HIGH);  // Set this as High to Go Forward
  digitalWrite(BackWheel_INT2_A, LOW);   // Set this as High to Go Backward
  digitalWrite(BackWheel_INT1_B, LOW);   // Set this as High to Go Backward
  digitalWrite(BackWheel_INT2_B, HIGH);  // Set this as High to go Forward
}

void setup() {
  //begins serial at baud rate 9600
  Serial.begin(9600);
  // COunt the motor pins array length
  int count_length = sizeof(motorPins)/sizeof(motorPins[0]);

  // Setup PWM for each motor
  ledcSetup(pwmChannels[0], pwmFrequency, pwmResolution);
  ledcSetup(pwmChannels[1], pwmFrequency, pwmResolution);
  ledcSetup(pwmChannels[2], pwmFrequency, pwmResolution);
  ledcSetup(pwmChannels[3], pwmFrequency, pwmResolution);

  // Define which pin be used for PWM
  ledcAttachPin(FrontWheel_EN_A, pwmChannels[0]);
  ledcAttachPin(FrontWheel_EN_B, pwmChannels[1]);
  ledcAttachPin(BackWheel_EN_A, pwmChannels[2]);
  ledcAttachPin(BackWheel_EN_B, pwmChannels[3]);

  // Set initial speed
  for (int i = 0; i < 4; i++) {
    ledcWrite(pwmChannels[i], MotorSpeed);
  }

  Serial.println("Starting setup");
  for (int i = 0; i < count_length; i++) {
    pinMode(motorPins[i], OUTPUT);
    Serial.print("Initialized pin: ");
    Serial.println(motorPins[i]);
    delay(100);
  }
  Serial.println("Setup complete");

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  Function_SetAllHigh();
  Serial.println("Pin & Speed Initialization Succeed");

  BP32.setup(&onConnectedController, &onDisconnectedController);
  //BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

}

void loop() {
  bool dataUpdated = BP32.update();
  // Find a way to be able to input speed in here
    if (dataUpdated)
        processControllers();
        Function_SetSpeed(); // Maybe this is the way, i don't know
  delay(100);
}
