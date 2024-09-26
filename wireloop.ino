#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define dirPin 8       // Control direction (high = forward) / (low = backward)
#define stepPin 9      // Pulse pin to drive motor
#define enablePin 10   // (low = on) / (high = off)
#define homeSwitch 7   // Home position pin (pull-up)j
#define endSwitch 2    // End position pin (pull-up)
#define startButton 4  // Start button
#define touchPin 3     // Touch sensor pin
#define signalPIN 6    // Signal for speaker

uint16_t touchCount = 0;              // Number of touches counted
const int stepDelay = 500;            // Adjust to increase/decrease motor speed (in microseconds)
volatile bool touchDetected = false;  // Interrupt flag for touch detection
volatile bool interruptOff = false;
volatile bool gameRunning = false;  // Game state flag (True when motor is moving to end position)
bool motorEnabled = false;          // Motor enabled state
bool goingHome = false;             // Flag for motor returning to home
unsigned long currentTime = 0;
unsigned long lastTouchTime = 0;         // Last touch timestamp
const unsigned long debounceTime = 400;  // Debounce interval for touch detection (400 ms)

LiquidCrystal_I2C lcd(0x3f, 16, 2);  // Declare i2c LCD

void setup() {
  Serial.begin(115200);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(homeSwitch, INPUT_PULLUP);
  pinMode(endSwitch, INPUT_PULLUP);
  pinMode(touchPin, INPUT_PULLUP);
  pinMode(startButton, INPUT_PULLUP);
  pinMode(signalPIN, INPUT);
  lcd.begin();
  lcd.backlight();
  // Move the motor to home position initially
  moveToHome();
  // Show the initial status on the LCD
  updateLCD("Stopped", touchCount);
}

void loop() {
  if (digitalRead(startButton) == LOW) {
    Serial.println("Start Button Detected!");
    if (gameRunning) {
      // If the game is running and start button is pressed, stop the game and go back home
      Serial.println("Stopping game");
      gameRunning = false;
      moveToHome();
    } else {
      // Start the game
      Serial.println("Starting game");
      delay(1000);
      gameRunning = true;
      touchCount = 0;  // Reset the touch count
      updateLCD("Running", touchCount);
      moveToEnd();  // Move towards the end position
    }
  }
}

// Function to handle touch sensor interrupts
void handleTouch() {
  if (gameRunning) {  // Only detect touch when the game is running
    touchDetected = true;
    detachInterrupt(digitalPinToInterrupt(touchPin));
  }
}

// Function to move the motor towards the home position
void moveToHome() {
  Serial.println("Going back to home position");
  updateLCD("Stopped", touchCount);
  digitalWrite(enablePin, LOW);  // Enable motor
  digitalWrite(dirPin, LOW);     // Set direction to backward (towards home)

  while (digitalRead(homeSwitch) == HIGH) {
    // Drive the motor until homeSwitch is pressed (LOW)
    stepMotor();
  }

  digitalWrite(enablePin, HIGH);  // Disable motor when home is reached
}

// Function to move the motor towards the end position
void moveToEnd() {
  digitalWrite(enablePin, LOW);  // Enable motor
  digitalWrite(dirPin, HIGH);    // Set direction to forward (towards end)
  attachInterrupt(digitalPinToInterrupt(touchPin), handleTouch, LOW);


  while (gameRunning && digitalRead(endSwitch) == HIGH) {

    if (interruptOff) {
      currentTime = millis();
      if (currentTime - lastTouchTime >= debounceTime) {
        interruptOff = false;
        attachInterrupt(digitalPinToInterrupt(touchPin), handleTouch, LOW);
      }
    }
    if (touchDetected && gameRunning) {
      touchCount++;
      updateValue(touchCount);
      lastTouchTime = millis();
      interruptOff = true;
      //detachInterrupt(digitalPinToInterrupt(touchPin));
      touchDetected = false;
    }
    stepMotor();

    // Check for start button press to stop the game
    if (digitalRead(startButton) == LOW) {
      detachInterrupt(digitalPinToInterrupt(touchPin));

      gameRunning = false;
      updateLCD("Stopped", touchCount);
      moveToHome();
      break;
    }
  }
  detachInterrupt(digitalPinToInterrupt(touchPin));
  digitalWrite(enablePin, HIGH);  // Disable motor when end is reached

  delay(2000);
  moveToHome();
}

// Function to drive a stepper motor step
void stepMotor() {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepDelay);
}

// Function to update the LCD display with the game status and touch count
void updateLCD(const char* status, uint16_t count) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Status: ");
  lcd.print(status);
  lcd.setCursor(0, 1);
  lcd.print("Count hit = ");
  lcd.print(count);
}
void updateValue(uint16_t count) {
  lcd.setCursor(12, 1);
  lcd.print(count);
}
