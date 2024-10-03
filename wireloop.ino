#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define dirPin 8       // Control direction (high = forward) / (low = backward)
#define stepPin 12     // Pulse pin to drive motor
#define enablePin 10   // (low = on) / (high = off)
#define homeSwitch 7   // Home position pin (pull-up)j
#define endSwitch 3    // End position pin (pull-up)
#define startButton 4  // Start button
#define touchPin 2     // Touch sensor pin
#define speakerPin 6   // Signal for speaker

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
const unsigned long speakerTime = 200;
uint8_t timeLimit = 30;
unsigned long startTime = 0;
uint8_t remainingTime = timeLimit;
unsigned long nextStep = 0;
bool speakerOn = false;

LiquidCrystal_I2C lcd(0x3f, 16, 2);  // Declare i2c LCD
void moveToHome();
void moveToEnd();
void updateLCD(const char* status, uint16_t count);
void updateValue(uint16_t count);
void updateTime(uint8_t time);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Board Start");
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(homeSwitch, INPUT_PULLUP);
  pinMode(endSwitch, INPUT_PULLUP);
  pinMode(touchPin, INPUT_PULLUP);
  pinMode(startButton, INPUT_PULLUP);
  pinMode(speakerPin, OUTPUT);
  randomSeed(analogRead(0));
  lcd.begin();
  lcd.backlight();
  // Move the motor to home position initially
  moveToHome();
  // Show the initial status on the LCD
  updateLCD("Stop", touchCount);
}

void loop() {
  if (digitalRead(startButton) == LOW) {
    Serial.println("Start Button Detected!");
    // Start the game
    Serial.println("Starting game");
    delay(1000);
    gameRunning = true;
    touchCount = 0;  // Reset the touch count
    updateLCD("Run", touchCount);
    moveToEnd();  // Move towards the end position
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
  updateLCD("Stop", touchCount);
  digitalWrite(enablePin, LOW);  // Enable motor
  digitalWrite(dirPin, LOW);     // Set direction to backward (towards home)

  while (digitalRead(homeSwitch) == HIGH) {
    // Drive the motor until homeSwitch is pressed (LOW)
    stepMotor();
  }
  Serial.println("Reached Home position");
  digitalWrite(enablePin, HIGH);  // Disable motor when home is reached
}

// Function to move the motor towards the end position
void moveToEnd() {               // game running
  digitalWrite(enablePin, LOW);  // Enable motor
  //set time variable
  startTime = millis();
  remainingTime = timeLimit;
  nextStep = 1000;
  attachInterrupt(digitalPinToInterrupt(touchPin), handleTouch, LOW);
  // set flag if game hit endSwitch
  bool endSwitchFlag = false;
  //set variable for counting
  uint16_t step = 8000;
  uint8_t direction = 1;            // 1 = forward , 0 = backward
  digitalWrite(dirPin, direction);  // Set direction to forward (towards end)
  while (true) {
    //for loop to drive motor in step and by direction set
    for (uint16_t i = 0; i < step; i++) {
      //in for loop do 6 things
      //1. check time to update LCD for time remaining
      currentTime = millis();
      if ((currentTime - startTime) >= nextStep) {
        remainingTime--;
        nextStep += 1000;
        updateTime(remainingTime);
        if (remainingTime == 0) {  // timeout break and end game
          gameRunning = false;
          break;
        }
      }
      //2. check duration to enable touch pin again
      if (interruptOff) {
        if (currentTime - lastTouchTime >= debounceTime) {
          interruptOff = false;
          attachInterrupt(digitalPinToInterrupt(touchPin), handleTouch, LOW);
        }
      }
      //handle speaker on for interval
      if (speakerOn) {
        if (currentTime - lastTouchTime >= speakerTime) {
          digitalWrite(speakerPin, LOW);
          speakerOn = false;
        }
      }
      //3.handle touchpin if value from interrupt is flagged
      if (touchDetected) {
        touchCount++;
        updateValue(touchCount);
        lastTouchTime = millis();
        interruptOff = true;
        touchDetected = false;
        speakerOn = true;
        digitalWrite(speakerPin, HIGH);
      }
      //4.drive motor
      stepMotor();
      //5.Check for start button press to stop the game
      if (digitalRead(startButton) == LOW) {
        detachInterrupt(digitalPinToInterrupt(touchPin));
        gameRunning = false;
        updateLCD("Stopped", touchCount);
        break;
      }
      //6. if end switch is hit, go backward for 3000 step
      if (digitalRead(endSwitch) == LOW) {
        step = 3000;
        i = 0;
        endSwitchFlag = true;
        direction = 0;
        digitalWrite(dirPin, direction);
      }
    }
    if (!gameRunning) {
      break;
    }
    //set opposite direction and random step value
    if (direction == 1) {
      direction = 0;
      step = random(2500, 3500);
      digitalWrite(dirPin, direction);
      Serial.print("Going backward. Step is : ");
      Serial.println(step);
    } else {
      direction = 1;
      if (endSwitchFlag) {
        endSwitchFlag = false;
        // fix delay this flag
        step = 6000;
      } else {
        step = (step * 6) / 5;
      }
      digitalWrite(dirPin, direction);
      Serial.print("Going hforward. Step is : ");
      Serial.println(step);
    }
  }
  //reset state
  digitalWrite(speakerPin, LOW);

  // beep signal to indicate time over
  digitalWrite(speakerPin, HIGH);
  delay(1000);
  digitalWrite(speakerPin, LOW);
  delay(500);
  speakerOn = false;
  if (interruptOff) {
    interruptOff = false;
  } else {
    detachInterrupt(digitalPinToInterrupt(touchPin));
  }
  touchDetected = false;
  delay(1000);
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
  lcd.print(status);
  lcd.setCursor(6, 0);
  lcd.print("Time:");
  lcd.setCursor(11, 0);
  lcd.print(remainingTime);
  lcd.setCursor(0, 1);
  lcd.print(count);
}
void updateValue(uint16_t count) {
  lcd.setCursor(0, 1);
  lcd.print(count);
}

void updateTime(uint8_t time) {
  lcd.setCursor(11, 0);
  if (time >= 10) {
    lcd.print(time);
  } else {
    lcd.print("0");
    lcd.print(time);
  }
}
