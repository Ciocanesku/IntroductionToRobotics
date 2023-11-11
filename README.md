# Introduction to Robotics (2023 - 2024)

Repository for Robotics Lab Assignments - Completed during the 3rd Year at the University of Bucharest's Faculty of Mathematics and Computer Science. Each assignment includes task descriptions, implementation details, as well as relevant code and image files.


# Homework
<details>
<summary>Homework 2 - RGB with three potentiometers</summary>

## Requirements

Use a separate potentiometer for controlling each color of the RGB LED: Red, Green, and Blue. This control must leverage digital electronics. Specifically, you need to read the potentiometer's value with Arduino and then write a mapped value to the LED pins.

## Photo of the circuit

![RGB LED](https://github.com/Ciocanesku/IntroductionToRobotics/assets/103603726/e1b534ac-20ef-4651-91b1-f876045eacea)

## Link for video

[Watch the video](https://www.youtube.com/shorts/guyWlb159wo)

## Code

```arduino
// declarare pini led
const int ledPinGreen = 8;
const int ledPinBlue = 9;
const int ledPinRed = 10;

//declarare pini potentiometre
const int potPinRed = A2;
const int potPinBlue = A1;
const int potPinGreen = A0;

//declarare valori led-uri
int redPinVal = 0;
int bluePinVal = 0;
int greenPinVal = 0;

//declarare valori potentiometre
int redPotVal = 0;
int bluePotVal = 0;
int greenPotVal = 0;

void setup() {

  Serial.begin(9600);
}

void loop() {
    //citire valori potentiometre
    redPotVal = analogRead(potPinRed);
    bluePotVal = analogRead(potPinBlue);
    greenPotVal = analogRead(potPinGreen);


    //scriere valori pini led cu functia map()
    redPinVal = map(redPotVal, 0, 1023, 0, 255); 
    greenPinVal = map(greenPotVal, 0, 1023, 0, 255); 
    bluePinVal = map(bluePotVal, 0, 1023, 0, 255);  

    //setare culori RGB
    analogWrite(ledPinRed,redPinVal);
    analogWrite(ledPinGreen,greenPinVal);
    analogWrite(ledPinBlue,bluePinVal);
}
```
</details>

<details>
  <summary>Homework 3 - Elevator Simulator</summary>

## Requirements
This project simulates a 3-floor elevator using the Arduino platform. It includes LED indicators, buttons for each floor, a buzzer for audio feedback, and implements a control system to manage elevator movements and user interactions.
- LED Indicators: Each of the 3 LEDs represents one of the 3 floors. The LED corresponding to the current floor lights up. An additional LED represents the elevator's operational state, blinking when the elevator is moving and remaining static when stationary.

- Buttons: Three buttons are implemented to simulate call buttons from the 3 floors. When pressed, the elevator simulates movement towards the pressed floor after a short interval (2-3 seconds).

- Buzzer: The buzzer provides audio feedback in the following scenarios:
  - Elevator arriving at the desired floor (something resembling a "cling").
  - Elevator doors closing and movement (split into two different sounds).

- State Change & Timers: The system handles state changes and timers. If the elevator is already at the desired floor, pressing the button for that floor has no effect. Otherwise, after a button press, the elevator waits for the doors to close and then moves to the corresponding floor. If the elevator is in movement, it either does nothing or stacks its decision (gets to the first programmed floor, opens the doors, waits, closes them, and then goes to the next desired floor).

- Debounce: Debounce is implemented for the buttons to avoid unintentional repeated button presses.

## Components
- 4 LEDs for floor indicators and elevator state
- 4 resistors, 3-220 ohm for floor LEDs and 1-330 ohm for elevator LED (they should be between 220-330, it just happened to have those 4 near me when I made the circuit)
- 3 push buttons for call buttons
- Buzzer for audio feedback

## Photo of the circuit
![ELEVATOR-SIMULATOR](https://github.com/Ciocanesku/IntroductionToRobotics/assets/103603726/b84ffb8b-ae7e-40e3-a5bd-a8ae528422d4)

## Video
[Watch the video](https://youtube.com/shorts/rlsuzNrGbUo)

## Code
```arduino
// Declare all the pins
const int pinButtonFirstFloor = 2;
const int pinButtonSecondFloor = 3;
const int pinButtonThirdFloor = 4;

const int pinLedFirstFloor = 8;
const int pinLedSecondFloor = 9;
const int pinLedThirdFloor = 10;
const int pinLedElevator = 11;

const int pinBuzzer = 13;

// Declare initial states
byte stateButtonFirstFloor = LOW;
byte stateButtonSecondFloor = LOW;
byte stateButtonThirdFloor = LOW;

byte stateLedFirstFloor = LOW;
byte stateLedSecondFloor = LOW;
byte stateLedThirdFloor = LOW;
byte stateLedElevator = LOW;

byte stateBuzzer = LOW;

// Declare the buzzer tone values
int buzzerToneClosing = 1000;
int buzzerToneMoving = 500;
int buzzerToneArriving = 2000;



// Variables for elevator state
int currentFloor = 1;  
int nextFloor = 1; 
int doorCloseTime = 0;  
const int doorCloseDuration = 2000;  
byte doorClose = false;
int ledElevatorBlinkTime = 500;
int ledElevatorLastBlinkTime = 0;

// Elevator level-switching variables
bool levelSwitching = false;
int levelSwitchEndTime = 0;
const int levelSwitchTime = 2000; 
const int arrivingTime = 100;
int arrivingTimeEnd = 0;
bool arrivedNextFloor = false;

// Button-press queue
const int maxQueueSize = 5;
int buttonPressQueue[maxQueueSize];  
int queueFront = 0;
int queueRear = 0;
int queueSize = 0;

// Debounce variables
const int time = 50;  
int lastTime = 0;

void setup() {
  pinMode(pinButtonFirstFloor, INPUT_PULLUP);
  pinMode(pinButtonSecondFloor, INPUT_PULLUP);
  pinMode(pinButtonThirdFloor, INPUT_PULLUP);

  pinMode(pinLedFirstFloor, OUTPUT);
  pinMode(pinLedSecondFloor, OUTPUT);
  pinMode(pinLedThirdFloor, OUTPUT);
  pinMode(pinLedElevator, OUTPUT);

  pinMode(pinBuzzer, OUTPUT);
}

void loop() {
// Read button states and apply the handleButtonPress function
  byte readingButtonFirstFloor = digitalRead(pinButtonFirstFloor);
  byte readingButtonSecondFloor = digitalRead(pinButtonSecondFloor);
  byte readingButtonThirdFloor = digitalRead(pinButtonThirdFloor);


  if (millis() - lastTime >= time) {  //debouncing line
    if (readingButtonFirstFloor != stateButtonFirstFloor) { //checking if any button is pressed, if it is, we add the value in the queue
      if (readingButtonFirstFloor == LOW) {
        addToQueue(1);
      }
      stateButtonFirstFloor = readingButtonFirstFloor;
      lastTime = millis();
    }

    if (readingButtonSecondFloor != stateButtonSecondFloor) {
      if (readingButtonSecondFloor == LOW) {
        addToQueue(2);
      }
      stateButtonSecondFloor = readingButtonSecondFloor;
      lastTime = millis();
    }

    if (readingButtonThirdFloor != stateButtonThirdFloor) {
      if (readingButtonThirdFloor == LOW) {
        addToQueue(3);
      }
      stateButtonThirdFloor = readingButtonThirdFloor;
      lastTime = millis();
    }
  }
   
   if(queueSize > 0 && !doorClose && !levelSwitching && !arrivedNextFloor){ //if we have elements in queue and we are not "using" the elevator we can go to the next button pressed 
    nextFloor = buttonPressQueue[queueFront];

    if (nextFloor != currentFloor)  // if the next floor is not the one we are currently at, we can start the trip
    {
      doorClose=true;
      currentFloor=nextFloor;
      levelSwitching = false;
      arrivedNextFloor = false;
      doorCloseTime = millis() + doorCloseDuration;
      levelSwitchEndTime = doorCloseTime + levelSwitchTime;
      arrivingTimeEnd = levelSwitchEndTime + arrivingTime;
      stateLedElevator = HIGH;
      digitalWrite(pinLedElevator, stateLedElevator);
    }
    
    removeFromQueue();
   }

  if(doorClose) //while the door is closing, we hear the sound
    {soundBuzzer(1);}

  if (doorClose && millis() >= doorCloseTime) {  //start timer for closing doors, turn all the leds off
    stateLedFirstFloor = LOW;
    stateLedSecondFloor = LOW;
    stateLedThirdFloor = LOW;
    digitalWrite(pinLedFirstFloor,stateLedFirstFloor);
    digitalWrite(pinLedSecondFloor,stateLedSecondFloor);
    digitalWrite(pinLedThirdFloor, stateLedSecondFloor);
    if (!levelSwitching) {
      levelSwitching = true;
    }
    doorClose= false;
    noTone(pinBuzzer);
  }

   if (levelSwitching && (millis() - ledElevatorLastBlinkTime >= ledElevatorBlinkTime)) { //led blinks while level switching
    ledElevatorLastBlinkTime = millis();
    digitalWrite(pinLedElevator, !digitalRead(pinLedElevator)); 
  }

  if(levelSwitching)
  {soundBuzzer(2);}

  if(levelSwitching && millis() >= levelSwitchEndTime) //end the transition
  {
    levelSwitching=false;
    if(!arrivedNextFloor)
      arrivedNextFloor = true;
    noTone(pinBuzzer);
  }

    if(!levelSwitching && !doorClose) //after the transition is made, we turn on the led for current level
  {
  digitalWrite(pinLedFirstFloor, currentFloor == 1 ? HIGH : LOW);
  digitalWrite(pinLedSecondFloor, currentFloor == 2 ? HIGH : LOW);
  digitalWrite(pinLedThirdFloor, currentFloor == 3 ? HIGH : LOW);
  }

  if(arrivedNextFloor) // check if arrived next floor to sound buzzer
    {
      soundBuzzer(3);
    }
  
    if(arrivedNextFloor && millis() >= arrivingTimeEnd) //end the transition
  {
    arrivedNextFloor = false;
    noTone(pinBuzzer);
  }


}


void addToQueue(int floor){  //function for adding elements to queue
  if (queueSize < maxQueueSize) {
    buttonPressQueue[queueRear] = floor;
    queueRear = queueRear + 1;
    queueSize++;
  }
}

void removeFromQueue(){  //function for removing 
  if(queueSize>0)
  {
    for(int i = queueFront; i<queueRear ; i++)
    {
      buttonPressQueue[i]=buttonPressQueue[i+1];
    }
    queueRear = queueRear-1;
    queueSize--;
  }
}

void soundBuzzer(int buzzerType) { //function for the two types of buzzers
  if (buzzerType == 1) {
    // sound moving
    tone(pinBuzzer, buzzerToneMoving, levelSwitchTime);
  }
  if (buzzerType == 2) {
    // sound door closing
    tone(pinBuzzer, buzzerToneClosing, doorCloseDuration);
  }
  if(buzzerType == 3){
    // sound arriving
    tone(pinBuzzer, buzzerToneArriving, arrivingTime);
  }
}
```

</details>

<details>
<summary>Homework 4 - 7DD controlled using a joystick</summary>

## Requirements
Use the joystick to control the position ofthe segment and ”draw” on the display.  The movement between segments should be natural, meaning they should jump from the current positiononly to neighbors, but without passing through ”walls”. The  initial  position  should  be  on  the  DP.  The  current position always blinks (irrespective of the fact that the segment is on or off).  Use the joystick to move from one position to neighbors (see table for corresponding movement).  Short pressing the button toggles the segmentstate  from  ON  to  OFF  or  from  OFF  to  ON.  Long  pressing  the  button resets the entire display by turning all the segments OFF and moving thecurrent position to the decimal point.

## Neighbors table
![image](https://github.com/Ciocanesku/IntroductionToRobotics/assets/103603726/f22934ff-6c4c-4878-ba37-badac2befa32)


## Components
- 7 digit display
- 8 resistors between 220-330 ohm
- Joystick

## Photo of the circuit
![7DD controlled with joystick](https://github.com/Ciocanesku/IntroductionToRobotics/assets/103603726/ec06d837-e228-4569-a9a5-9051916277b5)

## Electric scheme

![image](https://github.com/Ciocanesku/IntroductionToRobotics/assets/103603726/f143e25a-f292-4208-82ac-e084300916e9)


## Link for video

[Watch the video](https://youtu.be/z_FnSDS-nrg?si=n3D7EGPW7yPIr7tK)

## Code

```arduino
// declare all the pins
const int pinSW = 2; // digital pin connected to switch output
const int pinX = A0; // pin X output
const int pinY = A1; // pin Y output
byte swState = LOW;
int xValue = 0;
int yValue = 0;

// declare all the segments pins
const int segmentPinA = 12;
const int segmentPinB = 10;
const int segmentPinC = 9;
const int segmentPinD = 8;
const int segmentPinE = 7;
const int segmentPinF = 6;
const int segmentPinG = 5;
const int segmentPinDP = 4;
const int segSize = 8;
int currentSegmentIndex = 7; // start with the point


int segments[segSize] = {
  segmentPinA, segmentPinB, segmentPinC, segmentPinD, segmentPinE, segmentPinF, segmentPinG, segmentPinDP
};

const int trasholdRight = 620;
const int trasholdLeft = 420;
const int trasholdUp = 607;
const int trasholdDown = 407;

int movementMatrix[8][4] = {  // UP, DOWN, LEFT, RIGHT
  {-1, 6, 5, 1}, // a
  {0, 6, 5, -1}, // b
  {6, 3, 4, 7}, // c
  {6, -1, 4, 2}, // d
  {6, 3, -1, 2}, // e
  {0, 6, -1, 1}, // f
  {0, 3, -1, -1}, // g
  {-1, -1, 2, -1}, // dp
};

bool moving = false;

const int impossibleMove = -1;

const int blinkTime = 500;  // blink interval
unsigned long lastBlinkTime = 0;
bool segmentOn = false;

int holdSegments[9]; //array for storing the segments that are "clicked"
int holdSegmentsSize = -1;

byte buttonState = HIGH;    // current state of the button
byte lastButtonState = HIGH;  // previous state of the button
byte reading = LOW;
unsigned long buttonPressStartTime = 0;
const int resetTime = 2000;
unsigned long lastDebounceTime = 0;  // time of the last button state change
unsigned long debounceDelay = 50;  // debounce time in milliseconds


void setup() {
  // initialize all the pins
  pinMode(pinSW, INPUT_PULLUP);
  for (int i = 0; i < segSize; i++) {
    pinMode(segments[i], OUTPUT);
  }
  Serial.begin(9600);
}

void loop() {
  xValue = analogRead(pinX);
  yValue = analogRead(pinY);
  reading = digitalRead(pinSW);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();  // update the last debounce time
  }

  if (millis() - lastDebounceTime > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        if (buttonPressStartTime == 0) {
          buttonPressStartTime = millis();  // record the start time of button press
        }
      } else {
        if (millis() - buttonPressStartTime >= resetTime) {
          // if the button has been held for 2 seconds or more, reset calling clearHold()
          clearHold();
        } else {
          holdUnhold(currentSegmentIndex);
          Serial.print(currentSegmentIndex);
        }
        buttonPressStartTime = 0;  // reset the start time when the button is released
      }
    }
  }

  lastButtonState = reading;

  if (moving && xValue >= trasholdLeft && xValue <= trasholdRight && yValue >= trasholdDown && yValue <= trasholdUp) { //button is no more moving
    moving = !moving;
  }

  if (!moving) { //check to start the moving state, and get the next currentSegmentIndex from the moveMatrix
    if (xValue >= trasholdRight) {
      if (movementMatrix[currentSegmentIndex][3] != impossibleMove) { //move right
        currentSegmentIndex = movementMatrix[currentSegmentIndex][3];
      }
      moving = true;
    } else if (xValue <= trasholdLeft) { //move left
      if (movementMatrix[currentSegmentIndex][2] != impossibleMove) {
        currentSegmentIndex = movementMatrix[currentSegmentIndex][2];
      }
      moving = true;
    } else if (yValue <= trasholdDown) { //move down
      if (movementMatrix[currentSegmentIndex][0] != impossibleMove) {
        currentSegmentIndex = movementMatrix[currentSegmentIndex][0];
      }
      moving = true;
    } else if (yValue >= trasholdUp) { //move up
      if (movementMatrix[currentSegmentIndex][1] != impossibleMove) {
        currentSegmentIndex = movementMatrix[currentSegmentIndex][1];
      }
      moving = true;
    }
  }

  // reset all segments
  for (int i = 0; i < segSize; i++) {
    digitalWrite(segments[i], LOW);
  }

  if (segmentOn) {
    digitalWrite(segments[currentSegmentIndex], HIGH);
  }

  for (int i = 0; i <= holdSegmentsSize; i++) {
    if (currentSegmentIndex != holdSegments[i]) {
      digitalWrite(segments[holdSegments[i]], HIGH);
    }
  }

  if (millis() - lastBlinkTime >= blinkTime) { //used to make the current position blink
    lastBlinkTime = millis();
    segmentOn = !segmentOn;
  }
}

void holdUnhold(int i) { //function to add/remove elements from array
  int ok = 0;
  for (int j = 0; j <= holdSegmentsSize && ok == 0; j++) {
    if (holdSegments[j] == i) { //if already in the array, delete it from there
      removeFromHold(j);
      ok = 1;
    }
  }
  if (ok == 0) { // if not in the array, put it in
    holdSegmentsSize++;
    holdSegments[holdSegmentsSize] = i;
  }
}

void removeFromHold(int i) {
  for (int k = i; k < holdSegmentsSize; k++) {
    holdSegments[k] = holdSegments[k + 1];
  }
  holdSegmentsSize--;
}

void clearHold() { //function to reset the display
  holdSegmentsSize = -1;
  currentSegmentIndex = 7;
  for (int i = 0; i < segSize; i++) {
    digitalWrite(segments[i], LOW);
  }
}
```
</details>

<details>
<summary>Homework 5 - 4-7DD stopwatch timer </summary>

## Requirements

This project involves creating a stopwatch timer using a 4-digit 7-segment display and 3 buttons. The display initially shows "000.0," and the buttons have specific functions:

1. **Button 1 (Start/Pause):**
   - Starts or pauses the timer.

2. **Button 2 (Reset):**
   - Resets the timer if in pause mode.
   - Resets saved laps if in lap viewing mode.

3. **Button 3 (Save Lap):**
   - Saves lap times during counting mode, up to 4 laps.
   - Cycles through the last saved laps when pressed (up to 4 laps).

**Workflow:**
1. Display shows "000.0." Pressing Start initiates the timer.
2. During timer counting, pressing the lap button saves the timer value (up to 4 laps).
   - 5th press overrides the 1st saved lap.
   - Reset button has no effect during counting.
   - Pause button stops the timer.
3. In Pause mode, lap button is disabled. Reset button resets the display to "000.0."
4. After reset, pressing lap button cycles through saved laps.
   - Continuous pressing cycles laps continuously.
   - Reset button resets flags and timer to "000.0."

## Photo of the circuit

![image](https://github.com/Ciocanesku/IntroductionToRobotics/assets/103603726/443514bd-a8a0-4771-9a9d-bfa2b13de050)

## Electric Scheme

![image](https://github.com/Ciocanesku/IntroductionToRobotics/assets/103603726/2bda14c0-0e01-4f7c-abb6-ba2e0487d336)


## Link for video

[Watch the video](https://youtube.com/shorts/UBYtMDAq3_s)

## Code

```arduino
// Pin assignments for the shift register
const int latchPin = 11; // Pin connected to STCP of the shift register (Latch control)
const int clockPin = 10; // Pin connected to SHCP of the shift register (Clock)
const int dataPin = 12; // Pin connected to DS of the shift register (Data input)

// Pin assignments for controlling the common cathode/anode pins of the 7-segment digits
const int segD1 = 4;
const int segD2 = 5;
const int segD3 = 6;
const int segD4 = 7;

// Buttons Pins
const int buttonStartPin = 2;
const int buttonResetPin = 3;
const int buttonLapPin = 8;

// Size of the register in bits
const byte regSize = 8;

// Array to keep track of the digit control pins
int displayDigits[] = {segD1, segD2, segD3, segD4};
const int displayCount = 4; // Number of digits in the display

// Array representing the state of each bit in the shift register
byte registers[regSize];

// Array holding binary encodings for numbers and letters on a 7-segment display
const int encodingsNumber = 10;
byte byteEncodings[encodingsNumber] = {
    // Encoding for segments A through G and the decimal point (DP)
    // A B C D E F G DP
    B11111100, // 0
    B01100000, // 1
    B11011010, // 2
    B11110010, // 3
    B01100110, // 4
    B10110110, // 5
    B10111110, // 6
    B11100000, // 7
    B11111110, // 8
    B11110110  // 9
};

int activeDisplay = 0;

unsigned long currentSeconds = 0;
unsigned long currentTenthOfSeconds = 0;

unsigned long lastIncrement = 0;
unsigned long delayCount = 50; // Delay between updates (milliseconds)
unsigned long number = 0;      // The number being displayed
const int numberOf10thOfSecondsInMillisecond = 100;
const int tenPowerForGetting4Digits = 10000;

const int debounceDelay = 50; // Adjust as needed

byte readingButtonStart = HIGH;
byte readingButtonReset = HIGH;
byte readingButtonLap = HIGH;

byte stateButtonStart = HIGH;
byte stateButtonReset = HIGH;
byte stateButtonLap = HIGH;

byte activeButtonStart = LOW;
byte activeButtonReset = HIGH;
byte activeButtonLap = HIGH;

// Debounce variables
const int time = 50;  
unsigned long lastTime = 0;

const int printTime = 1000;
unsigned long lastPrintTime = 0;

unsigned long lastStopTime = 0;
unsigned long firstStopTime = 0;
unsigned long lastStartTime = 0;
unsigned long pauseStartTime = 0;
unsigned long elapsedTime = 0;

const int numberOfDigits = 10000;

int stopTimer=0;
const int lapsNumber=4;
unsigned long laps[lapsNumber];
int lapsIndex = -1;
int lapsViewMode = 0;
bool lapButtonPressed = false;
bool resetButtonPressed = false;
int lapDisplayIndex = -1;

unsigned long lastLapShow = 0;
unsigned long lapShowTime = 250;

void setup()
{
    // Initialize the digital pins connected to the shift register as outputs
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
    pinMode(buttonStartPin, INPUT_PULLUP);
    pinMode(buttonResetPin, INPUT_PULLUP);
    pinMode(buttonLapPin, INPUT_PULLUP);

    // Initialize the digit control pins as outputs and turn them off
    for (int i = 0; i < displayCount; i++)
    {
        pinMode(displayDigits[i], OUTPUT);
        digitalWrite(displayDigits[i], LOW);
    }

    Serial.begin(9600);
}

void loop()
{
   debounce();

    // helperPrint();

    startPause();

    reset();

    lapAdder();

    lapOrTimer();



}

void writeReg(int digit)
{
    // Prepare to shift data by setting the latch pin low
    digitalWrite(latchPin, LOW);
    // Shift out the byte representing the current digit to the shift register
    shiftOut(dataPin, clockPin, MSBFIRST, digit);
    // Latch the data onto the output pins by setting the latch pin high
    digitalWrite(latchPin, HIGH);
}

void writeNumber(unsigned long nr)
{
    int currentDigit = 0;
    for (int i = displayCount - 1; i >= 0; i--)
    {
        currentDigit = nr % 10;
        nr = nr / 10;
        activateDisplay(i);
        writeReg(B00000000);
        if (i == 2) //scriu si punctul pe display 3
        {
            byteEncodings[currentDigit] += 1;
        }
        writeReg(byteEncodings[currentDigit]);
        if (i == 2)
        {
            byteEncodings[currentDigit] -= 1;
        }
        writeReg(B00000000);
    }
}

void activateDisplay(int displayNumber)
{
    // Turn off all digit control pins to avoid ghosting
    for (int i = 0; i < displayCount; i++)
    {
        digitalWrite(displayDigits[i], HIGH);
    }
    // Turn on the current digit control pin
    digitalWrite(displayDigits[displayNumber], LOW);
}

void debounce()
{
    readingButtonStart = digitalRead(buttonStartPin);
    readingButtonReset = digitalRead(buttonResetPin);
    readingButtonLap = digitalRead(buttonLapPin);
   if (millis() - lastTime >= time) {  //debouncing line
    if (readingButtonStart != stateButtonStart) { //checking if any button is pressed, if it is, we add the value in the queue
      if (readingButtonStart == LOW) {
        activeButtonStart=!activeButtonStart;
      }
      stateButtonStart = readingButtonStart;
      lastTime = millis();
    }

    if (readingButtonReset != stateButtonReset) {
      if (readingButtonReset == LOW && !activeButtonStart) {
        resetButtonPressed = true;
      }
      stateButtonReset = readingButtonReset;
      lastTime = millis();
    }

    if (readingButtonLap != stateButtonLap) {
      if (readingButtonLap == LOW) {
        lapButtonPressed = true;
      }
      stateButtonLap = readingButtonLap;
      lastTime = millis();
    }
  }
}

void helperPrint()
{
      if(millis() - lastPrintTime >=printTime)
    {
    // Print the states of debounced buttons for verification
    Serial.print("Button Start: ");
    Serial.println(activeButtonStart);
    Serial.print("Button Reset: ");
    Serial.println(activeButtonReset);
    Serial.print("Button Lap: ");
    Serial.println(activeButtonLap);
    Serial.println("---------------------");
    for(int i=0; i<4;i++)
      {Serial.print("LAP ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println(laps[i]);}
    lastPrintTime = millis();
    }
}

void startPause()
{
        if (activeButtonStart)
    {
        // Timer is activated, record the elapsed time
        elapsedTime = millis() - pauseStartTime;
        activeButtonReset = 0;
        lapsViewMode = 0;
    }

    if (!activeButtonStart)
    {
        // Timer is stopped, calculate the pause starting time
        pauseStartTime = millis() - elapsedTime;

    }
    number = elapsedTime / numberOf10thOfSecondsInMillisecond;
}

void reset()
{
  if(resetButtonPressed == 1 && activeButtonReset == HIGH)
  {
    resetLaps();
    resetButtonPressed = 0;
  }

  if(resetButtonPressed == 1 && activeButtonStart == LOW)
    {elapsedTime = 0;
     pauseStartTime = millis();
     activeButtonReset=1;
     resetButtonPressed = 0;
     lapDisplayIndex=-1;
    }
}

void addLap(int nr)
{
  lapsIndex++;
  laps[lapsIndex]=nr;
  if(lapsIndex == 3)
    lapsIndex = -1;
}

void resetLaps()
{
  lapsIndex = -1;
  for(int i=0 ; i<lapsNumber; i++)
    {
      laps[i]=0;
    }
}

void lapAdder()
{
      if(lapButtonPressed && activeButtonStart == 1)
    {
      addLap(number%numberOfDigits);
      lapButtonPressed = !lapButtonPressed;
    }
}

void lapOrTimer()
{
  if(lapButtonPressed == 1 && activeButtonReset)
    {
      lapsViewMode = 1;
    }

    if(lapsViewMode)
    {
      if(stateButtonLap==LOW &&  millis()-lastLapShow >= lapShowTime)
      {
      lapButtonPressed = 0;
      lapDisplayIndex++;
      if(lapDisplayIndex == 4)
        lapDisplayIndex = 0;
      lastLapShow = millis();
      }
      writeNumber(laps[lapDisplayIndex]);
    }
    else
    {
      writeNumber(number);
    }
}

```
</details>


