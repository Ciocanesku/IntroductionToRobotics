# Introduction to Robotics (2023 - 2024)

Repository for Robotics Lab Assignments - Completed during the 3rd Year at the University of Bucharest's Faculty of Mathematics and Computer Science. Each assignment includes task descriptions, implementation details, as well as relevant code and image files.


# Homework
<details>
<summary>Homework 1 - RGB with three potentiometers</summary>

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
  <summary>Homework 2 - Elevator Simulator</summary>

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
[Watch the video](https://www.youtube.com/shorts/PnvZjtRN8aw)

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
   
   if(queueSize > 0 && !doorClose && !levelSwitching){ //if we have elements in queue and we are not "using" the elevator we can go to the next button pressed 
    nextFloor = buttonPressQueue[queueFront];

    if (nextFloor != currentFloor)  // if the next floor is not the one we are currently at, we can start the trip
    {
      doorClose=true;
      currentFloor=nextFloor;
      levelSwitching = false;
      doorCloseTime = millis() + doorCloseDuration;
      levelSwitchEndTime = doorCloseTime + levelSwitchTime;
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
  }

  if(!levelSwitching && !doorClose) //after the transition is made, we turn on the led for current level
  {
  digitalWrite(pinLedFirstFloor, currentFloor == 1 ? HIGH : LOW);
  digitalWrite(pinLedSecondFloor, currentFloor == 2 ? HIGH : LOW);
  digitalWrite(pinLedThirdFloor, currentFloor == 3 ? HIGH : LOW);}
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
    tone(pinBuzzer, buzzerToneMoving, 500);
  } else if (buzzerType == 2) {
    // sound door closing
    tone(pinBuzzer, buzzerToneClosing, 500);
  }
}
```

</details>

