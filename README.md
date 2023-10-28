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
</details>

## Photo of the circuit
![ELEVATOR-SIMULATOR](https://github.com/Ciocanesku/IntroductionToRobotics/assets/103603726/b84ffb8b-ae7e-40e3-a5bd-a8ae528422d4)

## Video
[Watch the video](https://www.youtube.com/shorts/PnvZjtRN8aw)
