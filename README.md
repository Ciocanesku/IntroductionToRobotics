# Introduction to Robotics (2023 - 2024)

Repository for Robotics Lab Assignments - Completed during the 3rd Year at the University of Bucharest's Faculty of Mathematics and Computer Science. Each assignment includes task descriptions, implementation details, as well as relevant code and image files.


# Homework
<details>
<summary>Homework 1</summary>

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
