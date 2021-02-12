#include <EEPROM.h>

// constants won't change. They're used here to set pin numbers:
const byte buttonPin = 4;    // the number of the pushbutton pin
const byte ledStrip = 5;     // the PWM pin the LED is attached to
const byte interruptPinHigh = 2; // the Interrupt pin of the sensor
const byte interruptPinLow = 3; // the Interrupt pin of the sensor
/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/
const int addr = 0;

// Variables will change:
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
int brightnessLevel[] = {0, 50, 100, 200, 255};
int reading = LOW;

volatile byte state = LOW;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  pinMode(ledStrip, OUTPUT);
  if(EEPROM.read(addr) < 255){
    brightness = EEPROM.read(addr);
  }
  Serial.println(EEPROM.read(addr));
  Serial.println(brightness);
  analogWrite(ledStrip, brightnessLevel[brightness]);
  attachInterrupt(digitalPinToInterrupt(interruptPinHigh), turnon, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPinLow), turnoff, LOW);
}

void loop() {
   if(state==HIGH){

    analogWrite(ledStrip, brightnessLevel[brightness]);
    // read the state of the switch into a local variable:
    reading = digitalRead(buttonPin);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != buttonState) {
        buttonState = reading;
        // only toggle the LED if the new button state is HIGH
        if (buttonState == HIGH) {
          if(brightness<4){
            brightness = brightness+1;
          }
          else {
            brightness=0;
          }
          // set the brightness of pin 3:
          Serial.println(brightness);
          analogWrite(ledStrip, brightnessLevel[brightness]);
          EEPROM.write(addr, brightness);
        }
      }
    }
  }
  else{
     analogWrite(ledStrip,0);
  }
  

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}

void turnon() {
  state = HIGH;
}

void turnoff() {
  state = LOW;
}
