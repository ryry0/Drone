/* ESC Driver for Arduino Uno

 Changes the PPM signal sent to an ESC on the quadcopter for throttle control.
 This example outputs the signal to pin 13 on the Arduino Uno. The serial
 monitor on the Arduino IDE can be used to change the PPM signal to preset
 levels.

 Serial commands:
  o = off
  s = slow (10%)
  h = half on
  f = full on
  i = increase by 1%
  d = decrease by 1%

 IMPORTANT: Do NOT connect the ESC logic control to the Arduino +5V source,
            or you may burn out your USB. The ESC logic board uses this +5v
            source to vibrate the motor to produce it's sonic output!

 */

// constants won't change. Used here to set a pin number :
const int ledPin =  13;      // the number of the LED pin

// Variables will change :
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long previousOnMillis = 0;      // will store last time LED was switched on

// constants won't change :
const unsigned long interval = 20000;           // interval at which to blink (microseconds)
const unsigned long max_pulse = 2000;
unsigned long pulse_interval = max_pulse >> 1;      // interval of on segment (microseconds)

void setup() {
  // set the digital pin as output:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // here is where you'd put code that needs to be running all the time.

  // check to see if it's time to blink the LED; that is, if the
  // difference between the current time and last time you blinked
  // the LED is bigger than the interval at which you want to
  // blink the LED.
  unsigned long currentMillis = micros();

  char incoming_byte = 0;
  if (Serial.available() > 0) {
    incoming_byte = Serial.read();

    switch(incoming_byte) {
      case 'f':
      pulse_interval = max_pulse;
      break;

      case 'o':
      pulse_interval = max_pulse >> 1;
      break;

      case 'h':
      pulse_interval = (max_pulse * 3) /4;
      break;

      case 's':
      pulse_interval = (max_pulse >> 1) + 100;
      break;

      case 'i':
      pulse_interval += 10;
      break;

      case 'd':
      pulse_interval -= 10;
      break;

    }
  }

  if (currentMillis - previousMillis >= interval && pulse_interval != 0) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    previousOnMillis = currentMillis;

    ledState = HIGH;

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);

  } else if (currentMillis - previousOnMillis >= pulse_interval) {
    ledState = LOW;
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }


}

