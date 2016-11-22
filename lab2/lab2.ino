#define TEMP_MAX 25
#define BLINK_MIN 200
#define BLINK_MAX 2000

const int tempPin = A0;
const int tempLed = 2;

const int lightPin = A1;
const int lightLed = 3;

const int potPin = A2;
const int potLed = 4;
int potLedState = LOW;

// will store last time LED was updated
unsigned long previousMillis = 0;

unsigned int minLight = 1023;
unsigned int maxLight = 0;

void setup() {
  pinMode(tempLed, OUTPUT);
  digitalWrite(tempLed, LOW);
  pinMode(lightLed, OUTPUT);
  digitalWrite(lightLed, LOW);
  pinMode(potLed, OUTPUT);
  digitalWrite(potLed, LOW);
}


void loop() {
  doExecuteTemperature();

  doExecuteLight();

  doExecutePotentiometer();
}


void doExecuteTemperature() {
  //getting the voltage reading from the temperature sensor
  int reading = analogRead(tempPin);

  // small calculations
  float tempVoltage = ((reading / 1024.0) * 5.0) - 0.5;

  // final calculation
  int temperature = tempVoltage * 100;

  if (temperature > TEMP_MAX)
    digitalWrite(tempLed, HIGH);
  else
    digitalWrite(tempLed, LOW);
}


void doExecuteLight() {
  // get current value
  int lightVal = analogRead(lightPin);

  if(lightVal < minLight)
    minLight = lightVal;
  if(lightVal > maxLight)
    maxLight = lightVal;

  // map value
  lightVal = map(lightVal, minLight, maxLight, 0, 255);
  
  analogWrite(lightLed, lightVal);
}


void doExecutePotentiometer() {
  // get current value and map
  int potVal = analogRead(potPin);

  // map value
  potVal = getAnalogMap(potVal, BLINK_MIN, BLINK_MAX);
  
  // get current time
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= potVal) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa
    if (potLedState == LOW) {
      potLedState = HIGH;
    } else {
      potLedState = LOW;
    }

    // set the LED with the ledState of the variable
    digitalWrite(potLed, potLedState);
  }
}


int getSimpleAnalogMap(int value, int maxValue) {
  double tempRes = maxValue / 1023.0; // keep it double
  return (tempRes * value);
}


int getAnalogMap(int value, int minValue, int maxValue) {
  double tempRes = ((maxValue - minValue) / 1023.0); // keep it double
  return ((tempRes * value) + minValue);
}

