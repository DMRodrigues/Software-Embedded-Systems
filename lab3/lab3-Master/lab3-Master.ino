#include <Wire.h>

/* Master:
    start with calibration of light and temperature, stores simple value
   prepares the buffer and sends 'START' msg loop until sended succesfully

   In loop gathers data from all sensors:
   - temperature saves last SAMPLE_TEMP values in array, calculates average and stores bufferData
   - light saves last SAMPLE_LIGHT values in array, calculates average and stores bufferData
   - potentiometer reads the input and stores bufferData

   After new values collected in the buffers, sends one packet
   with all the values and check option with inittial letter

   WARNING assumes global variables
*/

#define DEBUG 0

#define SLAVE_ADDR     8
#define SAMPLE_TEMP    5
#define SAMPLE_LIGHT   5
#define CALIBRATE_TIME 3000
#define MAX_BUFFER     8
#define TEMP_INDEX     1
#define LIGHT_INDEX    3
#define POT_INDEX      5
const String startString = "START";

byte bufferData[MAX_BUFFER];

const int tempPin = A0;
unsigned int tempReads[SAMPLE_TEMP];
unsigned int indexTemp = 0;

const int lightPin = A1;
unsigned int lightReads[SAMPLE_LIGHT];
unsigned int indexLight = 0;
unsigned int sensorMin = 128; // minimum sensor error
unsigned int sensorMax = 895; // maximum sensor error

const int potPin = A2;

void setup() {
  int reading;
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (DEBUG)
    Serial.begin(115200);

  Wire.begin(); // join i2c bus (address optional for master)

  while (millis() < CALIBRATE_TIME) { // calibrate the Light sensor
    reading = analogRead(lightPin);

    if (reading > sensorMax)
      sensorMax = reading;
    if (reading < sensorMin)
      sensorMin = reading;
  }
  for (indexLight = 0; indexLight < SAMPLE_LIGHT; indexLight++)
    lightReads[indexLight] = reading; // populate the array with one sample

  if (DEBUG)
    Serial.println("Calibrated Light");

  // get first temperature read to populate array (SIMPLE)
  reading = ((analogRead(tempPin) / 1024.0) * 5.0) - 0.5;
  reading = reading * 100;
  for (indexTemp = 0; indexTemp < SAMPLE_TEMP; indexTemp++)
    tempReads[indexTemp] = reading;

  if (DEBUG)
    Serial.println("Calibrated Temp");

  // better safe than sorry
  indexTemp = 0;
  indexLight = 0;

  setStartMsg(); // create the 'START' message
  if (DEBUG)
    Serial.println("Ready to send \'START\'");

  reading  = sendStartData(); // TELL SLAVE CAN START
  while (reading != 0) { // RETRY
    delay(100);
    reading = sendStartData();
  }
  if (DEBUG)
    Serial.println("\'START\' successfully send");

  setDataMsg(); // reset buffer to send readings
  if (DEBUG)
    Serial.println("Will start loop");
  digitalWrite(LED_BUILTIN, HIGH); // hardware debug
  delay(50);
}

void loop() {
  doExecuteTemperature();
  doExecuteLight();
  doExecutePotentiometer();

  sendDataOnce();
}

void doExecuteTemperature() {

  // getting the voltage reading from the temperature sensor
  int tempVal = analogRead(tempPin);
  float tempVoltage = ((tempVal / 1024.0) * 5.0) - 0.5;
  tempVal = tempVoltage * 100; // final calculation

  tempReads[indexTemp++] = tempVal;
  if (indexTemp >= SAMPLE_TEMP) // check index var
    indexTemp = 0;

  bufferData[TEMP_INDEX] = getTempAverage(); // average of last values

  delay(10); // to process
}

void doExecuteLight() {

  // check bondaries with our standarts, for errors correction
  int lightVal = analogRead(lightPin);
  if (lightVal < sensorMin)
    lightVal = sensorMin;
  if (lightVal > sensorMax)
    lightVal = sensorMax;

  lightReads[indexLight++] = lightVal;
  if (indexLight >= SAMPLE_LIGHT)  // check index var
    indexLight = 0;

  bufferData[LIGHT_INDEX] = getLightAverageMapped(); // average of last values

  delay(10); // to process
}

void doExecutePotentiometer() {
  bufferData[POT_INDEX] = getAnalogMap(analogRead(potPin)); // the value

  delay(10); // to process
}

// Simple map function between range [0-255]
int getAnalogMap(int value) {
  return ((255.0 / 1023.0) * value); // use float calculation with *.0
}

// Get the average temperature in the array [0-255]
int getTempAverage() {
  int res = 0;

  for (int i = 0; i < SAMPLE_TEMP; i++)
    res += tempReads[i];

  return (res / SAMPLE_TEMP); // the average
}

// Get the average light read in the array [0-255]
int getLightAverageMapped() {
  int res = 0;

  for (int i = 0; i < SAMPLE_LIGHT; i++)
    res += lightReads[i];

  res = (res / SAMPLE_LIGHT); // get the average

  return map(res, sensorMin, sensorMax, 0, 255); // map the average
}

// Function used to send all the data once
// If no new values updated sends the old ones
void sendDataOnce() {
  int err = -1;

  Wire.beginTransmission(SLAVE_ADDR); // transmit to device SLAVE_ADDR
  Wire.write(bufferData, MAX_BUFFER); // sends buffer with size

  err = Wire.endTransmission(); // endTransmission return value
  if (err != 0) {
    // ERROR not really important
    // slaves still have the old values
  }
  if (DEBUG) {
    Serial.print("Values: ");
    printByteAsString();
    Serial.print("With return call: ");
    Serial.println(err);
  }

  delay(100); // to process request
}

// Auxiliary function to set the 'START' command
void setStartMsg() {
  memset(bufferData, 0, MAX_BUFFER);
  bufferData[0] = 'S';
  bufferData[1] = 'T';
  bufferData[2] = 'A';
  bufferData[3] = 'R';
  bufferData[4] = 'T';
}

// Send the 'START' command and return it's value
int sendStartData() {
  Wire.beginTransmission(SLAVE_ADDR); // transmit to device SLAVE_ADDR
  Wire.write(bufferData, MAX_BUFFER); // sends buffer with size

  return Wire.endTransmission(); // endTransmission return value
}

// Auxiliary function to set the 'T', 'L' and 'P'
void setDataMsg() {
  memset(bufferData, 0, MAX_BUFFER);
  bufferData[0] = 'T'; // the first value is temperature
  // this mean index 1 is temperature value
  bufferData[2] = 'L'; // the second value is light
  // this mean index 3 is light value
  bufferData[4] = 'P';  // the last value is pot
  // this mean index 5 is pot value
}

void printByteAsString() {
  if (DEBUG) {
    for (int i = 0; i < MAX_BUFFER; i++) {
      Serial.print(bufferData[i]);
      Serial.print(" | ");
    }
    Serial.println();
  }
}

