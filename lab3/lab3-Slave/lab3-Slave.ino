#include <Wire.h>

/* Slave:
    starts and blocks until 'START' is received
    After waits until all values are updated with valid data to be used

    In loop, checks the buffers have data to be proccessed and does it
    simple uses global variables to set the LEDs

    WARNING assumes global variables
*/

#define DEBUG 0

#define MY_ADDR 8
#define MAX_BUFFER 16
#define TEMP_MAX  25
#define BLINK_MIN 200
#define BLINK_MAX 2000

#define START_DIM 5
const String startString = "START";
bool is_start = false;

// 2 buffers to receive comm (this app enough)
byte temp_buff1[MAX_BUFFER];
int buff1_howMany = 0;
byte buff1_flag = 0;

byte temp_buff2[MAX_BUFFER];
int buff2_howMany = 0;
byte buff2_flag = 0;

const int tempLed = 2;
int tempVal = -1;

const int lightLed = 3;
int lightVal = -1;

const int potLed = 4;
int potVal = -1;
int potValMapped = 0;
int potLedState = LOW;

unsigned long previousMillis = 0; // last time LED was updated

void setup() {
  if (DEBUG)
    Serial.begin(115200);

  pinMode(tempLed, OUTPUT);
  digitalWrite(tempLed, LOW);
  pinMode(lightLed, OUTPUT);
  digitalWrite(lightLed, LOW);
  pinMode(potLed, OUTPUT);
  digitalWrite(potLed, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // clear buffers
  memset(temp_buff1, 0, MAX_BUFFER);
  memset(temp_buff2, 0, MAX_BUFFER);

  Wire.begin(MY_ADDR); // join i2c bus with address MY_ADDR
  Wire.onReceive(receiveEvent); // register event

  if (DEBUG)
    Serial.println("Registed Wire, waiting \'START\'");

  delay(20); // can sleep for much more time

  // WAIT FOR THE 'START' FROM THE MASTER
  while (!is_start) {

    if (buff1_flag == 1) {
      doVerifyStart(temp_buff1, buff1_howMany);
      memset(temp_buff1, 0, MAX_BUFFER);
      buff1_flag = 0;

      if (is_start) // save the next check
        break;
    }
    if (buff2_flag == 1) {
      doVerifyStart(temp_buff2, buff2_howMany);
      memset(temp_buff2, 0, MAX_BUFFER);
      buff2_flag = 0;
    }
  }

  if (DEBUG)
    Serial.println("\'START\' received");

  while (1) {
    doVerifyData(); // verify if there are buffers to be processed

    if ((tempVal != -1) && (lightVal != -1) && (potVal != -1)) // wait for valid data
      break;
  }

  if (DEBUG)
    Serial.println("First values received, will start loop");

  digitalWrite(LED_BUILTIN, HIGH); // hardware debugt
}


void loop() {

  // verify if there are buffers to be processed
  doVerifyData();

  doExecuteTemperature();
  doExecuteLight();
  doExecutePotentiometer();
}

void doExecuteTemperature() {
  if (tempVal > TEMP_MAX) // simple use tempVal
    digitalWrite(tempLed, HIGH);
  else
    digitalWrite(tempLed, LOW);
}

void doExecuteLight() {
  analogWrite(lightLed, lightVal); // simple use lightVal
}

/* WARNING: potVal receives integer range [0-255] MUST CONVERT */
void doExecutePotentiometer() {

  // re-map the value to [200-2000]
  potValMapped = getAnalogMap(potVal, BLINK_MIN, BLINK_MAX);

  unsigned long currentMillis = millis(); // get current time

  if (currentMillis - previousMillis >= potValMapped) {
    previousMillis = currentMillis; // save last time LED blinked

    // if the LED is off turn it on and vice-versa (toggle)
    potLedState = !potLedState;

    digitalWrite(potLed, potLedState); // set LED with new ledState
  }
}

// function that executes when data is received from master
// stores on the first available buffer
void receiveEvent(int howMany) {

  if (buff1_flag == 0) {
    Wire.readBytes(temp_buff1, howMany);
    buff1_howMany = howMany;
    buff1_flag = 1;

  } else if (buff2_flag == 0) {
    Wire.readBytes(temp_buff2, howMany);
    buff2_howMany = howMany;
    buff2_flag = 1;

  } else {
    // ERROR
    // add more buffers to prevent data loss
  }

}

// auxiliary function to verify the 'START' command
void doVerifyStart(byte* s, int s_size) {
  if (s_size >= START_DIM) {
    if (contains_sequence(s, s_size, startString, START_DIM))
      is_start = true; // we can start
  }
}

// can be time comsuming, if toSearch is too big
// 1 if toFind exists in toSearch, 0 otherwise
byte contains_sequence(byte* toSearch, int toSearchSize, String toFind, int toFindSize) {
  int i, j;

  // run for possible windows cases until toSearch length ends
  for (i = 0; i <= toSearchSize - toFindSize; i++) {
    byte allSame = 1;

    // compare with toFind PATTERN
    for (j = 0; j < toFindSize; j++) {
      if (toSearch[i + j] != toFind[j]) {
        allSame = 0; // retry
        break;
      }
    }
    if (allSame == 1) // we have it
      return 1;
  }
  return 0;
}

// verify the buffers and if so proccess it's data
// like round-robin fashion
void doVerifyData() {

  if (buff1_flag == 1) {

    // verify the data in this buffer
    getData(temp_buff1, buff1_howMany);

    // clear
    memset(temp_buff1, 0, buff1_howMany);
    //buff1_howMany = 0;
    buff1_flag = 0;
  }
  if (buff2_flag == 1) {

    // verify the data in this buffer
    getData(temp_buff2, buff2_howMany);

    // clear
    memset(temp_buff2, 0, buff2_howMany);
    //buff2_howMany = 0;
    buff2_flag = 0;
  }
  // NO MORE BUFFERS TO PROCCESS
}

// verify if we have valid data to read, if so update values to the new
void getData(byte* s, int s_size) {

  if (DEBUG)
    Serial.print("Going to proccess: ");

  // first find the index of first data => 'T'
  // from there check if the rest of size has all the values => 'L' and 'P'
  // if so extract the data, else keep old values
  // OTHER APPROACH WOULD BE UPDATE SINGLE VALUES IF THEY WERE AVAILABLE
  for (int i = 0; i < s_size; i++) {

    // the first we want
    if (s[i] == 'T') {

      // if the rest data can be there
      if ((i + 5) <= s_size) {

        // the others are really there ?
        if ( (s[i + 2] == 'L') && (s[i + 4] == 'P') ) {
          tempVal = s[i + 1];
          lightVal = s[i + 3];
          potVal = s[i + 5];

          // We could check if the new values were in range and other checks
          if (DEBUG)
            printByteAsString();
        }
      }
    }
  }
}

// Simple map function between minValue and maxValue
int getAnalogMap(int value, int minValue, int maxValue) {
  float tempRes = ((maxValue - minValue) / 1023.0); // keep it float
  return ((tempRes * value) + minValue);
}

void printByteAsString() {
  if (DEBUG) {
    Serial.print("T: ");
    Serial.print(tempVal);
    Serial.print(", L: ");
    Serial.print(lightVal);
    Serial.print(", P: ");
    Serial.print(potVal);
    Serial.println();
  }
}

