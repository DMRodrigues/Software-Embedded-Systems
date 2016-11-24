#include <Wire.h>

#define DEBUG 0

#define RED_LED   2 /* off state */
#define GREEN_LED 3 /* on state */
#define BUS_LED   4
#define BUTTON    5

#define TL_SIZE 2 /* we have two traffic light */
#define TL1_ADDR 8
#define TL2_ADDR 9

#define MIN_TIME 4000
#define MAX_TIME 10000

#define MAX_BUFFER 8
byte data_out[MAX_BUFFER]; /* buffer to send data to slaves */
byte data_in[MAX_BUFFER]; /* buffer to receive data from slaves */

byte buff_ping[MAX_BUFFER]; /* buffer to be used with ping commands */
byte ping_res[TL_SIZE]; /* save the last result from pings to perform shutdown */
unsigned int nok_TL1; /* counter to count cycles of TL1 is nok */
unsigned int nok_TL2; /* counter to count cycles of TL2 is nok */

int button_state;
int button_state_old;

int controller_state = 0; /* 0-OFF, 1-ON */

const int potPin = A0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  if (DEBUG)
    Serial.begin(115200);

  /* init leds and buton */
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  pinMode(BUS_LED, OUTPUT);
  digitalWrite(BUS_LED, LOW);
  pinMode(BUTTON, INPUT);

  memset(data_out, 0, MAX_BUFFER);
  memset(data_in, 0, MAX_BUFFER);
  memset(buff_ping, 0, MAX_BUFFER);
  memset(ping_res, 0, TL_SIZE);

  /* start the traffic lights */
  make_off_msg();
  broadcast_data();

  if (DEBUG)
    Serial.println("Will start loop");
  digitalWrite(LED_BUILTIN, HIGH); // hardware debug
}

void loop() {
  do_check_state();

  /* the magic happens here */
}


void do_check_state() {
  button_state = digitalRead(BUTTON);
  delay(50); // stable the read
  if (button_state != button_state_old) {
    if (button_state == HIGH)
      controller_state = 1;
    else
      controller_state = 0;
  }
}


void send_data(int slave_add) {
  digitalWrite(BUS_LED, HIGH);
  Wire.beginTransmission(slave_add); // transmit to device SLAVE_ADDR
  Wire.write(data_out, MAX_BUFFER); // sends buffer with size

  byte err = Wire.endTransmission(); // endTransmission return value
  if (err != 0) {
    // Worry later!
  }
  if (DEBUG) {
    Serial.print("Sended: |");
    printByteArrayAsString(data_out);
    Serial.print("| Return call: ");
    Serial.println(err);
  }
  digitalWrite(BUS_LED, LOW);
  delay(100); // to process request
}


void broadcast_data() {
  digitalWrite(BUS_LED, HIGH);
  Wire.beginTransmission(0);  // broadcast to all
  Wire.write(data_out, MAX_BUFFER); // sends buffer with size
  Wire.endTransmission(); // endTransmission return value

  if (DEBUG) {
    Serial.print("Broadcasting: ");
    printByteArrayAsString(data_out);
  }
  digitalWrite(BUS_LED, LOW);
  delay(100); // to process request
}


void make_ping(int slave_add) {
  int i = 0;
  if (DEBUG)
    Serial.println("Requesting ping");

  digitalWrite(BUS_LED, HIGH);
  Wire.requestFrom(slave_add, MAX_BUFFER); // request MAX_BUFFER bytes from slave
  delay(10); // more than enough time for all the data to come in
  while (Wire.available()) { // slave may send less than requested
    buff_ping[i++] = Wire.read(); // receive a byte each
  }
  digitalWrite(BUS_LED, LOW);
  if (DEBUG) {
    Serial.println("Ping value: ");
    printByteArrayAsString(buff_ping);
  }

  /* see if ACK answer */
  int res_id = extract_ping_id(buff_ping, i);

  map_tl_ping(slave_add, res_id);

  if (DEBUG) {
    Serial.print("Slave answer: ");
    Serial.println(res_id);
  }
}


void map_tl_ping(int sa, int rid) {
  switch (sa) {
    case TL1_ADDR:
      ping_res[0] = rid;
      if (rid == -1)
        nok_TL1++; /* to count the cycles is down */
      break;
    case TL2_ADDR:
      ping_res[1] = rid;
      if (rid == -1)
        nok_TL2++; /* to count the cycles is down */
      break;
    default:
      break;
  }
}


void proccess_receive() {
  int i = 0;
  if (DEBUG)
    Serial.print("proccess_received: ");

  if (Wire.available()) {
    while (Wire.available()) { // slave may send less than requested
      data_in[i++] = Wire.read(); // receive a byte each
    }
    do_verify_command();

    if (DEBUG)
      printByteArrayAsString(data_in);
  }
}

void do_verify_command() {
  if (DEBUG)
    Serial.print("do_verify_command: ");

  /* verify what type of command received:
      ACK x
      RED x
  */

  memset(data_in, 0, MAX_BUFFER); /* clean in the end */
}


int extract_ping_id(byte* s, int s_size) {
  int res = -1;
  if (DEBUG)
    Serial.print("extract_traffic_id: ");

  for (int i = 0; i < s_size; i++) {

    // the first we want
    if (s[i] == 'P') {

      // if the rest data can be there
      if ((i + 5) <= s_size) {

        // aren the others really there ?
        if ( (s[i + 1] == 'I') && (s[i + 2] == 'N') && (s[i + 3] == 'G') ) {

          // safe to extract
          res = s[i + 5];
        }
      }
    }
  }
  if (DEBUG)
    Serial.println(res);
  return res;
}


int extract_ack_id(byte* s, int s_size) {
  int res = -1;
  if (DEBUG)
    Serial.print("extract_ack_id: ");

  for (int i = 0; i < s_size; i++) {

    // the first we want
    if (s[i] == 'A') {

      // if the rest data can be there
      if ((i + 4) <= s_size) {

        // aren the others really there ?
        if ( (s[i + 1] == 'C') && (s[i + 2] == 'K') ) {

          // safe to extract
          res = s[i + 4];
        }
      }
    }
  }
  if (DEBUG)
    Serial.println(res);
  return res;
}


int extract_red_id(byte* s, int s_size) {
  int res = -1;
  if (DEBUG)
    Serial.print("extract_red_id: ");

  for (int i = 0; i < s_size; i++) {

    // the first we want
    if (s[i] == 'R') {

      // if the rest data can be there
      if ((i + 4) <= s_size) {

        // aren the others really there ?
        if ( (s[i + 1] == 'E') && (s[i + 2] == 'D') ) {

          // safe to extract
          res = s[i + 4];
        }
      }
    }
  }
  if (DEBUG)
    Serial.println(res);
  return res;
}


/* message creation area down here */
void make_on_msg(int id) {
  memset(data_out, 0, MAX_BUFFER);
  data_out[0] = 'O';
  data_out[1] = 'N';
  data_out[2] = ' ';
  data_out[3] = lowByte(id); /* extract the byte */
}
void make_off_msg() {
  memset(data_out, 0, MAX_BUFFER);
  data_out[0] = 'O';
  data_out[1] = 'F';
  data_out[2] = 'F';
}
void make_time_msg(int time_ms) {
  memset(data_out, 0, MAX_BUFFER);
  data_out[0] = 'T';
  data_out[1] = 'I';
  data_out[2] = 'M';
  data_out[3] = 'E';
  data_out[4] = ' ';
  /* put time in milliseconds */
}
void make_ping_msg() {
  memset(data_out, 0, MAX_BUFFER);
  data_out[0] = 'P';
  data_out[1] = 'I';
  data_out[2] = 'N';
  data_out[3] = 'G';
}
void make_ack_msg() {
  memset(data_out, 0, MAX_BUFFER);
  data_out[0] = 'A';
  data_out[1] = 'C';
  data_out[2] = 'K';
}

void printByteArrayAsString(byte* data) {
  if (DEBUG) {
    for (int i = 0; i < MAX_BUFFER; i++) {
      Serial.print(data[i]);
      Serial.print(" | ");
    }
    Serial.println();
  }
}
