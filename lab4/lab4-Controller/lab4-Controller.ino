#include <Wire.h>

#define DEBUG 0

#define RED_LED   2 /* off state */
#define GREEN_LED 3 /* on state */
#define BUS_LED   4
#define BUTTON_SW 5

#define MY_ADDR  8
#define TL_SIZE  2 /* we have two traffic light */
#define TL1_ADDR 9
#define TL2_ADDR 10

#define MIN_TIME 4000
#define MAX_TIME 10000
#define MY_DIVISION (255.0 / 1023.0)

#define MAX_BUFFER 8
byte data_out[MAX_BUFFER]; /* buffer to send data to slaves */
byte data_in[MAX_BUFFER]; /* buffer to receive data from slaves */

byte buff_ping[MAX_BUFFER]; /* buffer to be used with ping commands */
byte ping_res[TL_SIZE]; /* count cycles of TLs is nok, to perform shutdown */
int wait_tl1; /* say I waiting answer */
int wait_tl2;

int button_state;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int controller_state = 0; /* 0-OFF, 1-ON */
int internal = 1;

#define POT_ERROR 5
const int potPin = A0;
int pot_val;
int last_pot;
unsigned long previousMillis = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  if (DEBUG)
    Serial.begin(115200);

  Wire.begin(MY_ADDR);

  /* init leds and buton */
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  pinMode(BUS_LED, OUTPUT);
  digitalWrite(BUS_LED, LOW);
  pinMode(BUTTON_SW, INPUT_PULLUP);

  memset(data_out, 0, MAX_BUFFER);
  memset(data_in, 0, MAX_BUFFER);
  memset(buff_ping, 0, MAX_BUFFER);
  memset(ping_res, 0, TL_SIZE);

  if (DEBUG)
    Serial.println("Starting TLs");

  /* start the traffic lights */
  make_off_msg();
  send_data(TL1_ADDR, &wait_tl1);
  send_data(TL2_ADDR, &wait_tl2);


  if (DEBUG)
    Serial.println("Will start loop");
  digitalWrite(LED_BUILTIN, HIGH); // hardware debug
}

void loop() {
  check_state();

  /* OFF */
  if (controller_state == 0) {


    /* ON */
  } else {
    if (internal) {
      make_off_msg();
      send_data(TL1_ADDR, &wait_tl1);
      send_data(TL2_ADDR, &wait_tl2);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      internal = 0; // no need to repeat
    }
  }

}


/* -------------------------------------------------------------------------------------- */
void set_off() {
  controller_state = 0;
  internal = 1;
}


/* -------------------------------------------------------------------------------------- */
void check_state() {
  button_state = digitalRead(BUTTON_SW);
  delay(10); // stable the read
  // improve the read
  if (button_state == LOW) {
    controller_state = !controller_state;
    internal = 1;
    while (digitalRead(BUTTON_SW) != HIGH); // until released
    Serial.print("check_state: ");
    Serial.println(controller_state);
  }
}

/* TO TEST */
void check_state_debounce() {
  int reading = digitalRead(BUTTON_SW);

  if (reading != lastButtonState)
    lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != lastButtonState) {
      lastButtonState = reading;

      if (reading == LOW) {
        controller_state = !controller_state;
        internal = 1;
        Serial.print("check_state: ");
        Serial.println(controller_state);
      }
    }
  }
}

void execute_pot() {
  if (DEBUG)
    Serial.print("execute_pot: ");
  int val = analogRead(potPin); // the value

  if ((val - last_pot > POT_ERROR) || (last_pot - val > POT_ERROR)) {
    last_pot = val;

    // get time for millis()
    pot_val = getAnalogMap(val);

    make_time_msg(getSimpleAnalogMap(val));
    // make_time_msg_mapped(pot_val);

    send_data(TL1_ADDR, &wait_tl1);
    send_data(TL2_ADDR, &wait_tl2);
    delay(10); // to process
  }
}

/* -------------------------------------------------------------------------------------- */
/* I2C auxiliary functions down here                                                      */
void send_data(int slave_add, int* to_inc) {
  digitalWrite(BUS_LED, HIGH);
  Wire.beginTransmission(slave_add); // transmit to device SLAVE_ADDR
  Wire.write(data_out, MAX_BUFFER); // sends buffer with size

  byte err = Wire.endTransmission(); // endTransmission return value
  if (err != 0) {
    // Worry later!
  }
  if (DEBUG) {
    Serial.print("Sent: |");
    printByteArrayAsString(data_out);
    Serial.print("| Return call: ");
    Serial.println(err);
  }
  (*to_inc)++;
  digitalWrite(BUS_LED, LOW);
  delay(50); // to process request
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
  delay(50); // to process request
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
  int res_id = extract_ack_id(buff_ping, i);

  map_tl_ping(slave_add, res_id);

  if (DEBUG) {
    Serial.print("Slave answer: ");
    Serial.println(res_id);
  }
}

void check_proccess_receive() {
  int i = 0;
  if (DEBUG)
    Serial.print("check_proccess_receive: ");

  if (Wire.available()) {
    while (Wire.available()) { // slave may send less than requested
      data_in[i++] = Wire.read(); // receive a byte each
    }
    verify_command(data_in, i);

    if (DEBUG)
      printByteArrayAsString(data_in);
  }
}


/* -------------------------------------------------------------------------------------- */
void map_tl_ping(int sa, int rid) {
  switch (sa) {
    case TL1_ADDR:
      if (rid == -1)
        ping_res[0] = ping_res[0] + 1; /* to count the cycles is down */
      else {
        wait_tl1--;
        if (wait_tl1 <= 0) {
          wait_tl1 = 0;
          ping_res[0] = 0;
        }
      }
      if (DEBUG) {
        Serial.print("map_tl_ping | TL1 | ");
        Serial.print(ping_res[0]);
        Serial.print("|");
        Serial.println(wait_tl1);
      }
      break;
    case TL2_ADDR:
      if (rid == -1)
        ping_res[1] = ping_res[1] + 1; /* to count the cycles is down */
      else {
        wait_tl2--;
        if (wait_tl2 <= 0) {
          wait_tl2 = 0;
          ping_res[1] = 0;
        }
      }
      if (DEBUG) {
        Serial.print("map_tl_ping | TL2 | ");
        Serial.print(ping_res[1]);
        Serial.print("|");
        Serial.println(wait_tl1);
      }
      break;
    default:
      break;
  }
}


/* -------------------------------------------------------------------------------------- */
int check_tl_dead() {
  /* if tl is dead 2 cycles then begin shutdown */
  if ((ping_res[0] >= 2) || (ping_res[1] >= 2)) {
    if (DEBUG)
      Serial.println("HAVE ONE TL DEAD");
    set_off();
  }
}


/* -------------------------------------------------------------------------------------- */
void check_cycles() {
  if (DEBUG)
    Serial.print("check_cycles | ");

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= pot_val) {
    previousMillis = currentMillis;  // save the last time of cycle

    // do we need to ping ?
    if (ping_res[0] > 0) {
      if (DEBUG)
        Serial.print("TL1 | ");
      make_ping(TL1_ADDR);
    }
    if (ping_res[1] > 0) {
      if (DEBUG)
        Serial.print("TL2 | ");
      make_ping(TL2_ADDR);
    }
  }
}


/* -------------------------------------------------------------------------------------- */
void verify_command(byte * s, int s_size) {
  if (DEBUG)
    Serial.print("verify_command | ");
  /* verify what type of command received:
    PING x -> send ACK
    ACK x  -> this is handled in make_ping
    RED x  -> then adapt to something ?
  */
  int id = extract_ping_id(s, s_size);
  if (id != -1) {
    if (DEBUG)
      Serial.print("PING from:");
    make_ack_msg();
    send_data(id, 0); // dont care for ack
  }
  else {
    id = extract_ack_id(s, s_size);
    if (id != -1) {
      if (DEBUG)
        Serial.print("ACK from:");
      map_tl_ping(id, id);
    }
    else {
      id = extract_red_id(s, s_size);
      if (id != -1) {
        if (DEBUG)
          Serial.print("RED from:");

        make_on_msg(2);
        if (id == TL1_ADDR)
          send_data(TL2_ADDR, &wait_tl2);
        else
          send_data(TL1_ADDR, &wait_tl1);

      }
      else {
        if (DEBUG)
          Serial.print("ERROR");
      }
    }
  }
  if (DEBUG)
    Serial.println(id);

  memset(data_in, 0, MAX_BUFFER); /* clean in the end */
}


/* -------------------------------------------------------------------------------------- */
int extract_ping_id(byte * s, int s_size) {
  int res = -1;
  if (DEBUG)
    Serial.print("extract_ping_id: ");
  /*
    PING occupies 6 (PING X), buffer has space for 8. if
    (i + 5) <= s_size, then we won't be able to
    extract a ping command from the buffer
  */
  for (int i = 0; i < s_size; i++) {

    // the first we want
    if (s[i] == 'P') {

      // if the rest data can be there
      if ((i + 5) <= s_size) {

        // aren the others really there ?
        if ( (s[i + 1] == 'I') && (s[i + 2] == 'N') && (s[i + 3] == 'G') && (s[i + 4] == ' ') ) {

          res = s[i + 5]; // safe to extract
        }
      }
    }
  }
  if (DEBUG)
    Serial.println(res);
  return res;
}

int extract_ack_id(byte * s, int s_size) {
  int res = -1;
  if (DEBUG)
    Serial.print("extract_ack_id: ");
  /*
    ACK occupies 5 (ACK X), buffer has space for 8. if
    (i + 4) <= s_size, then we won't be able to
    extract an ack command from the buffer
  */
  for (int i = 0; i < s_size; i++) {

    // the first we want
    if (s[i] == 'A') {

      // if the rest data can be there
      if ((i + 4) <= s_size) {

        // aren the others really there ?
        if ( (s[i + 1] == 'C') && (s[i + 2] == 'K') && (s[i + 3] == ' ') ) {

          res = s[i + 4]; // safe to extract
        }
      }
    }
  }
  if (DEBUG)
    Serial.println(res);
  return res;
}

int extract_red_id(byte * s, int s_size) {
  int res = -1;
  if (DEBUG)
    Serial.print("extract_red_id: ");
  /*
    RED occupies 5 (RED X), buffer has space for 8. if
    i(i + 4) <= s_size, then we won't be able to
    extract an ack command from the buffer
  */
  for (int i = 0; i < s_size; i++) {

    // the first we want
    if (s[i] == 'R') {

      // if the rest data can be there
      if ((i + 4) <= s_size) {

        // aren the others really there ?
        if ( (s[i + 1] == 'E') && (s[i + 2] == 'D') && (s[i + 3] == ' ') ) {

          res = s[i + 4]; // safe to extract
        }
      }
    }
  }
  if (DEBUG)
    Serial.println(res);
  return res;
}


/* -------------------------------------------------------------------------------------- */
/* message creation area down here                                                        */
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
  data_out[5] = lowByte(time_ms);
  data_out[6] = highByte(time_ms);
}
void make_time_msg_mapped(byte time_ms) {
  memset(data_out, 0, MAX_BUFFER);
  data_out[0] = 'T';
  data_out[1] = 'I';
  data_out[2] = 'M';
  data_out[3] = 'E';
  data_out[4] = ' ';
  data_out[5] = time_ms; /* time is converted [0-255], then TL must convert  */
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


/* -------------------------------------------------------------------------------------- */
void printByteArrayAsString(byte * data) {
  if (DEBUG) { /* double check, better safe than sorry */
    for (int i = 0; i < MAX_BUFFER; i++) {
      Serial.print(data[i]);
      Serial.print(" | ");
    }
    Serial.println();
  }
}


/* -------------------------------------------------------------------------------------- */
/* getSimpleAnalogMap => simple map function between range [0-255]                        */
/* getAnalogMap       => map function between range [MIN_TIME-MAX_TIME]                   */
int getSimpleAnalogMap(int value) {
  return (MY_DIVISION * value); // use float calculation with *.0
}

int getAnalogMap(int value) {
  return map(value, 0, 1023, MIN_TIME, MAX_TIME);
}

