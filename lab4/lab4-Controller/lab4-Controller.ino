#include <Wire.h>

#define DEBUG 1

#define GREEN_LED 2 /* on state */
#define RED_LED   3 /* off state */
#define BUS_LED   4
#define BUTTON_SW 5

#define RED    1
#define YELLOW 2
#define GREEN  3

#define MY_ADDR  8
#define TL_SIZE  2 /* we have two traffic light */
#define TL1_ADDR 9
#define TL2_ADDR 10

#define MIN_TIME 4000
#define MAX_TIME 10000
#define MY_DIVISION (255.0 / 1023.0)

#define MAX_BUFFER 8
byte data_out[MAX_BUFFER]; /* buffer to send data to slaves */

byte data_in1[MAX_BUFFER]; /* buffer to receive data from slaves */
int in1_flag = 0;
int in1_howMany = 0;

byte data_in2[MAX_BUFFER]; /* buffer to receive data from slaves */
int in2_flag = 0;
int in2_howMany = 0;

byte data_in3[MAX_BUFFER]; /* buffer to receive data from slaves */
int in3_flag = 0;
int in3_howMany = 0;

byte buff_ping[MAX_BUFFER]; /* buffer to be used with ping commands */
byte ping_res[TL_SIZE]; /* count cycles of TLs is nok, to perform shutdown */
int wait_tl1; /* say I waiting answer */
int wait_tl2;

int button_state;
int lastButtonState = HIGH;

int controller_state = 0; /* 0-OFF, 1-ON */
int internal = 0;
unsigned long previousCycle;

#define POT_ERROR 3
const int potPin = A0;
int pot_val;
int last_pot;
int val_sent = 0;
const unsigned long potInterval = 500;
unsigned long potPrevious;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  if (DEBUG)
    Serial.begin(115200);

  Wire.begin(MY_ADDR);
  Wire.onReceive(proccess_receive);

  /* init leds and buton */
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  pinMode(BUS_LED, OUTPUT);
  digitalWrite(BUS_LED, LOW);
  pinMode(BUTTON_SW, INPUT_PULLUP);

  previousCycle = millis();
  potPrevious = previousCycle;

  memset(data_out, 0, MAX_BUFFER);
  memset(data_in1, 0, MAX_BUFFER);
  memset(data_in2, 0, MAX_BUFFER);
  memset(buff_ping, 0, MAX_BUFFER);
  memset(ping_res, 0, TL_SIZE);

  if (DEBUG)
    Serial.println("Will start loop");
  digitalWrite(LED_BUILTIN, HIGH); // hardware debug
}

void loop() {
  check_state();

  if (controller_state == 0) {
    /* OFF */
    if (internal) {
      if (DEBUG)
        Serial.println("Controller Off");
      make_off_msg();
      send_data(TL1_ADDR, 0);
      Serial.print("\t");
      send_data(TL2_ADDR, 0);
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      internal = 0; // no need to repeat
    }
    /* more OFF state things to do */

  } else {
    /* ON */
    if (internal) {
      if (DEBUG)
        Serial.println("Controller On");
      restart_routine();
      internal = 0; // no need to repeat

      /* send red to one */
      make_on_msg(RED);
      send_data(TL1_ADDR, &wait_tl1);

      /* and green to other */
      make_on_msg(GREEN);
      send_data(TL2_ADDR, &wait_tl2);
    }

    execute_pot(); /* verify if the reading changed to update TL | worst cae: 60ms */

    do_verify_data();

    check_cycles(); /* verify if any TL needs ping */
    check_tl_dead(); /* verify if any TL is dead to shutdown */

    do_verify_data(); /* must be executed because whe need buffer processed and new info */
  }

}


/* -------------------------------------------------------------------------------------- */
/* say to controller must change to off                                                   */
void set_off() {
  controller_state = 0;
  internal = 1;
}

/* reset all variables to be used again                                                   */
void restart_routine() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  ping_res[0] = 0;
  ping_res[1] = 0;
  wait_tl1 = 0;
  wait_tl2 = 0;
  last_pot = 0;
  val_sent = 0;
  previousCycle = millis();
  potPrevious = previousCycle;
  memset(data_out, 0, MAX_BUFFER);
  memset(data_in1, 0, MAX_BUFFER);
  memset(data_in2, 0, MAX_BUFFER);
  memset(buff_ping, 0, MAX_BUFFER);
  memset(ping_res, 0, TL_SIZE);
}

/* -------------------------------------------------------------------------------------- */
void check_state() {
  button_state = digitalRead(BUTTON_SW);
  // improve the read
  if (button_state == LOW) {
    delay(20); // stable the read
    controller_state = !controller_state;
    internal = 1;
    while (digitalRead(BUTTON_SW) != HIGH) {
      delay(20);  // until released
    }
    Serial.print("check_state: ");
    Serial.println(controller_state);
  }
}


/* -------------------------------------------------------------------------------------- */
/* when the potentiometer values change send to TL                                        */
void execute_pot() {
  int val = analogRead(potPin); // the value

  if ((val - last_pot > POT_ERROR) || (last_pot - val > POT_ERROR)) {
    last_pot = val;
    potPrevious = millis();
    val_sent = 0;
    if (DEBUG)
      Serial.println("===============> NEW READ");
  }
  /* did the pot stop rotating for half second ? */
  if ((millis() - potPrevious >= potInterval) && !val_sent) {
    if (DEBUG)
      Serial.print("execute_pot: ");

    pot_val = getAnalogMap(val); // get time to be used to count cycle
    if (DEBUG) {
      Serial.print(pot_val);
      Serial.print(" | ");
    }
    // make_time_msg(getSimpleAnalogMap(val));
    make_time_msg_mapped(pot_val);

    send_data(TL1_ADDR, &wait_tl1);
    Serial.print("\t");
    send_data(TL2_ADDR, &wait_tl2);
    delay(10); // to process
    val_sent = 1;
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
    Serial.print("Sent: | ");
    printByteArrayAsString(data_out);
    Serial.print("Return call: ");
    Serial.println(err);
  }
  if ( (*to_inc) != -1)
    (*to_inc)++;

  digitalWrite(BUS_LED, LOW);
  delay(50); // to process request
}

/* DO NOT USE */
void broadcast_data() {
  digitalWrite(BUS_LED, HIGH);
  Wire.beginTransmission(0);  // broadcast to all
  Wire.write(data_out, MAX_BUFFER); // sends buffer with size
  Wire.endTransmission(); // endTransmission return value

  if (DEBUG) {
    Serial.print("Broadcasting: ");
    printByteArrayAsStringln(data_out);
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
    Serial.print("Ping value: ");
    printByteArrayAsString(buff_ping);
  }

  int res_id = extract_ack_id(buff_ping, i); /* see valid ACK answer */

  map_tl_ping(slave_add, res_id);

  memset(buff_ping, 0, MAX_BUFFER);
  if (DEBUG) {
    Serial.print("Slave answer: ");
    Serial.println(res_id);
  }
}

/* callback when data is received from slave, stores on the first available buffer        */
void proccess_receive(int i) {
  digitalWrite(BUS_LED, HIGH);

  if (in1_flag == 0) {
    Wire.readBytes(data_in1, i);
    in1_howMany = i;
    in1_flag = 1;

  } else if (in2_flag == 0) {
    Wire.readBytes(data_in2, i);
    in2_howMany = i;
    in2_flag = 1;

  } else if (in3_flag == 0) {
    Wire.readBytes(data_in3, i);
    in3_howMany = i;
    in3_flag = 1;

  } else {
    // ERROR
    // add more buffers to prevent data loss
  }
  digitalWrite(BUS_LED, LOW);
}


/* -------------------------------------------------------------------------------------- */
/* simple map to count when any TL fails 2 cycles, meaning no ACK received                */
/* either normal answer or ping                                                           */
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
        Serial.print(" | ");
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
        Serial.print(" | ");
        Serial.println(wait_tl2);
      }
      break;
    default:
      break;
  }
}


/* -------------------------------------------------------------------------------------- */
/* if any TL is dead 2 cycles then begin shutdown                                         */
void check_tl_dead() {
  if ((ping_res[0] >= 2) || (ping_res[1] >= 2)) {
    if (DEBUG)
      Serial.println("HAVE ONE TL DEAD");
    set_off();
  }
}


/* -------------------------------------------------------------------------------------- */
/* routine executed when every cycle is done to verify if ACK is missing or not           */
/* if ACK is missing then perform ping to make sure is dead or alive                      */
void check_cycles() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousCycle >= ((unsigned long) pot_val)) {
    previousCycle = currentMillis;  // save the last time of cycle
    if (DEBUG)
      Serial.println("check_cycles");

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
/* normal routine to verify if there are any data in buffers to be extracted              */
void do_verify_data() {
  if (in1_flag == 1) {

    // verify the data in this buffer
    verify_command(data_in1, in1_howMany);

    // clear
    memset(data_in1, 0, MAX_BUFFER);
    in1_flag = 0;
  }
  if (in2_flag == 1) {

    // verify the data in this buffer
    verify_command(data_in2, in2_howMany);

    // clear
    memset(data_in2, 0, MAX_BUFFER);
    in2_flag = 0;
  }
  if (in3_flag == 1) {

    // verify the data in this buffer
    verify_command(data_in3, in3_howMany);

    // clear
    memset(data_in3, 0, MAX_BUFFER);
    in3_flag = 0;
  }
  // NO MORE BUFFERS TO PROCCESS
}

/* to get the type of command received and reply                                          */
void verify_command(byte * s, int s_size) {
  if (DEBUG)
    Serial.print("verify_command | ");
  int id;
  /* verify what type of command received:
    PING x -> send ACK
    ACK x  -> late ACK received, process it
    RED x  -> adapt, send yellow to other TL
  */
  for (int i = 0; i < s_size; i++) {
    if (s[i] == 'P') {
      if (DEBUG)
        Serial.print("PING | ");
      id = extract_ping_id(s, i);
      if (id != -1) {
        make_ack_msg();
        send_data(id, 0); // dont care for ack
      }
      else {
        // bad PING request
      }
      break;
    }
    else if (s[i] == 'A') {
      id = extract_ack_id(s, i);
      if (id != -1) {
        if (DEBUG)
          Serial.print("ACK | ");
        map_tl_ping(id, id);
      }
      else {
        // bad ACK received
      }
      break;
    }
    else if (s[i] == 'R') {
      id = extract_red_id(s, i);
      if (id != -1) {
        if (DEBUG)
          Serial.print("RED | ");
        make_grn_msg();
        if (id == TL1_ADDR)
          send_data(TL2_ADDR, &wait_tl2);
        else
          send_data(TL1_ADDR, &wait_tl1);
      }
      else {
        // bad RED received, shutdown?
      }
      break;
    }
    else {
      if (DEBUG)
        Serial.print("ERROR");
    }
  }
}


/* -------------------------------------------------------------------------------------- */
int extract_ping_id(byte * s, int i) {
  int res = -1;
  if (DEBUG)
    Serial.print("extract_ping_id: ");
  /*
    [PING x] has 6 letters, buffer has space for 8. if
    it starts on i > 2, then we won't be able to
    extract a ping command from the buffer
  */
  if (i > 2)
    return res;

  if (s[i + 1] != 'I')
    return res;
  if (s[i + 2] != 'N')
    return res;
  if (s[i + 3] != 'G')
    return res;
  if (s[i + 4] != ' ')
    return res;

  res = s[i + 5]; // safe to extract
  if (DEBUG)
    Serial.println(res);
  return res;
}

int extract_ack_id(byte * s, int i) {
  int res = -1;
  if (DEBUG)
    Serial.print("extract_ack_id: ");
  /*
    [ACK x] has 5 letters, buffer has space for 8. if
    it starts on i > 3, then we won't be able to
    extract an ack command from the buffer
  */
  if (i > 3)
    return res;

  if (s[i + 1] != 'C')
    return res;
  if (s[i + 2] != 'K')
    return res;
  if (s[i + 3] != ' ')
    return res;

  res = s[i + 4]; // safe to extract
  if (DEBUG)
    Serial.println(res);
  return res;
}

int extract_red_id(byte * s, int i) {
  int res = -1;
  if (DEBUG)
    Serial.print("extract_red_id: ");
  /*
    [RED x] has 5 letters, buffer has space for 8. if
    it starts on i > 3, then we won't be able to
    extract an red command from the buffer
  */
  if (i > 3)
    return res;

  if (s[i + 1] != 'E')
    return res;
  if (s[i + 2] != 'D')
    return res;
  if (s[i + 3] != ' ')
    return res;

  res = s[i + 4]; // safe to extract
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
void make_grn_msg() {
  memset(data_out, 0, MAX_BUFFER);
  data_out[0] = 'G';
  data_out[1] = 'R';
  data_out[2] = 'N';
}


/* -------------------------------------------------------------------------------------- */
void printByteArrayAsString(byte * data) {
  if (DEBUG) { /* double check, better safe than sorry */
    for (int i = 0; i < MAX_BUFFER; i++) {
      if (data[i] == 32) // the space
        Serial.print("0");
      else if (data[i] > 64 && data[i] < 123)
        Serial.write(data[i]);
      else
        Serial.print(data[i]);
      Serial.print(" | ");
    }
  }
}

void printByteArrayAsStringln(byte * data) {
  if (DEBUG) { /* double check, better safe than sorry */
    for (int i = 0; i < MAX_BUFFER; i++) {
      if (data[i] == 32) // the space
        Serial.print("0");
      else if (data[i] > 64 && data[i] < 123)
        Serial.write(data[i]);
      else
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

