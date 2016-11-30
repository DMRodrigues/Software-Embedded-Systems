#include <Wire.h>

#define DEBUG 1

#define TRAFFIC_LIGHT_ID 9

#define GREEN_CLR 1
#define YELLOW_CLR 2
#define RED_CLR 3

#define GREEN_IN 8
#define YELLOW_IN 9
#define RED_IN 10

#define GREEN_LED 5
#define YELLOW_LED 6
#define RED_LED 7

#define PED_GREEN_LED 3 /* pedestrian traffic light */
#define PED_RED_LED 2 /* pedestrian traffic light */

#define BUTTON 4

#define MASTER_ADDRESS 8

#define MIN_TIME 4000
#define MAX_TIME 10000

#define MAX_BUFFER 8

byte data_out[MAX_BUFFER]; /* buffer to send data to master */
byte data_in[MAX_BUFFER]; /* buffer to receive data from master */
byte buff_ping[MAX_BUFFER]; /* buffer to be used with ping commands */

int cycle_time;
unsigned long previous_millis;

int start_color;
int previous_color;
int current_color;

boolean blinking;
boolean is_yellow; /* when blinking, is the led yellow or not? */
boolean stay_yellow; /* when we receive an ON YELLOW the traffic light should stay yellow until told otherwise */
boolean controller_alive;
boolean pedestrian; /* used to know if there's a pedestrian trying to cross the street */
boolean event_waiting; /* used to know if we have a command waiting to be serviced or not */

boolean green_working;
boolean red_working;
boolean yellow_working;

boolean state_changed;

void setup() {
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(PED_GREEN_LED, OUTPUT);
  pinMode(PED_GREEN_LED, OUTPUT);
  pinMode(GREEN_IN, INPUT);
  pinMode(RED_IN, INPUT);
  pinMode(YELLOW_IN, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  Wire.begin(TRAFFIC_LIGHT_ID);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(send_ack); /* only thing that is requested is the ACK */

  cycle_time = 4000;
  previous_millis = 0;

  blinking = true;
  is_yellow = false;
  stay_yellow = false;
  pedestrian = false;
  controller_alive = true;

  green_working = true;
  red_working = true;
  yellow_working = true;

  state_changed = false;

  previous_color = RED_CLR;
  current_color = RED_CLR;

  if (DEBUG) {
    Serial.begin(115200);
  }
  send_red();
}

void loop() {
  /* if it's blinking or the controller is dead,
     the traffic light should start blinking */
  //check_state();

  //send_ping();
  //delay(5000);
  //send_ack();

  //delay(5000);
  led_working();

  if ((blinking || !controller_alive) && yellow_working) {
    blink_yellow();
  }
  if (event_waiting) {
    event_waiting = false;
    read_command();
  }
  if (pedestrian) {
    pedestrian = false;
    shorten_cycle();
  }
  if (!red_working) {
    blinking = true;
    blink_yellow();
  }
  if (!green_working) {
    blinking = true;
    blink_yellow();
  }
  /* if we received a ON YELLOW, we just stay yellow */
  if (stay_yellow) {
    transition_to_yellow();
  }
  /*normal functioning: keep on changing colors */
  watch_the_cycle();
}

/*
    API: functions to be executed when the
    messages are received via I2C
*/

/*
    to be executed when the traffic light
    receives an ON message
*/
void turn_traffic_light_on(int clr) {
  start_color = clr;
  blinking = false;
  previous_millis = 0;

  if (DEBUG) {
    Serial.print("Start color: ");
    Serial.println(clr);
  }

  if (clr == GREEN_CLR) {
    stay_yellow = false;
    transition_red_to_yellow(); /* make sure it is yellow before green */
    transition_yellow_to_green(); /* turn green */
  }
  else if (clr == YELLOW_CLR) {
    transition_to_yellow();
    stay_yellow = true;
  }
  else if (clr == RED_CLR) {
    stay_yellow = false;
    transition_yellow_to_red();
  }

  send_ack();
}

/*
    to be executed when the traffic light
    receives an OFF message
*/
void turn_traffic_light_off() {
  blinking = true;
  if (DEBUG) {
    //Serial.println("Blinking");
  }
  send_ack();
}

/*
    to be executed when the traffic light
    receives a GRN message
*/
void start_cycle() {
  blinking = false;
  previous_millis = 0;

  if (DEBUG) {
    Serial.println("Starting cycle");
  }
  //current_color = clr;
  send_ack();
}

/*
    to be executed when the traffic light
    receives a TIME message
*/
void cycle_time_reset(int new_time) {
  if (DEBUG) {
    Serial.print("Cycle changed from ");
    Serial.print(cycle_time);
    Serial.print(" to ");
    Serial.println(new_time);
  }
  cycle_time = new_time;
  send_ack();
}

/*
    to be executed when the traffic light
    receives a PING message
*/
void ping() {
  if (DEBUG) {
    Serial.println("Ping received");
  }

  send_ack();
}

/*
    to be executed when the traffic light
    receives an ACK message
*/
void ack() {
  if (DEBUG) {
    Serial.println("ACK received");
  }
  controller_alive = true;
}

/* FUNCTIONS TO SEND MESSAGES TO THE CONTROLLER */

void send_ping() {
  make_ping_msg();
  Wire.beginTransmission(MASTER_ADDRESS);
  Wire.write(data_out, MAX_BUFFER);
  Wire.endTransmission();

  if (DEBUG) {
    Serial.println("Ping sent");
  }
}

void send_ack() {
  if (DEBUG) {
    Serial.println("send ack");
  }
  make_ack_msg();
  //Wire.beginTransmission(MASTER_ADDRESS);
  Wire.write(data_out, MAX_BUFFER);
  //Wire.endTransmission();

  if (DEBUG) {
    Serial.println("Ack sent");
  }
}

void send_red() {
  if (!red_working) {
    return;
  }
  make_red_msg();
  Wire.beginTransmission(MASTER_ADDRESS);
  Wire.write(data_out, MAX_BUFFER);
  Wire.endTransmission();

  if (DEBUG) {
    Serial.println("Red sent");
  }
}

/* PEDESTRIAN FUNCTIONS */

void shorten_cycle()
{
  unsigned long current_millis = millis();

  if ((previous_millis - current_millis) >= (MIN_TIME / 2)) {
    /* it's already in the end of the cycle, we can't shorten it */
    previous_millis = millis(); /* let's just update time */
  }
  else {
    /* the cycle duration will be cut down by half */
    cycle_time = cycle_time / 2;
  }
}

/* TRAFFIC LIGHT NORMAL FUNCTIONING */

void watch_the_cycle() {
  if (blinking) {
    return;
  }

  unsigned long current_millis = millis();

  if (current_color == YELLOW_CLR &&
     ((current_millis - previous_millis) >= 1000)) {
    if (DEBUG) {
      Serial.println("next color");
      Serial.println(current_millis - previous_millis);
    }
    previous_millis = millis();
    if (previous_color == RED_CLR) {
      transition_yellow_to_green();
    }
    else if (previous_color == GREEN_CLR) {
      led_working();
      transition_yellow_to_red();
    }
  }
  
  /* see if we need to change to yellow */
  else if (current_color != YELLOW_CLR &&
          ((current_millis - previous_millis) >= (cycle_time - 1000))) {
    if (DEBUG) {
      Serial.println("to yellow");
      Serial.println((current_millis - previous_millis));
    }
    previous_millis = millis();
    transition_to_yellow();
  }
  /* can transition to next color */

}

void led_working() {
  if (digitalRead(GREEN_IN) == HIGH) {
    green_working = false;
    if (DEBUG) {
      Serial.println("green not working");
    }
  }
  else {
    green_working = true;
  }
  if (digitalRead(RED_IN) == HIGH) {
    red_working = false;
    if (DEBUG) {
      Serial.println("red not working");
    }
  }
  else {
    red_working = true;
  }
  if (digitalRead(YELLOW_IN) == HIGH) {
    yellow_working = false;
    if (DEBUG) {
      Serial.println("yellow not working");
    }
  }
  else {
    yellow_working = true;
  }
}

/* TRAFFIC LIGHT COLOR TRANSITIONS */

void blink_yellow() {
  if (DEBUG) {
    //Serial.println("blinking");
  }
  unsigned long current_millis = millis();

  /* blink in intervals of 1s */
  if (current_millis - previous_millis >= 1000) {
    previous_millis = current_millis;

    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, LOW);
    
    if (is_yellow) {
      digitalWrite(YELLOW_LED, LOW);
      is_yellow = false;
    }
    else {
      digitalWrite(YELLOW_LED, HIGH);
      is_yellow = true;
    }
  }
}

void transition_to_yellow() {
  if (DEBUG) {
    Serial.println("Staying yellow");
  }
  led_working();
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  current_color = YELLOW_CLR;
}

void transition_red_to_yellow() {
  if (DEBUG) {
    Serial.println("Transition from red to yellow");
  }
  led_working();
  digitalWrite(GREEN_LED, LOW); /* just to make sure */
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, HIGH);

  current_color = YELLOW_CLR;
}

void transition_yellow_to_green() {
  if (DEBUG) {
    Serial.println("Transition from yellow to green");
  }
  led_working();
  digitalWrite(RED_LED, LOW); /* just to make sure */
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  previous_color = GREEN_CLR; /* so that we know where to start when
                                 we receive a start cycle message */
  current_color = GREEN_CLR;
}

void transition_green_to_yellow() {
  if (DEBUG) {
    Serial.println("Transition from green to yellow");
  }
  led_working();
  digitalWrite(RED_LED, LOW); /* just to make sure */
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, HIGH);

  current_color = YELLOW_CLR;
}

void transition_yellow_to_red() {
  if (DEBUG) {
    Serial.println("Transition from yellow to red");
  }
  led_working();
  digitalWrite(GREEN_LED, LOW); /* just to make sure */
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  led_working();
  send_red();
  previous_color = RED_CLR; /* so that we know where to start when
                                 we receive a start cycle message */
  current_color = RED_CLR;
}

void pedestrian_to_green() {
  if (DEBUG) {
    Serial.println("Pedestrian going from red to green");
  }
  digitalWrite(PED_RED_LED, LOW);
  digitalWrite(PED_GREEN_LED, HIGH);
}

void pedestrian_to_red() {
  if (DEBUG) {
    Serial.println("Pedestrian going from green to red");
  }
  digitalWrite(PED_GREEN_LED, LOW);
  digitalWrite(PED_RED_LED, HIGH);
}

/* MESSAGES TO SEND */

void make_red_msg() {
  memset(data_out, 0, MAX_BUFFER);
  data_out[0] = 'R';
  data_out[1] = 'E';
  data_out[2] = 'D';
  data_out[3] = ' ';
  data_out[4] = lowByte(TRAFFIC_LIGHT_ID);
}

void make_ping_msg() {
  memset(data_out, 0, MAX_BUFFER);
  data_out[0] = 'P';
  data_out[1] = 'I';
  data_out[2] = 'N';
  data_out[3] = 'G';
  data_out[4] = ' ';
  data_out[5] = lowByte(TRAFFIC_LIGHT_ID);
}

void make_ack_msg() {
  memset(data_out, 0, MAX_BUFFER);
  data_out[0] = 'A';
  data_out[1] = 'C';
  data_out[2] = 'K';
  data_out[3] = ' ';
  data_out[4] = lowByte(TRAFFIC_LIGHT_ID);
}

void printByteArrayAsString(byte * data) {
  if (DEBUG) { /* double check, better safe than sorry */
    for (int i = 0; i < MAX_BUFFER; i++) {
      Serial.print(data[i]);
      Serial.print(" | ");
    }
    Serial.println();
  }
}

/* READING FROM THE WIRE */

void receiveEvent(int howMany) {
  if (DEBUG) {
    Serial.println("Receiving event");
  }

  Wire.readBytes(data_in, howMany);
  if (DEBUG) {
    Serial.print("Received: ");
    printByteArrayAsString(data_in);
  }
  event_waiting = true;
}

/* READING THE COMMANDS */

void read_command() {
  int i;

  if (DEBUG) {
    Serial.println("Reading a command");
  }

  for (i = 0; i < MAX_BUFFER; i++) {
    if (data_in[i] == 'P') {
      read_ping(i);
      break;
    }
    else if (data_in[i] == 'A') {
      read_ack(i);
      break;
    }
    else if (data_in[i] == 'G') {
      read_grn(i);
      break;
    }
    else if (data_in[i] == 'T') {
      read_time(i);
      break;
    }
    else if (data_in[i] == 'O') {
      if (i < MAX_BUFFER - 1) {
        if (data_in[i + 1] == 'N') {
          read_on(i);
          break;
        }
        else if (data_in[i + 1] == 'F') {
          read_off(i);
          break;
        }
      }
    }
  }

  memset(data_in, 0, MAX_BUFFER); /* clean in the end */
}

void read_on(int i) {
  if (DEBUG) {
    Serial.println("Reading on");
  }

  /*
     ON occupies 4 (ON X), buffer has space for 8. if
     it starts on i > 4, then we won't be able to
     extract an on command from the buffer
  */
  if (i > 4) {
    if (DEBUG) {
      Serial.println("i is bigger than 4 so it's impossible to read the command");
    }
    return;
  }

  /* we've already checked data_in[i] and data_in[i + 1] */
  if (data_in[i + 2] != ' ') {
    return;
  }

  int clr = data_in[i + 3];

  turn_traffic_light_on(clr);
}

void read_off(int i) {
  if (DEBUG) {
    Serial.println("Reading off");
  }
  /*
     OFF has 3 letters, buffer has space for 8. if
     it starts on i > 5, then we won't be able to
     extract an off command from the buffer
  */
  if (i > 5) {
    if (DEBUG) {
      Serial.println("i is bigger than 5 so it's impossible to read the command");
    }
    return;
  }

  /* we've already checked data_in[i] and data_in[i + 1] */
  if (data_in[i + 2] != 'F') {
    return;
  }

  turn_traffic_light_off();
}

void read_ping(int i) {
  /*
     data_in[i] contains the first letter,
     we want to read from then on
  */
  if (DEBUG) {
    Serial.println("Reading ping");
  }
  /*
     PING has 4 letters, buffer has space for 8. if
     it starts on i > 4, then we won't be able to
     extract a ping command from the buffer
  */
  if (i > 4) {
    if (DEBUG) {
      Serial.println("i is bigger than 4 so it's impossible to read the command");
    }
    return;
  }

  /* otherwise, it's safe to extract all the letters */
  if (data_in[i + 1] != 'I') {
    return;
  }
  if (data_in[i + 2] != 'N') {
    return;
  }
  if (data_in[i + 3] != 'G') {
    return;
  }

  /* we've reached the end, it's a ping command */
  ping();
}

void read_ack(int i) {
  if (DEBUG) {
    Serial.println("Reading ack");
  }
  /*
     ACK has 3 letters, buffer has space for 8. if
     it starts on i > 5, then we won't be able to
     extract an ack command from the buffer
  */
  if (i > 5) {
    if (DEBUG) {
      Serial.println("i is bigger than 5 so it's impossible to read the command");
    }
    return;
  }

  /* otherwise, it's safe to extract all the letters */
  if (data_in[i + 1] != 'C') {
    return;
  }
  if (data_in[i + 2] != 'K') {
    return;
  }

  /* we've reached the end, it's an ack command */
  ack();
}

void read_time(int i) {
  if (DEBUG) {
    Serial.println("Reading time");
  }
  /*
     TIME occupies 6 (TIME X), buffer has space for 8. if
     it starts on i > 2, then we won't be able to
     extract a time command from the buffer
  */
  if (i > 2) {
    if (DEBUG) {
      Serial.println("i is bigger than 2 so it's impossible to read the command");
    }
    return;
  }

  /* otherwise, it's safe to extract everything */
  if (data_in[i + 1] != 'I') {
    return;
  }
  if (data_in[i + 2] != 'M') {
    return;
  }
  if (data_in[i + 3] != 'E') {
    return;
  }
  if (data_in[i + 4] != ' ') {
    return;
  }

  int new_time = map_time(data_in[i + 5]);

  if (DEBUG) {
    Serial.print("New time: ");
    Serial.println(new_time);
  }

  /* we've reached the end, it's a time command */
  cycle_time_reset(new_time);
}

int map_time(byte value) {
  return map(value, 0, 255, MIN_TIME, MAX_TIME);
}

void read_grn(int i) {
  if (DEBUG) {
    Serial.println("Reading grn");
  }
  /*
     GRN has 3 letters, buffer has space for 8. if
     it starts on i > 5, then we won't be able to
     extract a grn command from the buffer
  */
  if (i > 5) {
    if (DEBUG) {
      Serial.println("i is bigger than 5 so it's impossible to read the command");
    }
    return;
  }

  /* otherwise, it's safe to extract all the letters */
  if (data_in[i + 1] != 'R') {
    return;
  }
  if (data_in[i + 2] != 'N') {
    return;
  }

  /* we've reached the end, it's an ack command */
  start_cycle();
}

