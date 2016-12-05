#include <Wire.h>

#define DEBUG 1

#define TRAFFIC_LIGHT_ID 9

#define GREEN_CLR 1
#define YELLOW_CLR 2
#define RED_CLR 3

#define GREEN_LED 5
#define YELLOW_LED 6
#define RED_LED 7

#define PED_GREEN_LED 2 /* pedestrian traffic light */
#define PED_RED_LED 3 /* pedestrian traffic light */

#define GREEN_IN 8 /* to know if the green led is working */
#define YELLOW_IN 9 /* to know if the yellow led is working */
#define RED_IN 10 /* to know if the red led is working */

#define BUTTON 4

#define MASTER_ADDRESS 8

#define MIN_TIME 4000
#define MAX_TIME 10000

#define MAX_CYCLES 2 /* number of cycles the controller can spend without answering */

#define MAX_BUFFER 8

byte data_out[MAX_BUFFER]; /* buffer to send data to master */
byte data_in[MAX_BUFFER]; /* buffer to receive data from master */

unsigned long cycle_time; /* to check if the controller is alive during a cycle */
unsigned long original_cycle_time; /* original time, without shortening because of the pedestrians */
unsigned long previous_millis;
unsigned long previous_cycle;

int previous_color; /* to keep track of the colors of the traffic light */
int current_color;

int missed_cycles; /* how many cycles did the controller miss? */
int waiting_responses; /* how many responses we are waiting from the controller */
boolean sent_ping; /* are we expecting a ping from the controller? */

boolean blinking;
boolean is_yellow; /* when blinking, is the led yellow or not? */
boolean stay_yellow; /* when we receive an ON YELLOW the traffic light should stay yellow until told otherwise */
boolean can_start; /* when the traffic light receives a GRN message, it can go yellow -> green */
boolean controller_alive; 
boolean pedestrian; /* used to know if there's a pedestrian trying to cross the street */
boolean event_waiting; /* used to know if we have a command waiting to be serviced or not */

boolean green_working; /* is the green led working? */
boolean red_working; /* is the red led working? */
boolean yellow_working; /* is the yellow led working? */

boolean button_read; /* if somebody has already clicked the button, we don't need to take care of it anymore until it is green again */

void setup() {
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(PED_GREEN_LED, OUTPUT);
  pinMode(PED_RED_LED, OUTPUT);
  pinMode(GREEN_IN, INPUT);
  pinMode(RED_IN, INPUT);
  pinMode(YELLOW_IN, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  Wire.begin(TRAFFIC_LIGHT_ID);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(send_ack_request); /* only thing that is requested is the ACK */

  cycle_time = 4000;
  original_cycle_time = 4000;
  previous_millis = millis();
  previous_cycle = millis();

  missed_cycles = 0;
  waiting_responses = 0;
  sent_ping = false;

  blinking = true;
  is_yellow = true;
  stay_yellow = false;
  can_start = false;
  
  pedestrian = false;
  controller_alive = true;

  green_working = true;
  red_working = true;
  yellow_working = true;

  button_read = false;
  
  if (DEBUG) {
    Serial.begin(115200);
  }
}

void loop() {
  
  watch_the_cycle(); /* do the normal functioning of a traffic light */

  // check_controller(); /* check if the controller is alive */

  /* check if anybody clicked the button during the green
     if nobody has yet clicked */
  if (!button_read && current_color == GREEN_CLR) {
    check_button();
  }
  
  /* if it's blinking or the controller is dead,
     the traffic light should start blinking */
  if (blinking || !controller_alive) {
    blink_yellow();
    pedestrian_turn_off(); /* just to make sure these don't light up */
  }

  /* check if we received anything */
  if (event_waiting) {
    event_waiting = false;
    read_command();
  }

  /* check if we have a pedestrian waiting */
  if (pedestrian) {
    pedestrian = false;
    shorten_cycle();
  }

  /* if we received a ON YELLOW, we just stay yellow */
  if (stay_yellow) {
    transition_to_yellow();
    pedestrian_turn_off(); /* just to make sure these don't light up */
  }
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
  previous_millis = millis();
  stay_yellow = false;
  blinking = false;
  
  if (clr == YELLOW_CLR) {
    stay_yellow = true;
  }
  else if (clr == RED_CLR) {
    previous_color = GREEN_CLR;
    current_color = YELLOW_CLR;
    /* next color is red */
  }
  else if (clr == GREEN_CLR) {
    previous_color = YELLOW_CLR;
    current_color = RED_CLR;
    /* next color is green */
  }

  if (DEBUG) {
    Serial.print("Start color: ");
    Serial.println(clr);
  }
}

/*
    to be executed when the traffic light
    receives an OFF message
*/
void turn_traffic_light_off() {
  blinking = true;
  if (DEBUG) {
    // Serial.println("Blinking");
  }
}

/*
    to be executed when the traffic light
    receives a GRN message
*/
void start_cycle() {
  blinking = false;
  previous_millis = millis();
  can_start = true;
  previous_color = YELLOW_CLR;
  current_color = RED_CLR;

  if (DEBUG) {
    Serial.println("Starting cycle");
  }
}

/*
    to be executed when the traffic light
    receives a TIME message
*/
void cycle_time_reset(int new_time) {
  if (DEBUG) {
    Serial.print("Cycle changed from ");
    Serial.print(original_cycle_time);
    Serial.print(" to ");
    Serial.println(new_time);
  }
  original_cycle_time = new_time;
  cycle_time = new_time;
}

/* CHECK STATE FUNCTIONS */

void check_yellow_led() {
  /* if the voltage coming from a led is high, 
     then we know it's not working */
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

void check_green_led() {
  if (digitalRead(GREEN_IN) == HIGH) {
    green_working = false;
    blinking = true;
    if (DEBUG) {
      Serial.println("green not working");
    }
  }
  else {
    green_working = true;
  }
}

void check_red_led() {
  if (digitalRead(RED_IN) == HIGH) {
    red_working = false;
    blinking = true;
    if (DEBUG) {
      Serial.println("red not working");
    }
  }
  else {
    red_working = true;
  }
}

void check_controller() {
  if (!controller_alive) {
    return;
  }
  
  unsigned long current_millis = millis();

  if ((current_millis - previous_cycle) >= cycle_time) {
    if (sent_ping) {
      if (missed_cycles == MAX_CYCLES) {
        controller_alive = false;
        blinking = true;
        sent_ping = false;
        missed_cycles = 0;
        if (DEBUG) {
          Serial.println("Controller died");
        }
      }
    }
    else {
      send_ping();
      sent_ping = true;
    }
    previous_cycle = millis();
    if (sent_ping) {
      missed_cycles++;
    }
  }
}

/* PEDESTRIAN FUNCTIONS */

void check_button() {
  // improve the read
  if ((digitalRead(BUTTON) == LOW)) {
    delay(10); // stable the read
    pedestrian = true;
    button_read = true;
  }
}

void shorten_cycle()
{
  if (DEBUG) {
    Serial.println("Shorten cycle");
  }
  unsigned long current_millis = millis();

  /* if the time that has passed is less than MIN_TIME / 2, 
     then the cycle can be cut down by half */
  if ((cycle_time - (current_millis - previous_millis)) >= (MIN_TIME / 2)) {
    original_cycle_time = cycle_time;
    cycle_time = cycle_time / 2;
  }
  else {
    /* it's already in the end of the cycle, we can't shorten it */
    previous_millis = millis(); /* let's just update time */
  }
}

/* TRAFFIC LIGHT NORMAL FUNCTIONING */

void watch_the_cycle() {
  /* if the traffic light is in the blinking state
   *  then we don't have to do anything else and if
   *  it is indefinitely yellow the same happens */
  if (blinking || stay_yellow) {
    return;
  }

  unsigned long current_millis = millis();

  /* before transitioning to yellow -> green we need to make sure 
     that we can (only after receiving the GRN message) */
  if (current_color == RED_CLR && can_start && 
          ((current_millis - previous_millis) >= 1000)) {
    if (DEBUG) {
      Serial.println("to yellow from red");
    }
    previous_millis = millis();
    pedestrian_to_red();
    cycle_time = original_cycle_time; /* go back to normal cycle time, before it was shortened by the pedestrian */
    transition_to_yellow();
    button_read = false;
    can_start = false;
  }
  else if (current_color == GREEN_CLR &&
          ((current_millis - previous_millis) >= (cycle_time - 1000))) {
    if (DEBUG) {
      Serial.println("to yellow from green");
    }
    previous_millis = millis();
    cycle_time = original_cycle_time; /* go back to normal cycle time, before it was shortened by the pedestrian */
    transition_to_yellow();
    button_read = false;
  }
  /* if the time passed is bigger than one second, yellow time is up */
  else if (current_color == YELLOW_CLR &&
     ((current_millis - previous_millis) >= 1000)) {
    previous_millis = millis();
    if (previous_color == RED_CLR) {
      transition_yellow_to_green();
      if (DEBUG) {
        Serial.println("to green");
      }
    }
    else if (previous_color == GREEN_CLR) {
      transition_yellow_to_red();
      pedestrian_to_green();
      if (DEBUG) {
        Serial.println("to red");
      }
    }
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
      check_yellow_led();
      is_yellow = true;
    }
  }
}

void transition_to_yellow() {
  if (DEBUG) {
    Serial.println("To yellow");
  }
  
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  check_yellow_led(); /* check if the led is working */ 
  /* the traffic light can still work without it */

  previous_color = current_color;
  current_color = YELLOW_CLR;
}

void transition_yellow_to_green() {
  if (DEBUG) {
    Serial.println("To green");
  }
  
  digitalWrite(RED_LED, LOW); /* just to make sure */
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);

  check_green_led(); /* check if the led is working */ 
  
  previous_color = current_color;
  current_color = GREEN_CLR;
}

void transition_yellow_to_red() {
  if (DEBUG) {
    Serial.println("To red");
  }
  digitalWrite(GREEN_LED, LOW); /* just to make sure */
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  
  check_red_led(); /* check if the led is working */   
  
  if (red_working) {
    send_red();
  }

  previous_color = current_color;
  current_color = RED_CLR;
}

void pedestrian_turn_off() {
  digitalWrite(PED_RED_LED, LOW);
  digitalWrite(PED_GREEN_LED, LOW);
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

/* FUNCTIONS TO SEND MESSAGES TO THE CONTROLLER */

void send_ping() {
  make_ping_msg();
  Wire.beginTransmission(MASTER_ADDRESS);
  Wire.write(data_out, MAX_BUFFER);
  Wire.endTransmission();
  waiting_responses++;
  
  if (DEBUG) {
    Serial.println("Ping sent");
  }
}

/* 
 *  send_ack_request is called when the slave
 *  receives a request for an ack, it doesn't 
 *  specify where it is going to
 */
void send_ack_request() {
  make_ack_msg();
  Wire.write(data_out, MAX_BUFFER);

  if (DEBUG) {
    Serial.println("Ack sent");
  }
}

/* 
 *  send_ack is called when the slave receives
 *  a command from the master. it simply answers,
 *  specifying where it is going to
 */
void send_ack() {
  make_ack_msg();
  Wire.beginTransmission(MASTER_ADDRESS);
  Wire.write(data_out, MAX_BUFFER);
  Wire.endTransmission();

  if (DEBUG) {
    Serial.println("Ack sent");
  }
}

void send_red() {
  /* we can't send a message if the red led
     is not working */
  if (!red_working) {
    return;
  }
  make_red_msg();
  Wire.beginTransmission(MASTER_ADDRESS);
  Wire.write(data_out, MAX_BUFFER);
  Wire.endTransmission();
  waiting_responses++;

  if (DEBUG) {
    Serial.println("Red sent");
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
  controller_alive = true,
  send_ack();
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
  send_ack();
  controller_alive = true;
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

  /* we've reached the end, it's a ping command so we reply with an ack */
  send_ack();
  controller_alive = true;
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

  /* we've reached the end, it's an ack command so we know the controller is alive */
  controller_alive = true;
  sent_ping = false;
  missed_cycles = 0;
  waiting_responses--;
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
  send_ack();
  controller_alive = true;
  missed_cycles = 0;
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
  send_ack();
  controller_alive = true;
  missed_cycles = 0;
}

