#define NR_LEDS 4
const int myPins[NR_LEDS] = {2, 3, 4, 5};

#define TIME_1S 1000

void setup() {
  for(int i = 0; i < NR_LEDS; i++) {
    pinMode(myPins[i], OUTPUT);
    digitalWrite(myPins[i], LOW);
  }
}


void loop() {
  for(int i = 0; i < NR_LEDS; i++) {
    digitalWrite(myPins[i], HIGH);
    delay(TIME_1S);
    digitalWrite(myPins[i], LOW);
  }
}

