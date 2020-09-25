#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(9600); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = 0;
  toggle = 1;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  Serial.println(++count);
  toggle = toggle_state(toggle); // toggle LED value.
  digitalWrite(PIN_LED, toggle); // update LED status.
  if (count == 1) {
    delay(1000); // wait for 1,000 milliseconds
  } else if (count > 1 && count < 12) {
    delay(100); // wait for 100 ms
  } else {
    while(1){}
  }

}

int toggle_state(int toggle) {
  if (toggle == 1) {
    toggle = 0;
  } else if (toggle == 0 ) {
    toggle = 1;
  }
  return toggle;
}
