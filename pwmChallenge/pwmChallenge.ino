int ledPin = 7;
unsigned int duty, period;

void setup() {
  pinMode(ledPin, OUTPUT);
  period = set_period(10);
  duty = set_duty(period);
}

void loop() {
  for (int val = 0; val <= period; val += duty) {
    digitalWrite(ledPin, HIGH);
    delayMicroseconds(val);
    
    digitalWrite(ledPin, LOW);
    delayMicroseconds(period - val);
  }

  for (int val = duty; val <= period; val += duty) {
    digitalWrite(ledPin, HIGH);
    delayMicroseconds(period - val);
    
    digitalWrite(ledPin, LOW);
    delayMicroseconds(val);
  }
}

int set_period(int period) {
  period = period * 1000;
  return period;
}

int set_duty(int duty) {
  duty = duty / 100;
  return duty;
}
