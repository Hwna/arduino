int ledPin = 7;
double MS = 0.1; // ms
unsigned int period, duty;

void setup() {
  pinMode(ledPin, OUTPUT);
  period = set_period(MS);
  duty = set_duty(period);
}

void loop() {
  if (MS == 10) {
    for (int val = 0; val < period; val += duty) {
      digitalWrite(ledPin, HIGH);
      delayMicroseconds(val);
      
      digitalWrite(ledPin, LOW);
      delayMicroseconds(period - val);
    }
  
    for (int val = period; val > 0; val -= duty) {
      digitalWrite(ledPin, HIGH);
      delayMicroseconds(val);
      
      digitalWrite(ledPin, LOW);
      delayMicroseconds(period - val);
    }
  } else {
    int Max = 5000 / period;
    
    for (int val=0; val < period; val += duty) {
      for (int cnt = 0; cnt < Max; cnt++) {
        digitalWrite(ledPin, HIGH);
        delayMicroseconds(val);
        digitalWrite(ledPin, LOW);
        delayMicroseconds(period-val);
      }
    }
    
    for (int val=period; val > 0; val -= duty) {
      for (int cnt = 0; cnt < Max; cnt++) {
        digitalWrite(ledPin, HIGH);
        delayMicroseconds(val);
        digitalWrite(ledPin, LOW);
        delayMicroseconds(period-val);
      }
    }
  }
  
    
}

int set_period(double ms) {
  return ms * 1000;
}

int set_duty(int period) {
  return period / 100;
}
