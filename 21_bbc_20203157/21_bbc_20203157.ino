#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10

#define PIN_IR A0
#define PIN_LED 9

Servo myservo;

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO);
  
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  Serial.print("min:0,max:500,dist:");
  Serial.println(raw_dist);
  delay(20);

  if (raw_dist >= 160) {
    myservo.writeMicroseconds(1200);
  } else {
    myservo.writeMicroseconds(1730);
  }
}
