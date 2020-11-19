#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

#define _DIST_ALPHA 0.5

// global variables
float dist_raw, dist_ema, alpha;

Servo myservo;

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO);
  
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  alpha = _DIST_ALPHA;
  dist_raw = 0.0;
  
// initialize serial port
  Serial.begin(115200);

}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  dist_raw = ir_distance();
  dist_ema = (alpha)*dist_raw + (1-alpha)*(dist_ema);
  Serial.print("min:0,max:500,dist:");
  Serial.println(dist_ema);
  delay(20);

  if (dist_ema >= 185) {
    myservo.writeMicroseconds(1200);
  } else {
    myservo.writeMicroseconds(1600);
  }
}
