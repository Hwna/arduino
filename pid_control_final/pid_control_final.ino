#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10 
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 119
#define _DIST_MAX 387

// Distance sensor
#define _DIST_ALPHA 0.23

// Servo range
#define _DUTY_MIN 1150
#define _DUTY_NEU 1310
#define _DUTY_MAX 1450

// Servo speed control
#define _SERVO_ANGLE 30.0
#define _SERVO_SPEED 173.0

// Event periods
#define _INTERVAL_DIST 10
#define DELAY_MICROS  1500
#define _INTERVAL_SERVO 10
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 0.64
#define _KD 23.50
#define _KI 0.008

//////////////////////
// global variables //
//////////////////////

float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 5;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

// Servo instance
Servo myservo;
// Distance sensor
float dist_target, dist_curr; // location to send the ball 
float dist_raw, dist_ema, alpha;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
float duty_chg_per_interval;
float duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

const float coE[] = {-0.0000107, 0.0060959, 0.2215089, 79.2129955};

// Sigmoid variables
float mean, z;

void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);
// initialize global variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
  dist_target = _DIST_TARGET;
  duty_target = duty_curr = _DUTY_NEU;
  alpha = _DIST_ALPHA;
  pterm = dterm = iterm = 0.0;


// move servo to neutral position
//  myservo.writeMicroseconds(_DUTY_NEU);
// initialize serial port
  Serial.begin(115200);
// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (float(_INTERVAL_SERVO) / (float)1000);
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
    dist_raw = filtered_ir_distance();
    
  // PID control logic
    error_curr = _DIST_TARGET - dist_raw; 
    pterm = _KP * error_curr; 
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm;
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  // duty_target < _DUTY_MIN 일 때 duty_target 를 _DUTY_MIN 로 고정
    } else if (duty_target > _DUTY_MAX) {
      duty_target = _DUTY_MAX; // duty_target > _DUTY_MAX 일 때 duty_target 를 _DUTY_MAX 로 고정
    }  

    // update error_prev
    error_prev = error_curr;
  }
  
  if(event_servo) {
    event_servo = false; // [3153] servo EventHandler Ticket -> false
    
    mean = (duty_target+duty_curr)/2.0;
    z = map(mean-duty_curr, -150, 150, -3, 3);
    duty_chg_per_interval = sigmoid(z);

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }  
    else if(duty_target < duty_curr) {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    else {
      duty_chg_per_interval = 0.0;  
    }
    
    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(" ,T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245, +G:265, m:0, M:800");
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  float x = ir_distance();
  float val = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  return val;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance_filtered();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = _DIST_ALPHA*lowestReading + (1-_DIST_ALPHA)*ema_dist;
  return ema_dist;
}

float sigmoid(float z) {
  return (1.0/(1.0+exp(-z))) * (1-1.0/(1.0+exp(-z))) * 120.0;
}#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10 
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 119
#define _DIST_MAX 387

// Distance sensor
#define _DIST_ALPHA 0.23

// Servo range
#define _DUTY_MIN 1150
#define _DUTY_NEU 1310
#define _DUTY_MAX 1450

// Servo speed control
#define _SERVO_ANGLE 30.0
#define _SERVO_SPEED 173.0

// Event periods
#define _INTERVAL_DIST 10
#define DELAY_MICROS  1500
#define _INTERVAL_SERVO 10
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 0.64
#define _KD 23.50
#define _KI 0.008

//////////////////////
// global variables //
//////////////////////

float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 5;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

// Servo instance
Servo myservo;
// Distance sensor
float dist_target, dist_curr; // location to send the ball 
float dist_raw, dist_ema, alpha;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
float duty_chg_per_interval;
float duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

const float coE[] = {-0.0000107, 0.0060959, 0.2215089, 79.2129955};

// Sigmoid variables
float mean, z;

void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);
// initialize global variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;
  dist_target = _DIST_TARGET;
  duty_target = duty_curr = _DUTY_NEU;
  alpha = _DIST_ALPHA;
  pterm = dterm = iterm = 0.0;


// move servo to neutral position
//  myservo.writeMicroseconds(_DUTY_NEU);
// initialize serial port
  Serial.begin(115200);
// convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (float(_INTERVAL_SERVO) / (float)1000);
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
    dist_raw = filtered_ir_distance();
    
  // PID control logic
    error_curr = _DIST_TARGET - dist_raw; 
    pterm = _KP * error_curr; 
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm;
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  // duty_target < _DUTY_MIN 일 때 duty_target 를 _DUTY_MIN 로 고정
    } else if (duty_target > _DUTY_MAX) {
      duty_target = _DUTY_MAX; // duty_target > _DUTY_MAX 일 때 duty_target 를 _DUTY_MAX 로 고정
    }  

    // update error_prev
    error_prev = error_curr;
  }
  
  if(event_servo) {
    event_servo = false; // [3153] servo EventHandler Ticket -> false
    
    mean = (duty_target+duty_curr)/2.0;
    z = map(mean-duty_curr, -150, 150, -3, 3);
    duty_chg_per_interval = sigmoid(z);

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }  
    else if(duty_target < duty_curr) {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    else {
      duty_chg_per_interval = 0.0;  
    }
    
    myservo.writeMicroseconds(duty_curr);
  }
  
  if(event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(" ,T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245, +G:265, m:0, M:800");
  }
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  float x = ir_distance();
  float val = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  return val;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance_filtered();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = _DIST_ALPHA*lowestReading + (1-_DIST_ALPHA)*ema_dist;
  return ema_dist;
}

float sigmoid(float z) {
  return (1.0/(1.0+exp(-z))) * (1-1.0/(1.0+exp(-z))) * 120.0;
}
