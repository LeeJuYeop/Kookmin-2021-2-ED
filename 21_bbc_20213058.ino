#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255  // 멈추려는 목표거리 정의
#define _DIST_MIN 100  // 측정 최소거리, 플로터 범위
#define _DIST_MAX 410    // 측정 최대거리,플로터 범위

// Distance sensor
#define _DIST_ALPHA 0.1

// Servo range
#define _DUTY_MIN 1000     // 서보 duty 최소값
#define _DUTY_NEU 1700    // 서보 duty 중간값
#define _DUTY_MAX 2300    // 서보 duty 최댓값

// Servo speed control
#define _SERVO_ANGLE 30       // 서보 각도
#define _SERVO_SPEED 30     // 서보의 속도

// Event periods
#define _INTERVAL_DIST 20   // 거리측정 주기
#define _INTERVAL_SERVO 20    // 서보 조정 주기  
#define _INTERVAL_SERIAL 100  // 시리얼 제어 주기

//////////////////////
// global variables //
//////////////////////

// Servo instance 
Servo myservo;    // 서보 변수명을 myservo로 선언

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema, alpha;    // 거리 측정값, ema필터 적용값

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; // 거리, 서보, 시리얼 측정 여부
   
// Servo speed control
int duty_chg_per_interval; // 한 주기 당 제어할 최대 duty값
int duty_target, duty_curr; // 목표 pulse주기값, 현재 pulse주기값



void setup() {
// initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO);

// move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); // 서보 중립위치로 초기화

// initialize serial port
  Serial.begin(57600); //

// initialize global variables 
  dist_target = _DIST_TARGET;
  dist_raw = ir_distance();
  alpha = _DIST_ALPHA;

  last_sampling_time_dist = 0; // last_sampling_time_dist 초기화
  last_sampling_time_servo = 0; // 
  last_sampling_time_serial = 0; // 

  event_dist = event_servo = event_serial = false;

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / ((float)_SERVO_ANGLE) * _INTERVAL_SERVO / 1000; // 한 주기 당 제어할 최대 duty값 초기화
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
    event_dist = false; //
 // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered(); // 
    dist_ema = (alpha*dist_raw) + (1-alpha)*dist_ema;

    if(dist_ema > 255){
      duty_target = 1200;
    }
    else{
      duty_target = 2100;
    }

    // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target < _DUTY_MIN){
      duty_target = _DUTY_MIN;
    } //
    if(duty_target > _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }
    
  }
  if(event_servo) {
    event_servo = false; // [20213078]

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
    myservo.writeMicroseconds(duty_curr); // [20213058]
    }
    
  if(event_serial) {
    event_serial = false; //
    Serial.print("dist_ir:");
//    Serial.print(dist_raw);
//    Serial.print("dist_ema:");
    Serial.print(dist_ema);
    Serial.print(",duty_target:");
    //Serial.print(map(duty_target,1000,2300,410,510));
    Serial.print(",duty_curr:");
    //Serial.print(map(duty_curr,1000,2300,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410"); // 
// Min :측정 최소거리 Max : 최대거리 Low : 목표구역 최소거리, dist_target :기준이 되는 거리, Low : 목표구역 최소거리 값을 시리얼 모니터에 표시
  }
}
  
float ir_distance(void){ // return value unit: mm
  int a = 84;  // 100mm에서 dist_raw
  int b = 280; // 400mm에서 dist_raw
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  val = 100 + 300.0 / (b - a) * (val - a);
  return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  return ir_distance(); // for now, just use ir_distance() without noise filter.
}