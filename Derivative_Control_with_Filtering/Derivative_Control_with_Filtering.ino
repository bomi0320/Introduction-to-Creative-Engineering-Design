#include <Servo.h> // 서보 헤더파일 포함

// Arduino pin assignment
#define PIN_LED 9 // LED를 아두이노 GPIO 9번 핀에 연결
#define PIN_SERVO 10 // 서보모터를 아두이노 GPIO 10번 핀에 연결
#define PIN_IR A0 // 적외선센서를 아두이노 A0핀에 연결

#include "medianfilter.h" // 중위수필터파일 포함

// Framework setting
#define _DIST_TARGET 255 // 탁구공의 목표 위치를 255mm로 설정
#define _DIST_MIN 100 // 거리 센서가 인식 가능하게 설정한 최소 거리
#define _DIST_MAX 410 // 거리 센서가 인식 가능하게 설정한 최대 거리

// Distance sensor
#define _DIST_ALPHA 0.2 // ema 필터에 적용할 알파값***

// Servo range
#define _DUTY_MIN 700 // 서보의 최소 각도값***
#define _DUTY_NEU 1320 // 서보의 중간 각도값***
#define _DUTY_MAX 1900 // 서보의 최대 각도값*** 

// 레일플레이트 높이를 조정하는데 필요한 서보의 각도값
#define _SERVO_HIGH 1170 // 22cm일때 서보의 각도값***
#define _SERVO_NEU 1320 // 20cm(수평)일때 서보의 각도값***
#define _SERVO_LOW 1480 // 18cm일때 서보의 각도값***

// Servo speed control
#define _SERVO_ANGLE 30 // 서보 각도 설정
#define _SERVO_SPEED 200 // 서보 속도 설정

//Event periods
#define _INTERVAL_DIST 20 // 거리센서 측정 주기
#define _INTERVAL_SERVO 20 // 서보위치 갱신 주기
#define _INTERVAL_SERIAL 100 // 제어 상태 시리얼 출력 주기

// PID parameters
#define _KP 1.0 // 비례이득
#define _KD 20.5 // 미분이득

// Servo instance
Servo myservo; // 서보 정의

int a, b, c, d, e, f, g; // 적외선거리센서 보정을 위한 구간별 보간에 필요한 변수

// Distance sensor
float dist_target; // 탁구공 목표 값
float dist_mid;
float dist_filtered;
float dist_raw; // 적외선센서로부터 얻은 거리 값
float dist_cali; // 구간별 보간을 통해 얻은 거리 값
float dist_ema; // ema 필터링을 거친 거리 값
float alpha; // ema 필터링의 알파값을 저장하는 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
// last_sampling_time_dist: 거리센서 측정 주기
// last_sampling_time_servo: 서보 위치 갱신 주기
// last_sampling_time_serial: 제어 상태 시리얼 출력 주기
bool event_dist, event_servo, event_serial; // 각각의 주기에 도달했는지 여부를 불리언 값으로 저장하는 변수

// Servo speed control
int duty_chg_per_interval; // 한 주기당 변화할 서보 활동량
int duty_target, duty_curr; // duty_target: 목표위치
                            // duty_curr: 서보에 입력할 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

// 센서값을 읽고 구간별 보간을 통해 얻은 거리를 리턴하는 함수

float calcMid(short value){
  float volt = float(analogRead(PIN_IR));
  float val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 100 + 300.0 / (b - a) * (val - a);
}

float calcDistance(float dist_raw){ 
  // distance <= 150mm
  if (dist_raw <= b){
    dist_cali = 100 + (150 - 100) / (b - a) * (dist_raw - a);
  }
  
  // 150mm < distance <= 200mm
  else if(dist_raw > b && dist_raw <= c){
    dist_cali = 150 + (200 - 150) / (c - b) * (dist_raw - b);
  }
  
  // 200mm < distance <= 250mm
  else if(dist_raw > c && dist_raw <= d){
    dist_cali = 200 + (250 - 200) / (d - c) * (dist_raw - c);
  }
  
  // 250mm < distance <= 300mm
  else if(dist_raw > d && dist_raw <= e){
    dist_cali = 250 + (300 - 250) / (e - d) * (dist_raw - d);
  }

  // 300mm < distance <= 350mm
  else if(dist_raw > e && dist_raw <= f){
    dist_cali = 300 + (350 - 300) / (f - e) * (dist_raw - e);
  }

  // 350mm < distance
  else{
    dist_cali = 350 + (400 - 350) / (g - f) * (dist_raw - f);
  }
  
  return dist_cali;
}

MedianFilter<calcMid> filter;

void setup(){
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT); // LED를 출력 모드로 설정
  myservo.attach(PIN_SERVO); // 디지털 IO핀을 서보 객체에 연결

  filter.init();
  
  // 구간별 보간에 필요한 변수(각 거리에 따른 적외선센서의 거리 출력 값)
  a = 89;  // 100mm
  b = 147;  // 150mm  //147  
  c = 191;  // 200mm
  d = 212;  // 250mm
  e = 240;  // 300mm
  f = 265;  // 350mm
  g = 290;  // 400mm
  
  // initialize global variables
  alpha = _DIST_ALPHA; // ema의 알파값 초기화
  dist_ema = 0; // dist_ema 값 초기화

  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU); // 서보 모터를 중간 위치에 지정

  // initialize serial port
  Serial.begin(115200); // 시리얼 포트를 115200의 속도로 연결

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0); // 한 주기마다 이동할 양

  // 이벤트 변수 초기화
  last_sampling_time_dist = 0; // 마지막 거리 측정 시간 초기화
  last_sampling_time_servo = 0; // 마지막 서보 업데이트 시간 초기화
  last_sampling_time_serial = 0; // 마지막 출력 시간 초기화
  event_dist = event_servo = event_serial = false; // 각 이벤트 변수 false로 초기화
} 


void loop(){
  
  unsigned long time_curr = millis(); // event 발생 조건 설정
  
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true; // 거리 측정 주기에 도달했다는 이벤트 발생
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
    last_sampling_time_servo += _INTERVAL_SERVO; 
    event_servo = true; // 서보모터 제어 주기에 도달했다는 이벤트 발생
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true; // 출력주기에 도달했다는 이벤트 발생
  }
  
  // get a distance reading from the distance sensor
  if(event_dist){
    event_dist = false;
    
    if(filter.ready()){ // 필터가 값을 읽을 준비가 되었는지 확인
    
      dist_filtered = filter.read(); 
      // dist_filtered = calcDistance(dist_mid); // 구간별 보간을 통해 얻은 값을 저장

      //ema 필터링
      if(dist_ema == 0){ 
        dist_ema = dist_filtered;
      }
      else{
        dist_ema = alpha * dist_filtered + (1-alpha) * dist_ema;
      }

      // PID control logic
      error_curr = _DIST_TARGET - dist_ema; // 탁구공의 목표위치(255mm) - 현재 탁구공의 위치
      pterm = _KP * error_curr;
      dterm = _KD * (error_curr - error_prev);
      
      if(pterm > 0){  // 탁구공이 가까이 있을 때
        control = dterm + pterm;
      }
      else{  // 탁구공이 멀리 있을 때
        control = dterm + 0.7 * pterm;
      }
      
      // control = dterm + pterm;
      duty_target = _DUTY_NEU + control;

      // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
      //duty_target이 _DUTY_MIN, _DUTY_MAX의 범위 안에 있도록 제어 
      if(duty_target < _DUTY_MIN) { duty_target = _DUTY_MIN; } 
      else if(duty_target > _DUTY_MAX) { duty_target = _DUTY_MAX; }

      //update error_prev
      error_prev = error_curr;
    }
  }
 
  if(event_servo){
    event_servo = false;
    
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr){ // 목표 값이 서보에 입력한 값보다 큰 경우
      duty_curr += duty_chg_per_interval; // duty_curr은 주기마다 duty_chg_per_interval 만큼 증가
    }
    else{
      duty_curr -= duty_chg_per_interval; // duty_curr은 주기마다 duty_chg_per_interval 만큼 감소
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr); // 서보 움직임 조절
  }

  if(event_serial){
    event_serial = false;
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}
