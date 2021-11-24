#include <Servo.h> // 서보 헤더파일 포함

// Arduino pin assignment
#define PIN_SERVO 10 // LED를 아두이노 GPIO 9번 핀에 연결
#define PIN_IR A0 // 적외선센서를 아두이노 A0핀에 연결

#include "medianfilter.h" // 중위수필터 파일 포함 

#define _DIST_ALPHA 0.2 // ema 필터에 적용할 알파값 

Servo myservo; // 서보 정의

int a, b, c, d, e, f, g; // unit: mm

float dist_raw; // 적외선센서로부터 얻은 거리 값
float dist_cali; // 구간별 보간을 통해 얻은 거리 값
float dist_ema; // ema 필터링을 거친 거리 값
float alpha; // ema 필터링의 알파값

int high, neu, low; // 서보의 각도값

// 센서값을 읽고 구간별 보간을 통해 얻은 거리를 리턴하는 함수
float calcDistance(short value){ 
  float volt = float(analogRead(PIN_IR));
  dist_raw = ((6762.0/(volt-9.0))-4.0) * 10.0;
  
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

MedianFilter<calcDistance> filter;

void setup() {
  myservo.attach(PIN_SERVO); // 디지털 IO핀을 서보 객체에 연결
  
  filter.init();
  
  a = 89;  // 100mm
  b = 146;  // 150mm    
  c = 207;  // 200mm
  d = 211;  // 250mm
  e = 234;  // 300mm
  f = 259;  // 350mm
  g = 274;  // 400mm

  alpha = _DIST_ALPHA; // ema의 알파값 초기화
  dist_ema = 0; // dist_ema 값 초기화
  
  high = 1080;  // 22cm일때 서보의 각도값
  neu = 1230;  // 20cm(수평)일때 서보의 각도값
  low = 1390;  // 18cm일때 서보의 각도값
  
  // initialize serial port
  Serial.begin(57600); // 시리얼 포트를 57600의 속도로 연결
}

void loop() {
  
  if(filter.ready()){ // 필터가 값을 읽을 준비가 되었는지 확인
    
    float dist_filtered = filter.read(); // 구간별 보간을 통해 얻은 값을 저장

    //ema 필터링
    if(dist_ema == 0){ 
      dist_ema = dist_cali;
    }
    else{
      dist_ema = alpha * dist_cali + (1-alpha) * dist_ema;
    }
    
    Serial.print("min:0,max:500,dist_filtered:");
    Serial.print(dist_filtered);
    Serial.print(",dist_ema:");
    Serial.println(dist_ema);
    
    if(dist_cali > 255){ // 공이 255mm보다 가까이 있으면 레일 플레이트를 내림
      myservo.writeMicroseconds(high);
    }
    else{ // 공이 255mm보다 멀리 있으면 레일 플레이트를 올림
      myservo.writeMicroseconds(low);
    }
  }
  delay(20);
}
