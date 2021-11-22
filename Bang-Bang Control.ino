#include <Servo.h>
// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0

Servo myservo;

int a, b, c, d, e, f, g; // unit: mm

int high, neu, low; // microseconds

void setup() {
  myservo.attach(PIN_SERVO); 
  
  // initialize serial port
  Serial.begin(57600);

  a = 89;  // 100mm 일때 raw 값
  b = 146;  // 150mm 일때 raw 값    
  c = 207;  // 200mm 일때 raw 값
  d = 211;  // 250mm 일때 raw 값
  e = 234;  // 300mm 일때 raw 값
  f = 259;  // 350mm 일때 raw 값
  g = 274;  // 400mm 일때 raw 값

  high = 1080;  // 22cm
  neu = 1230;  // 20cm(수평)
  low = 1390;  // 18cm
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali;
  
  // distance <= 150mm
  if (raw_dist <= b){
    dist_cali = 100 + (150 - 100) / (b - a) * (raw_dist - a);
  }
  
  // 150mm < distance <= 200mm
  else if(raw_dist > b && raw_dist <= c){
    dist_cali = 150 + (200 - 150) / (c - b) * (raw_dist - b);
  }
  
  // 200mm < distance <= 250mm
  else if(raw_dist > c && raw_dist <= d){
    dist_cali = 200 + (250 - 200) / (d - c) * (raw_dist - c);
  }
  
  // 250mm < distance <= 300mm
  else if(raw_dist > d && raw_dist <= e){
    dist_cali = 250 + (300 - 250) / (e - d) * (raw_dist - d);
  }

  // 300mm < distance <= 350mm
  else if(raw_dist > e && raw_dist <= f){
    dist_cali = 300 + (350 - 300) / (f - e) * (raw_dist - e);
  }

  // 350mm < distance
  else{
    dist_cali = 350 + (400 - 350) / (g - f) * (raw_dist - f);
  }
  
  Serial.print("min:0,max:500,raw_dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  
  if(dist_cali > 255){
    myservo.writeMicroseconds(high);
  }
  else{
    myservo.writeMicroseconds(low);
  }
  
  delay(20);
}
