#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 180 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.3 // EMA weight of new sample (range: 0 to 1). Setting this value to 1 effectively disables EMA filter.

#define _DUTY_MIN 560 // servo full clockwise position (0 degree)
#define _DUTY_MAX 2400 // servo full counterclockwise position (180 degree)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev, dist_ema, alpha; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
int gradient; // 서보 각도를 계산하기 위해 필요한 변수 b의 기울기
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_MIN);
  analogWrite(PIN_LED, 255);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;
  dist_ema = 0;
  gradient = (2400-560)/(360-180); //102 

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  dist_ema = (alpha*dist_raw) + ((1-alpha)*dist_ema);



// adjust servo position according to the USS read value
  if(dist_ema < _DIST_MIN) { // 거리 18cm 이하: 0도
     myservo.writeMicroseconds(_DUTY_MIN);
  }
  else if(dist_ema > _DIST_MIN && dist_ema < _DIST_MAX){ // 거리 18~36cm 사이
     int result = gradient*(dist_ema - 180) + 560; // (18, 560)과 (36, 2400)을 지나는 직선 위 (dist_ema, result) 구하기
     myservo.writeMicroseconds(result);
  }
  else { // 거리 36cm 이상: 180도
    myservo.writeMicroseconds(_DUTY_MAX); 
  }

  // output the read value to the serial port
  Serial.print("Min:100,raw:");
  Serial.print(dist_raw);
  Serial.print(",ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());  
  Serial.println(",Max:400");
   
// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.

  // turn on the LED if the distance is between dist_min and dist_max
  if(reading == 0) {
    analogWrite(PIN_LED, 255); // 범위 밖인 경우 LED 끔
  }
  else {
    analogWrite(PIN_LED, 0);
  }
  
  if(reading == 0.0) reading = dist_prev;
  else dist_prev = reading;
  
  return reading;
}
