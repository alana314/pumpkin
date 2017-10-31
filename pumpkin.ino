#define servo1Forward 8
#define servo1Reverse 9
#define servo2Forward 10
#define servo2Reverse 11
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define ARMSOUT 380
#define LEFTARMSTOP 360
#define RIGHTARMSTOP 364
#define LEFTARMIN 345
#define RIGHTARMIN 340
#define LED1 6
#define LED2 7
int buttonEnabled = true;

void setup() {
  // put your setup code here, to run once:
  pinMode(servo1Forward, OUTPUT);
  pinMode(servo1Reverse, OUTPUT);
  pinMode(servo2Forward, OUTPUT);
  pinMode(servo2Reverse, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  Serial.begin(9600);
  pwm.begin();
  digitalWrite(servo1Forward, LOW);
  digitalWrite(servo1Reverse, LOW);
  digitalWrite(servo2Forward, LOW);
  digitalWrite(servo2Reverse, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  pwm.setPWMFreq(60);
  yield();
  
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {
  int sensorVal = digitalRead(2);
  if (sensorVal == LOW && buttonEnabled) {
    buttonEnabled = false;
    //stand up
    digitalWrite(servo2Forward, HIGH);
    delay(5000);
    digitalWrite(servo2Forward, LOW);
    delay(2000);
    //arms out
    pwm.setPWM(14, 0, ARMSOUT);
    pwm.setPWM(15, 0, ARMSOUT);
    delay(1100);
  
    //arms stop
    pwm.setPWM(14, 0, RIGHTARMSTOP);
    pwm.setPWM(15, 0, LEFTARMSTOP);
    delay(1500);
    //head up
    digitalWrite(servo1Forward, HIGH);
    delay(1000);
    digitalWrite(servo1Forward, LOW);
    delay(1000);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    delay(100);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    delay(100);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    
    //arms in
    pwm.setPWM(14, 0, RIGHTARMSTOP);
    pwm.setPWM(15, 0, LEFTARMSTOP);
    delay(1500);
    pwm.setPWM(14, 0, LEFTARMIN);
    pwm.setPWM(15, 0, RIGHTARMIN);
    delay(1019);
    pwm.setPWM(14, 0, RIGHTARMSTOP);
    pwm.setPWM(15, 0, LEFTARMSTOP);
    delay(1500);
  
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    //head down
    digitalWrite(servo1Reverse, HIGH);
    delay(1000);
    digitalWrite(servo1Reverse, LOW);
    delay(1000);
  
    //back down
    digitalWrite(servo2Reverse, HIGH);
    delay(5000);
    digitalWrite(servo2Reverse, LOW);
    delay(2000);
    buttonEnabled = true;
  }else
  {
    //Serial.println("Low, skipping");
  }
  
}
