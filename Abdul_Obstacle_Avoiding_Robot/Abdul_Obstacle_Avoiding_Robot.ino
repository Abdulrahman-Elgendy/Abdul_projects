#include <Servo.h>
Servo myservo;
int echo = A4;
int trig = A5;
int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int ena = 5;
int enb = 11;
int ABS = 150;
int rightDistance=0;
int leftDistance=0;
int middleDistance=0;
float distance=0;

void move_forward(){
  analogWrite(ena,ABS);
  analogWrite(enb,ABS);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
}

void move_backwards(){
  analogWrite(ena,ABS);
  analogWrite(enb,ABS);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}

void move_right(){
  analogWrite(ena,ABS);
  analogWrite(enb,ABS);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
}

void move_left(){
  analogWrite(ena,ABS);
  analogWrite(enb,ABS);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}

void stop_moving(){
  digitalWrite(ena,LOW);
  digitalWrite(enb,LOW);
}


int distance_test(){
   digitalWrite(trig,LOW);
   delayMicroseconds(2);
   digitalWrite(trig,HIGH);
   delayMicroseconds(20);
   digitalWrite(trig,LOW);
   distance=pulseIn(echo,HIGH);
   distance=distance/58;
   return (int)distance;
}
void setup() {
  // put your setup code here, to run once:

myservo.attach(3);
myservo.write(90);
pinMode(echo,INPUT);
pinMode(trig,OUTPUT);
pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(in3,OUTPUT);
pinMode(in4,OUTPUT);
pinMode(ena,OUTPUT);
pinMode(enb,OUTPUT);
stop_moving();
}

void loop() {
  // put your main code here, to run repeatedly
  myservo.write(100);
  delay(500);
  middleDistance=distance_test();
  if(middleDistance<=50){
      stop_moving();
      delay(1000);
      myservo.write(20);
      delay(1000);
      rightDistance=distance_test();
      myservo.write(100);
      delay(1000);
      myservo.write(175);
      delay(1000);
      leftDistance=distance_test();
      myservo.write(90);
      if(leftDistance<=20 && rightDistance<=20){
        move_backwards();
        delay(2000);
      }
      else if(rightDistance>leftDistance){
        move_right();
        delay(2000);
      }
      else{
        move_left();
        delay(2000);
      }
  }
  else{
    move_forward();
  }
}
