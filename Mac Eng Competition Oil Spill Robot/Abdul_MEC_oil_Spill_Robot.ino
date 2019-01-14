#include <Servo.h>
Servo myservo;
Servo myservo1;

int in1 = 9;
int in2 = 8;
int in3 = 4;
int in4 = 3;

void move_forward(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
}

void move_backwards(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}

void move_right(){
  
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
}

void move_left(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}

void stop_moving(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}

void setup() {
delay(1000);

myservo.attach(12);
myservo1.attach(10);
myservo.write(0);
myservo1.write(0);
pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(in3,OUTPUT);
pinMode(in4,OUTPUT);
stop_moving();
move_backwards();
delay(500);
myservo.write(20);
  myservo1.write(150);
  delay(2000);
  myservo1.write(0);
  delay(2000);
  myservo.write(180);
  move_left();
  delay(2000);
  move_right();
  delay(2000);
  move_left();
  delay(2000);
  move_right();
  delay(2000);
  move_left();
  delay(2000);
  move_right();
  delay(2000);
  move_left();
  delay(2000);
 move_forward();
 delay(500);
}

void loop() {
  //move_forward();
}
