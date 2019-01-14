//const int QRD1114_PIN_Right = A0;
//const int QRD1114_PIN_Left = A1;
int echo_right = A2;
int trig_right = A3;
int echo_left = A4;
int trig_left = A5;
int in1 = 7;
int in2 = 8;
int in3 = 9;  //in3 front left
int in4 = 11;
int ena = 6;
int enb = 5;
int ABS = 200;
float distance_right=0;
float distance_left=0;
#define LT_R !digitalRead(10)
#define LT_M !digitalRead(4)
#define LT_L !digitalRead(2)


typedef enum{Determine_State,MOVE_FORWARD,TURN,STOP} state_t;
state_t state=Determine_State;

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
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}

void move_left(){
  analogWrite(ena,ABS);
  analogWrite(enb,ABS);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
}

void stop_moving(){
  digitalWrite(ena,LOW);
  digitalWrite(enb,LOW);
}

int distance_right_test(){
   digitalWrite(trig_right,LOW);
   delayMicroseconds(2);
   digitalWrite(trig_right,HIGH);
   delayMicroseconds(20);
   digitalWrite(trig_right,LOW);
   distance_right=pulseIn(echo_right,HIGH);
   distance_right=distance_right/58;
   return (int)distance_right;
}

int distance_left_test(){
   digitalWrite(trig_left,LOW);
   delayMicroseconds(2);
   digitalWrite(trig_left,HIGH);
   delayMicroseconds(20);
   digitalWrite(trig_left,LOW);
   distance_left=pulseIn(echo_left,HIGH);
   distance_left=distance_left/58;
   return (int)distance_left;
}

//float Right_Edge_Sensor(){
//  int proximityADC = analogRead(QRD1114_PIN_Right);
//  float proximityV = (float)proximityADC * 5.0 / 1023.0;
//  return (float)proximityV;
//  delay(50);
//}

//float Left_Edge_Sensor(){
//  int proximityADC = analogRead(QRD1114_PIN_Left);
//  float proximityV = (float)proximityADC * 5.0 / 1023.0;
//  return (float)proximityV;
//  delay(50);
//}

//float Back_Edge_Sensor(){
//  int proximityADC = analogRead(QRD1114_PIN);
//  float proximityV = (float)proximityADC * 5.0 / 1023.0;
//  return (float)proximityV;
//}

void setup() {
  // put your setup code here, to run once:
pinMode(echo_right,INPUT);
pinMode(trig_right,OUTPUT);
pinMode(echo_left,INPUT);
pinMode(trig_left,OUTPUT);
pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(in3,OUTPUT);
pinMode(in4,OUTPUT);
pinMode(ena,OUTPUT);
pinMode(enb,OUTPUT);
//pinMode(QRD1114_PIN_Right,INPUT);
//pinMode(QRD1114_PIN_Left,INPUT);
Serial.begin(9600);

}

void loop() {
 switch (state) 
    {
       case Determine_State:
            if(!LT_R){
              move_backwards();
              delay(800);
              move_right();
              delay(500);
              state=Determine_State;
              break;
            }else if(!LT_L){
                move_backwards();
                delay(800);
                move_left();
                delay(500);
                state=Determine_State;
                break;
            }else{
              distance_right= distance_right_test();
              //Serial.print(distance_right);
              distance_left= distance_left_test();
              //Serial.println(distance_left);
              if(distance_right>50 && distance_left>70){
                   state=MOVE_FORWARD;
              }else if(abs(distance_right-distance_left)>15){
                   state=TURN;
              }else{
                   state=MOVE_FORWARD;
              }  
            }
            break;
       case MOVE_FORWARD:
            move_forward();
            //Serial.println("forward");
            delay(5);
            state=Determine_State;
            break;
       case TURN:
            if(distance_right>distance_left){
              move_left();
              //Serial.println("left");
            }else{
              move_right();
              //Serial.println("right");
            }
            delay(5);
            state=Determine_State;
            break;
       case STOP:
            stop_moving();
            //Serial.println("stop");
            delay(5);
            state=Determine_State;
            break;
      
    }  


  
  
}
