#include <LiquidCrystal.h>
#include <IRremote.h>

#define B 16754775
#define UNKNOWN_B 2747854299

int Reciever=12;
IRrecv irrecv(Reciever);
decode_results results;
unsigned long val;

LiquidCrystal lcd(1,2,4,5,6,7);

int relay=13;
int echo = A1;
int trig = A0;
int duration=0;
int distance_cm=0;
int distance_value=0;

typedef enum{Determine_State,SAFE,CAUTION,DANGER} state_t;
state_t state=Determine_State;

int distance_test(){
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(20);
  digitalWrite(trig,LOW);
  duration=pulseIn(echo,HIGH);
  distance_cm=(duration/2)/29.1;
  return (int) distance_cm;
}

void setup() {
  lcd.begin(16,2);
  pinMode(echo,INPUT);
  pinMode(trig,OUTPUT);
  pinMode(relay,OUTPUT); 
  irrecv.enableIRIn();
  //Serial.begin(9600);
}

void loop() {   
    switch (state) 
    {
       case Determine_State:
            lcd.clear();
            distance_value=distance_test();
            if(distance_value>10){
                 state=DANGER;
            }else if(irrecv.decode(&results)){
                 val=results.value;
                 //Serial.println(val);
                 irrecv.resume();
                 if(val==B || val==UNKNOWN_B){
                        state=CAUTION;
                 }else{
                        state=SAFE;
                 }
              
            }else{
                 if(val==B || val==UNKNOWN_B){
                        state=CAUTION;
                 }else{
                        state=SAFE;   
                 }
            }
            break;
       case SAFE:
            //lcd.print(distance_value);
            lcd.setCursor(0,1);
            lcd.print("enjoy ur day");
            delay(1000);
            state=Determine_State;
            break;
       case CAUTION:
            //lcd.print(distance_value);
            lcd.setCursor(0,1);
            lcd.print("stay focused");
            delay(1000);
            state=Determine_State;
            break;
       case DANGER:
            //lcd.print(distance_value);
            lcd.setCursor(0,1);
            lcd.print("stay focused");
            digitalWrite(relay,HIGH);
            delay(1000);
            state=Determine_State;
            digitalWrite(relay,LOW);
            break;
      
    }
}
