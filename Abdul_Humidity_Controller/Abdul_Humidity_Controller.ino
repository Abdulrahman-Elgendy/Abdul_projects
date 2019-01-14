#include <LiquidCrystal.h>
#include <dht.h>

#define dataPin 8
int relay=12;
dht DHT;
LiquidCrystal lcd(1,2,4,5,6,7);

void setup() {
  pinMode(relay,OUTPUT);
  delay(1000);
  lcd.begin(16,2);
}

void loop() {
  lcd.clear();
  int values=DHT.read11(dataPin);
  lcd.print("temp: ");
  lcd.print(DHT.temperature);
  lcd.print(" deg");  
  lcd.setCursor(0,1);
  lcd.print("humidity: ");
  lcd.print(DHT.humidity); 
  lcd.print("%");

  if(DHT.humidity>=30.00){
    digitalWrite(relay,HIGH);
  }
  else{
    digitalWrite(relay,LOW);
  }
  
  delay(1500);
}
