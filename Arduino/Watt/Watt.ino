#include <Wire.h>
#include <SoftwareSerial.h>
#include <SparkFun_VL6180X.h> //https://github.com/sparkfun/SparkFun_ToF_Range_Finder-VL6180_Arduino_Library
#include <LCD.h>
#include <LiquidCrystal_I2C.h> //https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads

LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack
SoftwareSerial mySerial =  SoftwareSerial(10, 11);
VL6180x sensor(0x29);

void print(unsigned int vel, unsigned int pos) {
  lcd.home(); 
  String sep1, sep2;
  
  String a = "Velocidad";
  String b = "Posicion";

  for (int i = 0; i < 16-String("Velocidad").length()-String(vel).length()-String("RPM").length(); i++) 
    sep1.concat(" ");
   
  lcd.print("Velocidad" + sep1 + String(vel) + "RPM");

  lcd.setCursor(0,1);
  
  for (int i = 0; i < 16-String("Posicion").length()-String(pos).length()-String("mm").length(); i++) 
    sep2.concat(" ");

  lcd.print("Posicion" + sep2 + String(pos) + "mm"); 

}

float vref = 0.0;
float v = 0.0;
float pos = 0.0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  
  mySerial.begin(115200);
  mySerial.setTimeout(10);

  lcd.begin (16,2); 
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  
  if(sensor.VL6180xInit() != 0){
    Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
  }; 
  sensor.VL6180xDefautSettings(); //Load default settings to get started.
}

void loop() {
  if (Serial.available() > 0) {
    mySerial.println(vref = Serial.parseFloat());
  }
  if (mySerial.available() > 0) {
    v = Serial.parseFloat();
  }
  
  pos = sensor.getDistance();
  print(v,pos);

  //Serial.println(String(vref) + " " + String(v));
     
  delay(500);
}



