#include <Wire.h>

//GPIO configuration
#define INT_1_PIN     2
#define DIR_1_PIN     10
#define MOTOR_1_PWM   5
#define MOTOR_2_PWM   6

//Interrupt Configuration
#define MOTOR_1_PIN         PINB
#define MOTOR_1_DIR         PINB2 

#define REDUCT 46.67
#define PULSES  32.0
  
void setMotor1Pwm(int16_t pwm) { 
  pwm = constrain(pwm,-250,250);
  if(pwm < 0)
  {
    digitalWrite(5, LOW);
    analogWrite(6, abs(pwm));  
  }
  else
  {
    digitalWrite(6, LOW);
    analogWrite(5, abs(pwm));
  }
}

volatile uint16_t count1 = 0;

void motor1() {
   count1++;
}

volatile double speed = 0;
void speedRPM() {
  static unsigned long lastCount = count1;
  static unsigned long lastTime = millis();


  double incrAngulo = (double) ((long) lastCount - (long) count1 )/ 360.0 * PULSES / REDUCT ;
  speed = 1000.0 * incrAngulo / (double) ((millis() - lastTime)) * 60;


  lastTime = millis();
  lastCount = count1;
}

void setup() {

  Serial.begin(115200);
  
  pinMode(INT_1_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(INT_1_PIN),motor1,RISING);
    /*
     Cuando el TIMER0 del ATMEGA328, pase por 0F se ejecutará
     nuestra interrupción (cada ms).
     Este TIMER es el que usa la función millis().
     Referencia: https://learn.adafruit.com/
  */
  OCR0A = 0x0F;
  TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
  if (!(millis()%100)) speedRPM();
}
unsigned long sum = 0;

void loop() {  
    double err = 120.0+speed;




  sum += err;

  setMotor1Pwm(constrain(abs(sum * 0.8), 0, 255));
  Serial.println(String(-speed) + " 120"); delay(100);

  
  
}
