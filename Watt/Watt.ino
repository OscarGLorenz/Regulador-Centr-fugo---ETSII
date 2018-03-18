#include <Wire.h>

//GPIO configuration
#define INT_1_PIN     2
#define DIR_1_PIN     10
#define MOTOR_1_PWM   5
#define MOTOR_2_PWM   6

//Motor configuration
#define REDUCT 46.67
#define PULSES  32.0

template <typename T>
union Data {
  T value;
  byte buffer[sizeof(T)];
};
template <typename T> using Union = Data<T>;

//Control motores
void setMotor(int16_t pwm) {
  pwm = constrain(pwm, -250, 250);
  if (pwm < 0)
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

volatile long encoder = 0;
volatile double speed = 0;

void speedRPM() {
  static long lastEncoder = encoder;
  static long lastTime = millis();


  double incrAngulo = (double) ((long) lastEncoder - (long) encoder ) / 360.0 * PULSES / REDUCT ;
  speed = 1000.0 * incrAngulo / (double) ((millis() - lastTime)) * 60;

  lastTime = millis();
  lastEncoder = encoder;
}

ISR(TIMER0_COMPA_vect) {
  if (!(millis() % 100)) speedRPM();
}

void setup() {

  Serial.begin(9600);
  Serial.setTimeout(100);

  pinMode(INT_1_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(INT_1_PIN), 
  [&encoder]() {
    encoder++;
  } 
  ,RISING);
  
  /*
    Cuando el TIMER0 del ATMEGA328, pase por 0F se ejecutará
    nuestra interrupción (cada ms).
    Este TIMER es el que usa la función millis().
    Referencia: https://learn.adafruit.com/
  */
  OCR0A = 0x0F;
  TIMSK0 |= _BV(OCIE0A);
}

void loop() {
  Union<double> data;
  static long sum = 0;
  static double ref = -30.0;

  if (Serial.available() > 0) {
    Serial.readBytes(data.buffer, sizeof(data));
    ref = -data.value;
  } 

  double err = ref - speed;
  sum = constrain(err+sum,-400,400);

  setMotor(sum * 0.8);

  data.value = -speed;
  Serial.write(data.buffer, sizeof(data));
  delay(100);
}
