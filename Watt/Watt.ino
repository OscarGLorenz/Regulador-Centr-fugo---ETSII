#include <Wire.h>

//GPIO configuration
#define INT_1_PIN     2
#define DIR_1_PIN     10
#define MOTOR_1_PWM   5
#define MOTOR_2_PWM   6

//Motor configuration
#define REDUCT 46.67
#define PULSES  32.0

// Regulador PI
#define KP 0.0
#define KI 0.8
#define WIND_UP 400

//Control motores
void setMotor(int16_t pwm) {
  pwm = constrain(pwm, -250, 250);
  if (pwm < 0) {
    digitalWrite(5, LOW);
    analogWrite(6, abs(pwm));
  } else {
    digitalWrite(6, LOW);
    analogWrite(5, abs(pwm));
  }
}

// Variables globales para los pulsos del encoder y la velocidad
volatile long encoder = 0;
volatile double speed = 0;

// Calcula la velocidad actual
void speedRPM() {
  // Variables estáticas, se conservan entre llamadas a la función.
  static long lastEncoder = encoder;
  static long lastTime = millis();

  // El ángulo que ha variado en grados, teniendo en cuenta la reducción.
  double incrAngulo = (double) ((long) lastEncoder - (long) encoder ) / 360.0 * PULSES / REDUCT ;

  // Velocidad en RPM
  speed = 1000.0 * incrAngulo / (double) ((millis() - lastTime)) * 60;

  // Guardar variables para el siguiente ciclo
  lastTime = millis();
  lastEncoder = encoder;
}

// Rutina de interrupción, cada 100ms se ejecuta speedRPM
ISR(TIMER0_COMPA_vect) {
  if (!(millis() % 100)) speedRPM();
}

void setup() {

  Serial.begin(9600);
  Serial.setTimeout(100);
  
  pinMode(INT_1_PIN, INPUT_PULLUP);

  // Interrupción del encoder, a flaco de subida, suma 1 al contador encoder.
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
  // Variables estáticas de la suma de la parte Integral y la referencia.
  static long sum = 0;
  static double ref = -0.0;

  // Actualizar la referencia por serial
  if (Serial.available() > 0) {
    ref = -Serial.parseFloat();
  } 

  // Cálculo del error
  double err = ref - speed;

  // Cálculo de la suma para la I, con anti wind-up.
  sum = constrain(err+sum,-WIND_UP,WIND_UP);

  // Regulador PI
  setMotor(KP * err + KI * sum);

  // Mostrar por serial la velocidad
  Serial.println(-speed);

  delay(100);
}
