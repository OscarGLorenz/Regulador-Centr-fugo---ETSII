#include <Wire.h>

//GPIO configuration
#define INT_1_PIN     3
#define DIR_1_PIN     10
#define MOTOR_1_PWM   6
#define MOTOR_1_H1    A0
#define MOTOR_1_H2    A1

//Motor configuration
#define REDUCT 46.67
#define PULSES  32.0
#define GEAR_RELATION 90.0/62.0
// Regulador PI
#define KP 0.0
#define KI 0.8
#define WIND_UP 400

//Control motores
void setMotor(int pwm) {
  pwm = constrain(pwm, -250, 250);
  if (pwm < 0) {
    digitalWrite(MOTOR_1_H1, LOW);
    digitalWrite(MOTOR_1_H2, HIGH);
    analogWrite(MOTOR_1_PWM, abs(pwm));
  } else {
    digitalWrite(MOTOR_1_H1, HIGH);
    digitalWrite(MOTOR_1_H2, LOW);
    analogWrite(MOTOR_1_PWM, abs(pwm));
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
  speed = 1000.0 * incrAngulo / (double) ((millis() - lastTime)) * 60 * GEAR_RELATION;

  // Guardar variables para el siguiente ciclo
  lastTime = millis();
  lastEncoder = encoder;
}

// Rutina de interrupción, cada 100ms se ejecuta speedRPM
ISR(TIMER0_COMPA_vect) {
  if (!(millis() % 100)) speedRPM();
}

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(1);

  pinMode(INT_1_PIN, INPUT_PULLUP);

  // Interrupción del encoder, a flaco de subida, suma 1 al contador encoder.
  attachInterrupt(digitalPinToInterrupt(INT_1_PIN),
  [&encoder]() {
    encoder++;
  }
  , RISING);
  speedRPM();
  delay(100);
  speedRPM();
  /*
    Cuando el TIMER0 del ATMEGA328, pase por 0F se ejecutará
    nuestra interrupción (cada ms).
    Este TIMER es el que usa la función millis().
    Referencia: https://learn.adafruit.com/
  */
  OCR0A = 0x0F;
  TIMSK0 |= _BV(OCIE0A);
}
double ref = 0.0;
double sum = 0.0;

void loop() {
  // Cálculo del error
  double err = ref + speed;

  // Cálculo de la suma para la I, con anti wind-up.
  sum  = constrain(err + sum, -WIND_UP, WIND_UP);

  // Regulador PI
  if (abs(ref) > 0.1) {
    setMotor(abs(KP * err + KI * sum));
  } else {
    setMotor(0);
    sum = 0;
  }

  Serial.println(-speed);

  if (Serial.available() > 0) {
    ref = Serial.parseFloat(); 
    Serial.parseFloat();
  }
  delay(100);
}

