#include <QTRSensors.h>

#define NUM_SENSORS  8
#define TIMEOUT       2500
#define EMITTER_PIN   31

#define rightMotor1 10 //Sağ Motorun ileri pini.b 
#define rightMotor2 11
#define rightMotorPWM 5

#define leftMotor1 8
#define leftMotor2 9 //Sol Motorun ileri pini.
#define leftMotorPWM 4

#define rightBaseSpeed 40
#define leftBaseSpeed 40

QTRSensorsRC qtrrc((unsigned char[]) {39, 41, 43, 45, 47, 49, 51, 53} , NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);

  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(38, OUTPUT);

  int i;
  digitalWrite(13, HIGH);
  digitalWrite(36, HIGH);
  digitalWrite(38, HIGH); 
  for (int i = 0; i < 150; i++) 
  {
    qtrrc.calibrate();
  }
  digitalWrite(13, LOW);

  Serial.begin(9600);
  delay(2000);
}

int pre_error = 0; //Önceki hata.

float Kp = 0.09; //Hata sabiti.
float Kd = 2.25; //Türev sabiti.
float Ki = 0; //İntegral sabiti.

float integral = 0;
float deriative = 0;

int rightMotorSpeed = 0;
int leftMotorSpeed = 0;

void loop()
{
  unsigned int sensors[6];
  int position = qtrrc.readLine(sensors);
  int error = position - 3500;
  /*
  integral = integral + (error * dt);
  deriative = (error - pre_error) / dt;
  pre_error = error;
  */
  integral = integral + error;
  
  deriative = error - pre_error;
  pre_error = error;
  
  int motorSpeed = (Kp * error) + (Kd * deriative) + (Ki * integral);

  rightMotorSpeed = rightBaseSpeed - motorSpeed  ;
  leftMotorSpeed = leftBaseSpeed + motorSpeed   ;

  if (rightMotorSpeed > 75  ) rightMotorSpeed = 75;
  if (leftMotorSpeed > 75 ) leftMotorSpeed = 75;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; 
  
  Serial.print(position);
  Serial.print("-----");
  Serial.print(rightMotorSpeed);
  Serial.print("-----");
  Serial.println(leftMotorSpeed);

  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, rightMotorSpeed);

  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, leftMotorSpeed);
Kp = 0.09; //Hata sabiti.
Kd = 2.25; //Türev sabiti.
}
