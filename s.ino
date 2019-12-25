#include <QTRSensors.h>
#define Kp 0.1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 3
#define MaxSpeed 200
#define BaseSpeed 120
#define speedturn 90
#define NUM_SENSORS  6
#define rightMotor1 8
#define rightMotor2 9
#define rightMotorPWM 3
#define leftMotor1 5
#define leftMotor2 6
#define leftMotorPWM 10
#define motorpower 7

#define IR_RIGHT A1
#define IR_LEFT A0
#define IR_TOP 4
#define buzzer 2

QTRSensors qtr;
uint16_t sensors[NUM_SENSORS];
char path[20];
int i = 0;

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  pinMode(buzzer, OUTPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_TOP, INPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A7, A6, A5, A4, A3, A2
  }, NUM_SENSORS);
  qtr.setEmitterPin(13);

  delay(2000);

  int i;
  for (int i = 0; i < 100; i++)
  {
    if (i < 25 || i > 75) {
      move(1, speedturn, 0);//motor derecho hacia adelante
      move(0, speedturn, 1);//motor derecho hacia adelante
    }
    else {
      move(1, speedturn, 1);//motor derecho hacia adelante
      move(0, speedturn, 0);//motor derecho hacia adelante
    }
    qtr.calibrate();
    delay(5);
  }
  wait();
  delay(3000); // wait for 2s to position the bot before entering the main loop
}

int lastError = 0;
int position = qtr.readLineWhite(sensors);
int rightMotorSpeed = 0;
int leftMotorSpeed = 0;

void loop()
{
  int path[] = {4,2,2,2,4,4,2,2,1};
  if(path[i]==4){
    while(digitalRead(IR_LEFT)!=0){
      goto pid;
    }
    while(digitalRead(IR_LEFT)==0){
      left();
    }
    i++;
    goto pid;
  }
  else if(path[i]==2){
    while(digitalRead(IR_RIGHT)!=0){
      goto pid;
    }
    while(digitalRead(IR_RIGHT)==0){
      right();
    }
    i++;
    goto pid;
  }
  pid:
  position = qtr.readLineWhite(sensors);

  int error = position - 2500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  rightMotorSpeed = BaseSpeed + motorSpeed;
  leftMotorSpeed = BaseSpeed - motorSpeed;

  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed;
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed;
  if (rightMotorSpeed < 0)rightMotorSpeed = 0;
  if (leftMotorSpeed < 0)leftMotorSpeed = 0;

  move(1, rightMotorSpeed, 1);//motor derecho hacia adelante
  move(0, leftMotorSpeed, 1);//motor izquierdo hacia adelante
}

void wait() {
  digitalWrite(motorpower, LOW);
}

void forward() {
  move(1, BaseSpeed, 1);//motor derecho hacia adelante
  move(0, BaseSpeed, 1);//motor derecho hacia adelante
}

void brake() {
  move(1, BaseSpeed, 0);//motor derecho hacia adelante
  move(0, BaseSpeed, 0);//motor derecho hacia adelante
}

void left() {
  move(1, 0, 1);//motor derecho hacia adelante
  move(0, speedturn, 1);//motor derecho hacia adelante
}

void right() {
  move(1, speedturn, 1);//motor derecho hacia adelante
  move(0, 0, 1);//motor derecho hacia adelante
}

void move(int motor, int speed, int direction) {
  digitalWrite(motorpower, HIGH);
  boolean inPin1 = HIGH;
  boolean inPin2 = LOW;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if (direction == 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if (motor == 0) {
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if (motor == 1) {
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }
}
