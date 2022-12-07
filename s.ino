#include <QTRSensors.h>
#define Kp 0.01 
#define Kd 0.5 // ( Note: Kp < Kd) 
#define AMaxSpeed 250
#define ABaseSpeed 150 
#define Aspeedturn 100
#define MaxSpeed 150
#define BaseSpeed 100 
#define speedturn 50
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
#define white 350
#define black 900

QTRSensors qtr;

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(buzzer, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A7,A6,A5,A4,A3,A2}, NUM_SENSORS);
  qtr.setEmitterPin(13);
  
  delay(2000);
  
  int i;
  for (int i = 0; i < 40; i++) 
  {
    if(i<10||i<20){
      move(1, speedturn, 0);//motor derecho hacia adelante
      move(0, speedturn, 1);//motor derecho hacia adelante
    }
    else{
      move(1, speedturn, 1);//motor derecho hacia adelante
      move(0, speedturn, 0);//motor derecho hacia adelante
    }
    qtr.calibrate();
    delay(100);
  }
  wait();
  delay(5000); // wait for 2s to position the bot before entering the main loop 
}  

int lastError = 0;
unsigned int sensors[NUM_SENSORS];
int position = qtr.readLineWhite(sensors);
uint16_t rawValues[NUM_SENSORS];

int rightMotorSpeed = 0;
int leftMotorSpeed = 0;

void loop()
{ 
  qtr.read(rawValues);
  if(sensors[0] > black && sensors[1] > black && sensors[2] > black && sensors[3] > black && sensors[4] > black && sensors[5] > black){
    wait();
    qtr.readLineWhite(sensors);
    apply_break();
    delay(1000);
    qtr.readLineWhite(sensors);
    while(sensors[2] > black && sensors[3] > black){
      qtr.readLineWhite(sensors);
      move(1, speedturn, 0);//motor derecho hacia adelante
      move(0, speedturn, 1);//motor derecho hacia adelante
    }
    apply_break();
    wait();
  }

  qtr.readLineWhite(sensors);
  position = qtr.readLineWhite(sensors);

  if(digitalRead(IR_RIGHT) && !digitalRead(IR_LEFT)){
    digitalWrite(buzzer, HIGH);
    apply_break();
    wait();
    delay(1000);
    digitalWrite(buzzer, LOW);
    position = qtr.readLineWhite(sensors);
    while(sensors[2] > black && sensors[3] > black){
      position = qtr.readLineWhite(sensors);
      move(1, speedturn, 0);//motor derecho hacia adelante
      move(0, speedturn, 1);//motor derecho hacia adelante
    }
    apply_break();
    wait();
  }

  if(!digitalRead(IR_RIGHT) && digitalRead(IR_LEFT)){
    digitalWrite(buzzer, HIGH);
    apply_break();
    wait();
    delay(1000);
    digitalWrite(buzzer, LOW);
    position = qtr.readLineWhite(sensors);
    while(sensors[2] > black && sensors[3] > black){
      position = qtr.readLineWhite(sensors);
      move(1, speedturn, 1);//motor derecho hacia adelante
      move(0, speedturn, 0);//motor derecho hacia adelante
    }
    apply_break();
    wait();
  }

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

void wait(){
  digitalWrite(motorpower,LOW);
}

void forward(){
  move(1, BaseSpeed, 1);//motor derecho hacia adelante
  move(0, BaseSpeed, 1);//motor derecho hacia adelante
}

void left(){
      move(1, speedturn, 1);//motor derecho hacia adelante
      move(0, speedturn, 0);//motor derecho hacia adelante
}

void right(){
      move(1, speedturn, 0);//motor derecho hacia adelante
      move(0, speedturn, 1);//motor derecho hacia adelante
}

void move(int motor, int speed, int direction){
  digitalWrite(motorpower,HIGH);
  boolean inPin1=HIGH;
  boolean inPin2=LOW;
  
  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }  
  if(direction == 0){
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if(motor == 0){
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if(motor == 1){
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }  
}

void apply_break(){
  move(0, 100, 0);//motor derecho hacia adelante
  move(1, 100, 0);//motor izquierdo hacia adelante
  delay(10);
  wait();
}

void turn(char direction){
  digitalWrite(buzzer, HIGH);

  qtr.read(rawValues);
  while(rawValues[2] > black && rawValues[3] > black){
    qtr.read(rawValues);
    if(direction == 'r'){
      right();
    }
    else if(direction == 'l'){
      left();
    }
    
  }

  digitalWrite(buzzer, LOW);
}
