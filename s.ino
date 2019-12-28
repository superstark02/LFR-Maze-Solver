#include <QTRSensors.h>
#define Kp 0.1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 3
#define MaxSpeed 200
#define BaseSpeed 120
#define speedturn 120
#define NUM_SENSORS  6
#define rightMotor1 8
#define rightMotor2 9
#define rightMotorPWM 3
#define leftMotor1 5
#define leftMotor2 6
#define leftMotorPWM 10
#define motorpower 7

#define IR_BACK 12
#define IR_RIGHT A1
#define IR_LEFT A0
#define IR_TOP 4
#define buzzer 2

QTRSensors qtr;
uint16_t sensors[NUM_SENSORS];

int path[20];
int node = 0;

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
  pinMode(IR_BACK,INPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A7, A6, A5, A4, A3, A2}, NUM_SENSORS);
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
    delay(20);
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
  qtr.readLineWhite(sensors);
  
  if (sensors[0] > 700 && sensors[1] > 700 && sensors[2] > 700 && sensors[3] > 700 && sensors[4] > 700 && sensors[5] > 700 && digitalRead(IR_LEFT)==1 && digitalRead(IR_RIGHT)==1) {
    wait(); delay(10); brake(); delay(50); wait();
    delay(300);

    while (digitalRead(IR_RIGHT) == 1) {
      move(1, 70, 0);//motor derecho hacia adelante
      move(0, 70, 1);//motor derecho hacia adelante
    }
    while (digitalRead(IR_RIGHT) == 0) {
      move(1, speedturn, 0);//motor derecho hacia adelante
      move(0, speedturn, 1);//motor derecho hacia adelante
    }
    path[node] = 3;
    node++;
    goto pid;
  }

  if (digitalRead(IR_RIGHT) == 0 && digitalRead(IR_LEFT) == 0) {
    goto right;
  }

  else if (digitalRead(IR_RIGHT) == 0) {
    right:
    brake(); delay(20); wait();
    while (digitalRead(IR_RIGHT) == 0) {
      right();
    }
    while(digitalRead(IR_LEFT)==0){
      right();
    }
        if (digitalRead(IR_LEFT)==0 && digitalRead(IR_BACK)==0){
          wait(); delay(10); brake(); delay(20); wait();
          digitalWrite(buzzer, HIGH); delay(1000); digitalWrite(buzzer, LOW);
          while (digitalRead(IR_LEFT) == 0) {
          move(1, 70, 1);//motor derecho hacia adelante
          move(0, 70, 0);//motor derecho hacia adelante
          }
          while (digitalRead(IR_LEFT) == 1) {
            move(1, 70, 1);//motor derecho hacia adelante
            move(0, 70, 0);//motor derecho hacia adelante
          }
          while(digitalRead(IR_LEFT) == 0){
            move(1, 70, 1);//motor derecho hacia adelante
            move(0, 70, 0);//motor derecho hacia adelante
          }
            wait();
            digitalWrite(buzzer,HIGH); delay(500); digitalWrite(buzzer,LOW);
            furtherRun();
            mazeSolve();
        }
        
    path[node] = 2;
    node++;
    goto pid;
  }

  else if (digitalRead(IR_LEFT) == 0) {
    brake(); delay(20); wait();
    while (digitalRead(IR_LEFT) == 0) {
      left();
      if (digitalRead(IR_RIGHT) == 0) {
        brake(); delay(20); wait();
        while (digitalRead(IR_RIGHT) == 0) {
          right();
        }
        path[node] = 1;
        node++;
        goto pid;
      }
    }
      path[node] = 4;
      node++;
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

void furtherRun(){
  int runTime = 0;
  while(runTime<=node){
      qtr.readLineWhite(sensors);
      if (sensors[0] > 700 && sensors[1] > 700 && sensors[2] > 700 && sensors[3] > 700 && sensors[4] > 700 && sensors[5] > 700 && digitalRead(IR_LEFT)==1 && digitalRead(IR_RIGHT)==1) {
        wait(); delay(10); brake(); delay(50); wait();
        delay(300);
    
        while (digitalRead(IR_RIGHT) == 1) {
          move(1, 70, 0);//motor derecho hacia adelante
          move(0, 70, 1);//motor derecho hacia adelante
        }
        while (digitalRead(IR_RIGHT) == 0) {
          move(1, speedturn, 0);//motor derecho hacia adelante
          move(0, speedturn, 1);//motor derecho hacia adelante
        }
        runTime++;
        goto pid;
      }
      if (digitalRead(IR_RIGHT) == 0 && digitalRead(IR_LEFT) == 0) {
        goto right;
      }
    
      else if (digitalRead(IR_RIGHT) == 0) {
        right:
        brake(); delay(20); wait();
        while (digitalRead(IR_RIGHT) == 0) {
          right();
        }
        while(digitalRead(IR_LEFT)==0){
          right();
        }
            if (digitalRead(IR_LEFT)==0 && digitalRead(IR_BACK)==0){
              wait(); delay(10); brake(); delay(20); wait();
              digitalWrite(buzzer, HIGH); delay(1000); digitalWrite(buzzer, LOW);
              while (digitalRead(IR_LEFT) == 0) {
              move(1, 70, 1);//motor derecho hacia adelante
              move(0, 70, 0);//motor derecho hacia adelante
              }
              while (digitalRead(IR_LEFT) == 1) {
                move(1, 70, 1);//motor derecho hacia adelante
                move(0, 70, 0);//motor derecho hacia adelante
              }
              while(digitalRead(IR_LEFT) == 0){
                move(1, 70, 1);//motor derecho hacia adelante
                move(0, 70, 0);//motor derecho hacia adelante
              }
                wait();
                digitalWrite(buzzer,HIGH); delay(500); digitalWrite(buzzer,LOW);
            }
        runTime++;
        goto pid;
      }
    
      else if (digitalRead(IR_LEFT) == 0) {
        brake(); delay(20); wait();
        while (digitalRead(IR_LEFT) == 0) {
          left();
          if (digitalRead(IR_RIGHT) == 0) {
            brake(); delay(20); wait();
            while (digitalRead(IR_RIGHT) == 0) {
              right();
            }
            runTime++;
            goto pid;
          }
        }
          runTime++;
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
  return;
}

void mazeSolve() {
  while(search(3)){
    for(int j = 0; j<node; j++){   
        if (path[j] == 4 && path[j + 1] == 3 && path[j + 2] == 2) {   
          path[j + 2] = 3;
          Delete(j+1);
          Delete(j);
          j--;
        }
        else if (path[j] == 2 && path[j + 1] == 3 && path[j + 2] == 1) {
          path[j + 2] = 4;
          Delete(j+1);
          Delete(j);
          j--;
        }
        else if (path[j] == 2 && path[j + 1] == 3 && path[j + 2] == 4) {
          path[j + 2] = 3;
          Delete(j+1);
          Delete(j);
          j--;
        }
        else if (path[j] == 1 && path[j + 1] == 3 && path[j + 2] == 2) {
          path[j + 2] = 4;
          Delete(j+1);
          Delete(j);
          j--;
        }
        else if (path[j] == 1 && path[j + 1] == 3 && path[j + 2] == 1) {
          path[j + 2] = 3;
          Delete(j+1);
          Delete(j);
          j--;
        }
        else if (path[j] == 2 && path[j + 1] == 3 && path[j + 2] == 2) {
          path[j + 2] = 1;
          Delete(j+1);
          Delete(j);
          j--;
        }
    }
  }


  digitalWrite(buzzer, HIGH); delay(1000); digitalWrite(buzzer, LOW);
  delay(5000);
  for (int j = 0; j < 2 ; j++) {
    digitalWrite(buzzer, HIGH);
    delay(1000);
    digitalWrite(buzzer, LOW);
    delay(1000);
  }
  
  ////////////////////////////Actual Run//////////////////////////////
  int k = 0;
  while(1){
  if(path[k]==0){
    while(digtalRead(IR_RIGHT)==1||DigitalRead(IR_LEFT)==1){
      goto pid;
    }
    wait();
    break;
  }
    
  else if(path[k]==4){
    while(digitalRead(IR_LEFT)!=0){
      goto pid;
    }
    brake(); delay(10); wait();
    while(digitalRead(IR_LEFT)==0){
      left();
    }
    k++;
    goto pid;
  }
  else if(path[k]==2){
    while(digitalRead(IR_RIGHT)!=0){
      goto pid;
    }
    brake(); delay(10); wait();
    while(digitalRead(IR_RIGHT)==0){
      right();
    }
    k++;
    goto pid;
  }
  else if(path[k]==1){
    while(digitalRead(IR_LEFT)!=0 && digitalRead(IR_RIGHT)!=0){
      goto pid;
    }
    brake(); delay(20); wait(); delay(300);
    if(digitalRead(IR_LEFT)==0){
      while(digitalRead(IR_LEFT)==0){
        forward();
      }
      brake(); delay(20); wait(); delay(300);
      while(digitalRead(IR_RIGHT)==0){
        right();
      }
      k++;
      goto pid;
    }
    else if(digitalRead(IR_RIGHT)==0){
      while(digitalRead(IR_RIGHT)==0){
        forward();
      }
      brake(); delay(20); wait(); delay(300);
      while(digitalRead(IR_LEFT)==0){
        left();
      }
      k++;
      goto pid;
    }
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

  while(1){
    wait(); digitalWrite(buzzer, HIGH);
  }
}

int search(int back){
  for(int j = 0; j<node; j++){
    if(path[j]==back){
      return 1;
    }
  }
  return 0;
}

void Delete(int index){
  for(int x=index; x<node; x++){
    path[x] = path[x+1];
    path[x+1] = 0;
  }
  return;
}

void wait() {
  digitalWrite(motorpower, LOW);
}

void forward() {
  move(1, 70, 1);//motor derecho hacia adelante
  move(0, 70, 1);//motor derecho hacia adelante
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
