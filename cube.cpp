#include "Arduino.h"
#include "cube.h"
#include <Stepper.h>
#include <ESP32Servo.h>

#define STEP_A_PIN_DIRECTION 32//32
#define STEP_A_PIN_STEP  33//33

#define STEP_A_END_STOP 12 //13

#define STEP_B_PIN_DIRECTION   27//D11
#define STEP_B_PIN_STEP   14//14

#define STEP_B_END_STOP  13//12

#define SERVO_A_DIGITAL_PIN 26 //conver to pwm
#define SERVO_B_DIGITAL_PIN 25 //conver to pwm

#define    SEIZE         12 //12 //8 //10 //12     // angle bias for seizing cube w/o pressure

int CURRENT_STEPS_A = 0;
int CURRENT_STEPS_B = 0;

#define motorInterfaceType 1

Cube::Cube(Servo  downPinch_servo, int downRot_step_pin,  int downRot_dir_pin, Servo  backPinch_servo,  int backRot_step_pin, int backRot_dir_pin){

  _downPinch_servo = downPinch_servo;
  _backPinch_servo = backPinch_servo;

  _backRot_step_pin = backRot_step_pin;
  _backRot_dir_pin = backRot_dir_pin;
  //A
  _downRot_step_pin = downRot_step_pin;
  _downRot_dir_pin =  downRot_dir_pin;
}

void moveDownRot(int steps , bool clockWise){
  if(clockWise){
    digitalWrite(STEP_A_PIN_DIRECTION, LOW);
    for (int x = 0; x <= steps; x++){
      digitalWrite(STEP_A_PIN_STEP, HIGH);
      delayMicroseconds(8000);
      digitalWrite(STEP_A_PIN_STEP, LOW);
      delayMicroseconds(8000);
  }
    
  }else{
    digitalWrite(STEP_A_PIN_DIRECTION, HIGH);
    
      for (int x = 0; x <= steps; x++){
        digitalWrite(STEP_A_PIN_STEP, HIGH);
        delayMicroseconds(8000);
        digitalWrite(STEP_A_PIN_STEP, LOW);
        delayMicroseconds(8000);
    }
  }
  
}

void moveBackRot(int steps , bool clockWise){
  if(clockWise){
    digitalWrite(STEP_B_PIN_DIRECTION, LOW);
    for (int x = 0; x <= steps; x++){
      digitalWrite(STEP_B_PIN_STEP, HIGH);
      delayMicroseconds(8000);
      digitalWrite(STEP_B_PIN_STEP, LOW);
      delayMicroseconds(8000);
  }
    
  }else{
    digitalWrite(STEP_B_PIN_DIRECTION, HIGH);
    
    for (int x = 0; x <= steps; x++){
      digitalWrite(STEP_B_PIN_STEP, HIGH);
      delayMicroseconds(8000);
      digitalWrite(STEP_B_PIN_STEP, LOW);
      delayMicroseconds(8000);
  }
  }
  
}


void Cube::begin(int speed)  {
  pinMode(STEP_A_END_STOP, INPUT_PULLUP);
  pinMode(STEP_B_END_STOP, INPUT_PULLUP);

  pinMode(STEP_A_PIN_STEP, OUTPUT);
  pinMode(STEP_A_PIN_DIRECTION, OUTPUT);
  pinMode(STEP_B_PIN_STEP, OUTPUT);
  pinMode(STEP_B_PIN_DIRECTION, OUTPUT);

  _downPinch_servo.attach(SERVO_A_DIGITAL_PIN, 1000, 2000);           
  _backPinch_servo.attach(SERVO_B_DIGITAL_PIN, 1000, 2000); // modified by richimoeller, values were used during position findservo

  // GRADOS
  _downClose = 80;
  _downOpen = 125;
  _downCW = 176;
  _downCCW = 7;
  _downMid = 92;
  _backClose = 0;
  _backOpen = 70;
  _backCW = 173;
  _backCCW = 4;
  _backMid = 88;
  _speed = speed;

  calibration();
  downArmOpen_backArmCW_downArmClose();
  downArmOpen_backArmCCW_downArmClose();

}





void Cube::calibration(){
  while (true){
    
    if(digitalRead(STEP_A_END_STOP) == LOW){
      Serial.println(" Step a touched");
      moveDownRot(90,true);
      CURRENT_STEPS_A += 90;
      break;
    }else{
      delay(50);
      Serial.println(" Not A touched");
      moveDownRot(1,false);

    }
  }
  while (true){
    if(digitalRead(STEP_B_END_STOP) == LOW){
      Serial.println(" Step B touched");
      moveBackRot(90,true);
      CURRENT_STEPS_B += 90;
      break;
    }else{
      delay(50);
      Serial.println(" Not B touched");
      moveBackRot(1,false);
    }
  }
  
}



void Cube::downSetLimits(int Close, int Open, int CW, int Mid, int CCW)  {
  _downClose = Close;
  _downOpen = Open;
  _downCW = CW;
  _downCCW = CCW;
  _downMid = Mid;
}

void Cube::backSetLimits(int Close, int Open, int CW, int Mid, int CCW)  {
  _backClose = Close;
  _backOpen = Open;
  _backCW = CW;
  _backCCW = CCW;
  _backMid = Mid;

}

void Cube::setSpeed(int speed)  {
  _speed = speed;
}

// -------------------------------------------------------------------------

void Cube::seize()  {
  _downPinch_servo.write(_downClose + SEIZE);
  _backPinch_servo.write(_backClose + SEIZE);
}

void Cube::grip()  {
  _downPinch_servo.write(_downClose);
  _backPinch_servo.write(_backClose);
}



void Cube::gripSoft()  {
  _downPinch_servo.write(_downClose + SEIZE - 2 );
  _backPinch_servo.write(_backClose + SEIZE - 2 );
}

void Cube::reseat()  {
  _downPinch_servo.write(_downClose + SEIZE + 4 );
  _backPinch_servo.write(_backClose + SEIZE + 4 );
  delay(50);
  _downPinch_servo.write(_downClose );
  _backPinch_servo.write(_backClose );
}

void Cube::free()  {
  _downPinch_servo.write(_downClose + SEIZE + 2 );
  _backPinch_servo.write(_backClose + SEIZE + 2 );
}


//  ----< Move functions (F R B L U D) >-----------------------------

void Cube::Fr()    {
  Fr(1);
}
void Cube::Fr2()   {
  Fr(2);
}
void Cube::Frp()   {
  Fr(3);
}

void Cube::L()     {
  L(1);
}
void Cube::L2()    {
  L(2);
}
void Cube::Lp()    {
  L(3);
}

void Cube::R()     {
  R(1);
}
void Cube::R2()    {
  R(2);
}
void Cube::Rp()    {
  R(3);
}

void Cube::U()     {
  U(1);
}
void Cube::U2()    {
  U(2);
}
void Cube::Up()    {
  U(3);
}

void Cube::D()     {
  D(1);
}
void Cube::D2()    {
  D(2);
}
void Cube::Dp()    {
  D(3);
}

void Cube::B()     {
  B(1);
}
void Cube::B2()    {
  B(2);
}
void Cube::Bp()    {
  B(3);
}

void Cube::D(int type)  {             // 1:"D"  2:"D2"  3:"D'"
  grip();
  if      (type == 1)    downArmCW();
  else if (type == 2)  {
    downArmCW();
    downArm_OpenCenterClose();
    delay(200);
    downArmCW();
  }
  else if (type == 3)    downArmCCW();
  downArm_OpenCenterClose();
//  reseat();
}

void Cube::B(int type)  {
  grip();
  if      (type == 1)    backArmCW();
  else if (type == 2)  {
    backArmCW();
    backArm_OpenCenterClose();
    delay(200);
    backArmCW();
  }
  else if (type == 3)    backArmCCW();
  backArm_OpenCenterClose();
}

void Cube::R(int type)  {
  grip();
  backArmOpen_downArmCW_backArmClose();
   // reseat();
  downArm_OpenCenterClose();
   // reseat();
  B(type);
  backArmOpen_downArmCCW_backArmClose();
   // reseat();
  downArm_OpenCenterClose();
   // reseat();
}

void Cube::L(int type)  {
  grip();
  backArmOpen_downArmCCW_backArmClose();
  downArm_OpenCenterClose();
   // reseat();
  B(type);
  backArmOpen_downArmCW_backArmClose();
  downArm_OpenCenterClose();
   // reseat();
}

void Cube::U(int type)  {
  grip();
  downArmOpen_backArmCW_downArmClose();
  backArm_OpenCenterClose();
  downArmOpen_backArmCW_downArmClose();
  backArm_OpenCenterClose();
   // reseat();
  D(type);
  downArmOpen_backArmCW_downArmClose();
  backArm_OpenCenterClose();
  downArmOpen_backArmCW_downArmClose();
  backArm_OpenCenterClose();
   // reseat();
}

void Cube::Fr(int type)  {
  grip();
  backArmOpen_downArmCW_backArmClose();
   // reseat();
  downArm_OpenCenterClose();
   // reseat();
  backArmOpen_downArmCW_backArmClose();
   // reseat();
  downArm_OpenCenterClose();
   // reseat();
  B(type);
  backArmOpen_downArmCW_backArmClose();
   // reseat();
  downArm_OpenCenterClose();
   // reseat();
  backArmOpen_downArmCW_backArmClose();
   // reseat();
  downArm_OpenCenterClose();
   // reseat();
}




//  ----< Scan functions (f r b l u d) >-----------------------------
// richimoeller: entered a delay of 500ms here to allow for a clean color recognition shot
void Cube::scanBack()  {
  delay(500);
  y(1);

//  grip();
// gripSoft();
}

void Cube::scanLeft()  {
  delay(500);
  y(1);
}

void Cube::scanFront()  {
  delay(500);
  y(1);
}

void Cube::scanRight()  {
  delay(500);
  x(3);
}

void Cube::scanUp()  {
  delay(500);
  x(2);
}

void Cube::scanDown()  {
  delay(500);
  x(3);
  y(3);
}

void Cube::scanFront2()  {
 // x(3);
 // y(3);
}

//  x : rotate the entire Cube in the direction of R
//  y : rotate the entire Cube in the direction of U
//  z : rotate the entire Cube in the direction of F

void Cube::x(int type)  {             // type=1:"x"  2:"x2"  3:"x'"
  grip();
  backArmOpen_downArmCCW_backArmClose();
  downArmOpen();
  downArmCenter();
  if      (type == 1)    backArmCCW();
  else if (type == 2)    {
    backArmCW();
    downArmClose();
    backArm_OpenCenterClose();
    downArmOpen();
    backArmCW();
  }
  else if (type == 3)    backArmCW();
  downArmClose();
  backArmOpen();
  backArmCenter();
  downArmCW();
  backArmClose();
  downArm_OpenCenterClose();
  reseat();
  gripSoft();
}

void Cube::y(int type)  {
  grip();
  if      (type == 1)    backArmOpen_downArmCCW_backArmClose();
  else if (type == 3)    backArmOpen_downArmCW_backArmClose();
  downArm_OpenCenterClose();
  reseat();
  gripSoft();
}

void Cube::z(int type)  {
  grip();
  if      (type == 1)    downArmOpen_backArmCCW_downArmClose();
  else if (type == 3)    downArmOpen_backArmCW_downArmClose();
  backArm_OpenCenterClose();
  reseat();
  gripSoft();
}



//  ----< Utilities (private) >-----------------------------
void Cube::downArmCW()  {
  Serial.println("Down Arm CW");
  downArmCenter();
  moveDownRot(60,true);
  CURRENT_STEPS_A += 50;
}



void Cube::downArmCCW()  {
 Serial.println("Down Arm CCW");
  downArmCenter();
  moveDownRot(50,false);
  CURRENT_STEPS_A -= 50;
}

void Cube::downArmCenter()  {
  Serial.println("Down Arm Center");
  int stepsToMake = 0;
  if (CURRENT_STEPS_A >= 90){
    while (CURRENT_STEPS_A >= 90){
      CURRENT_STEPS_A = CURRENT_STEPS_A - 1;
      stepsToMake++;
    }
    delay(50);
    moveDownRot(stepsToMake,false);

  }else{
    while(CURRENT_STEPS_A <= 90){
      CURRENT_STEPS_A = CURRENT_STEPS_A + 1;
      stepsToMake++;
    }
    delay(50);
    moveDownRot(stepsToMake,true);
  }

}

void Cube::backArmCW()  {
  Serial.println("Back Arm CW");
  backArmCenter();
  moveBackRot(65,true);
  CURRENT_STEPS_B += 65;
}

void Cube::backArmCCW()  {
  Serial.println("Back Arm CCW");
  backArmCenter();
  moveBackRot(40,false);
  CURRENT_STEPS_B -= 40;
}

void Cube::backArmCenter()  {
  Serial.println("Back Arm Center");
  int stepsToMake = 0;
  if (CURRENT_STEPS_B >= 90){
    while (CURRENT_STEPS_B >= 90){
      CURRENT_STEPS_B = CURRENT_STEPS_B - 1;
      stepsToMake++;
    }
    delay(50);
    moveBackRot(stepsToMake,false);
 
    // _backRot_stepper.runToPosition();

  }else{
    while(CURRENT_STEPS_B <= 90){
      CURRENT_STEPS_B = CURRENT_STEPS_B + 1;
      stepsToMake++;
    }
    moveBackRot(stepsToMake,true);

  }
}

void Cube::downArmOpen()  {
  _downPinch_servo.write(_downOpen);
}

void Cube::downArmClose()  {
  _downPinch_servo.write(_downClose);
}

void Cube::backArmOpen()  {
  _backPinch_servo.write(80);
}

void Cube::backArmClose()  {
  _backPinch_servo.write(10);
}

void Cube::downArm_OpenCenterClose()  {
  downArmOpen();
  downArmCenter();
  downArmClose();
  reseat();
}

void Cube::backArm_OpenCenterClose()  {
  backArmOpen();
  backArmCenter();
  backArmClose();
  reseat();
}

void Cube::backArmOpen_downArmCW_backArmClose()  {
  backArmOpen();
  downArmCW();
  backArmClose();
  reseat();
}

void Cube::backArmOpen_downArmCCW_backArmClose()  {
  backArmOpen();
  downArmCCW();
  backArmClose();
  reseat();
}

void Cube::downArmOpen_backArmCW_downArmClose()  {
  downArmOpen();
  backArmCW();
  downArmClose();
  reseat();
}

void Cube::downArmOpen_backArmCCW_downArmClose()  {
  downArmOpen();
  backArmCCW();
  downArmClose();
  reseat();
}
