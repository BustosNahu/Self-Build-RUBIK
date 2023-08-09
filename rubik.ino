#define VERSION       "Cube Mover V1.3  @kas2014\n"
#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include "cube.h"
// ---------- user adjustments -------------------
#define    DOWN_CLOSE          80
#define    DOWN_OPEN          125

#define    DOWN_CW            176
#define    DOWN_MID            93

#define    DOWN_CCW             5

#define    BACK_CLOSE         0
#define    BACK_OPEN          70

#define    BACK_CW            176
#define    BACK_MID            88
#define    BACK_CCW             2

#define    LOW_SPEED           50
#define    HI_SPEED            80
// -----------------------------------------------

#define STEP_A_PIN_DIRECTION 32//32
#define STEP_A_PIN_STEP  33//33

#define STEP_A_END_STOP 13 //13

#define STEP_B_PIN_DIRECTION   27//D11
#define STEP_B_PIN_STEP   14//14

#define STEP_B_END_STOP  12//12

#define SERVO_A_DIGITAL_PIN 26 //conver to pwm
#define SERVO_B_DIGITAL_PIN 25 //conver to pwm

#define    bipPin               0           // buzzer
#define    myRX                 6
#define    myTX                 7
#define    ledPin              0

#define    STX               0x02             // serial data frame delimiters
#define    ETX               0x03
#define motorInterfaceType 1
BluetoothSerial SerialBT;
 

boolean    DEBUG_SERIAL =         false;


Servo  servo_a;
Servo  servo_b;
Servo  servo_c;
Servo  servo_d;


Cube myCube(servo_a,servo_c ,servo_b,servo_d);

char cmd[128];                                 // bytes received buffer

void setup() {
  Serial.begin(57600);
  SerialBT.begin("ESP-nahu-Rubik"); //Bluetooth device name
  pinMode(2,OUTPUT);

  myCube.begin(HI_SPEED);                 // set HIGH servo's speed 
 
  //Serial.println("OK bis hierher_1");

}

void loop() {       // uncomment for checking moves
  
  
  if (DEBUG_SERIAL)  {
    if(getSerialMonitor())    parseData();        // obtain commands from Serial Monitor
  }  else  {
    if(getSerialData())       parseData();        // obtain commands through Bluetooth
  } 
}

// ---------------------------

boolean getSerialData()  {
  if(SerialBT.available())  {  
    digitalWrite(2,HIGH);                                     // data received from smartphone
    delay(2);
    cmd[0] =  SerialBT.read();
      if(cmd[0] == STX)  {
      int i=1;      
      while(SerialBT.available())  {
        delay(1);
        cmd[i] = SerialBT.read();
        if(cmd[i]>'u' || i>124)    {bip(100, 2); return false;}    // Communication error  XXX reinitialiser à zero <<<
        if((cmd[i]==ETX))          {bip(200, 1);  return true; } 
        i++;
      }
    }
    else {bip(100, 3); return false;}
  }
  return false;
}

boolean getSerialMonitor()  {  // Serial Monitor setting: Newline
  if(Serial.available())  {
    for(int i=0; i<124; i++)    cmd[i] = 0;
    int n = Serial.readBytesUntil('\n', cmd, 124);
   cmd[n+1] = ETX;
   return true;
  }
 return false; 
}
//p32

void parseData()    { // parseData(cmd)
  int i = 0;
  String progress = "";
  Serial.println(cmd);
  while (cmd[i] != ETX) {
    SerialBT.print(cmd[i]); SerialBT.print(" ");
//    Serial.println(cmd[i]);
    switch(cmd[i])  {

      // Test commands  --------------------------------------------------------------
//      1: downArmOpen      6: backArmOpen
//      2: downArmClose     7: backArmClose
//      3: downArmCW        8: backArmCW
//      4: downArmCenter    9: backArmCenter
//      5: downArmCCW       0: backArmCCW
//      T: seize     release gripper pressure


      // Move commands  ------------------------------------------------------------
      case 'R':                                                    //  'R' moves
        switch(cmd[i+1])  {
          case '2':
            //Serial.print("R2 ");
            myCube.R2();
            break;
          case 39: // ASCII 39: apostrophe character
            //Serial.print("R' ");
            myCube.Rp();
            break;
          default:
            //Serial.print("R ");
            myCube.R();
            break;
        }
        break;
      case 'L':                                                    //  'L' moves
        switch(cmd[i+1])  {
          case '2':
            //Serial.print("L2 ");
            myCube.L2();
            break;
          case 39:
            //Serial.print("L' ");
            myCube.Lp();
            break;
          default:
            //Serial.print("L ");
            myCube.L();
            break;
        }
        break;
      case 'U':                                                    //  'U' moves
        switch(cmd[i+1])  {
          case '2':
            //Serial.print("U2 ");
            myCube.U2();
            break;
          case 39:
            //Serial.print("U' ");
            myCube.Up();
            break;
          default:
            //Serial.print("U ");
            myCube.U();
            break;
        }
        break;
      case 'D':                                                    //  'D' moves
        switch(cmd[i+1])  {
          case '2':
            //Serial.print("D2 ");
            myCube.D2();
            break;
          case 39:
            //Serial.print("D' ");
            myCube.Dp();
            break;
          default:
            //Serial.print("D ");
            myCube.D();
            break;
        }
        break;
      case 'F':                                                    //  'F' moves
        switch(cmd[i+1])  {
          case '2':
            //Serial.print("F2 ");
            myCube.Fr2();
            break;
          case 39:
            //Serial.print("F' ");
            myCube.Frp();
            break;
          default:
            //Serial.print("F ");
            myCube.Fr();
            break;
        }
        break;
      case 'B':                                                    //  'B' moves
        switch(cmd[i+1])  {
          case '2':
            //Serial.print("B2 ");
            myCube.B2();
            break;
          case 39:
            //Serial.print("B' ");
            myCube.Bp();
            break;
          default:
            //Serial.print("B ");
            myCube.B();
            break;
        }
        break;

      // Scan commands  -----------------------------------------------------------
      case 'f':                                             // Scan Front side
        myCube.scanFront();
        //Serial.print("OKf ");
        break;
      case 'r':                                            // Scan Right side
        myCube.scanRight();
        //Serial.print("OKr");
        break;
      case 'b':                                            // Scan Back side
        myCube.scanBack();
        //Serial.print("OKb");
        break;
      case 'l':                                            // Scan Right side
        myCube.scanLeft();
        //Serial.print("OKl");
        break;
      case 'u':                                            // Scan Up side
        myCube.scanUp();
        //Serial.print("OKu");
        break;
      case 'd':                                            // Scan Down side
        myCube.scanDown();
        //Serial.print("OKd");
        break;
      case 'g':                                           // back to Front side
        myCube.scanFront2();
        //Serial.print("OKg");
        break;

      // Other commands  --------------------------------------------------------------
      case 'T':                                          // release gripper pressure
        myCube.seize();
        bip(40, 2);
        //if (DEBUG_SERIAL)    Serial.print("seize");
        break;
      case 'S':                                         // change move speed
        switch(cmd[i+1])  {
          case '2':
            myCube.setSpeed(HI_SPEED);
            //if (DEBUG_SERIAL)    Serial.print("High Speed");
            break;
          case '1':
            myCube.setSpeed(LOW_SPEED);
            //if (DEBUG_SERIAL)    Serial.print("Low Speed");
            break;
        }
        break;
      case 'V':                                         // bips
        switch(cmd[i+1])  {
          case '4':
            bip(80, 4);
            //if (DEBUG_SERIAL)    Serial.print("bip (4)");
            break;
          case '2':
            bip(80, 2);
            //if (DEBUG_SERIAL)    Serial.print("bip (2)");
            break;
          default:
            bip(80, 1);
            //if (DEBUG_SERIAL)    Serial.print("bip ");
            break;
        }
        break;

      default:
          break;
      }
     i++;
  }
  //if (DEBUG_SERIAL)    Serial.println();
  //bip(20, 2);
}

void bip(int duration, int n)    {            // Bip piezo: duration in ms, n repeats
  for(int i=0; i<n; i++)  {  
     digitalWrite(bipPin, HIGH);        
     delay(duration);
     digitalWrite(bipPin, LOW);         
     delay(75);
  }
}
