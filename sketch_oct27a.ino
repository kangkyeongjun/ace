#include <SoftwareSerial.h>

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <AFMotor.h>
#include <Wire.h> 

#define HC06 Serial3


#include "math.h"

ros::NodeHandle  nh;



AF_DCMotor motor1(1);              // 모터(좌): M1에 연결
AF_DCMotor motor2(2);              // 모터(우): M2에 연결      
AF_DCMotor motor3(3);

//엔코더
//오른쪽 바퀴
const byte encoderPinA = 18;//A pin -> the interrupt pin 5, 파란선
const byte encoderPinB = 16;//B pin -> 흰색선(녹)
//왼쪽 바퀴
const byte encoderPinC = 19;//A pin -> the interrupt pin 4, 파란선
const byte encoderPinD = 17;//B pin -> 흰색선(녹)

int encoderPos = 0;
int encoderPosL = 0;
byte encoder0PinALast;
byte encoder0PinALastL;

//바퀴
boolean DirectionL = FORWARD;
boolean Direction = BACKWARD;
int standardSpeedR = 90;
int fixedSpeedL =standardSpeedR;

//몸체
int standardDeg = 0; // 현재 각도
#define addr 0x1E

//왼쪽바퀴 스피드 감시
void wheelSpeedL(){  
  int Lstate = digitalRead(encoderPinC);

  if((encoder0PinALastL == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoderPinD);

    if(val == LOW && DirectionL)
      DirectionL = false; //Reverse

      else if(val == HIGH && !DirectionL)
      DirectionL = true;  //Forward
  }

  encoder0PinALastL = Lstate;

  if(!DirectionL)  
    encoderPosL++;
  else  encoderPosL--;

  if(encoderPosL > 10000)
    encoderPosL = encoderPosL - 10000;

  else if(encoderPosL < -10000)
    encoderPosL = encoderPosL + 10000;
}


//오른쪽바퀴 스피드 감시
void wheelSpeedR(){  
  int Lstate = digitalRead(encoderPinA);

  if((encoder0PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoderPinB);

    if(val == LOW && Direction)
      Direction = false; //Reverse

      else if(val == HIGH && !Direction)
      Direction = true;  //Forward
  }
  encoder0PinALast = Lstate;

  if(!Direction)  
    encoderPos++;

  else  encoderPos--;

  if(encoderPos > 10000)
    encoderPos = encoderPos - 10000;

  else if(encoderPos < -10000)
    encoderPos = encoderPos + 10000;
}


void speedGap(){
  float gap = abs(encoderPosL) - abs(encoderPos); // 왼쪽 바퀴와 오른쪽 바퀴 스피드를 비교

  if(gap > 10){ // 왼쪽바퀴가 더 빠름
    fixedSpeedL = standardSpeedR * (100 - abs(gap)/10) / 100; //속도 줄이기
  }

  else if(gap < -10){ // 왼쪽바퀴가 더 느림
    fixedSpeedL = standardSpeedR * (100 + abs(gap)/10) / 100; //속도 높이기
  }

  //왼쪽바퀴 스피드 수정
  motor2.setSpeed(fixedSpeedL);


  // 오른쪽바퀴 스피드 수정(갭이 너무 커질 경우만 실행)
  int p = abs(encoderPos) - abs(encoderPosL);

  if(p > 70){ // 왼쪽바퀴가 너무 느려지면
    //오른쪽바퀴 스피드까지 수정
    motor1.setSpeed(standardSpeedR - (p/70 + 1)*5); 
  }
  else if(p < -70){ // 왼쪽바퀴가 너무 빨라지면
    //오른쪽바퀴 스피드까지 수정
    motor1.setSpeed(standardSpeedR + (p/70 + 1)*5); 
  }
}

//각도 얻기
int getDegree(){  
  int x,y,z; //triple axis data

  Wire.beginTransmission(addr);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission(); 


  Wire.requestFrom(addr, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //MSB  x 
    x |= Wire.read(); //LSB  x
    z = Wire.read()<<8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read()<<8; //MSB y
    y |= Wire.read(); //LSB y
  }  

  float deg = 0;
  if(x==0 && y < 0)deg = 90.0;
  else if(x==0 && y >0)deg = 270.0;
  else if(x < 0 && y == 0)deg = 180.0;
  else if(x > 0 && y == 0)deg = 270.0;
  else if(x < 0 && y < 0)deg = - (atan2(y,x))*180/M_PI;
  else if(x < 0 && y > 0)deg = 360 - (atan2(y,x))*180/M_PI;
  else if(x > 0 && y < 0)deg = -(atan2(y,x))*180/M_PI;  
  else if(x > 0 && y > 0)deg = 360 -(atan2(y,x))*180/M_PI;

  return (int)deg;
}


void motor(const geometry_msgs::Twist& msg){
  float a = msg.angular.z;
  float v = msg.linear.x;

  motor3.run(FORWARD);
  motor3.setSpeed(255);
  if(a==0 && v==0.5){           // 전진
    motor1.run(FORWARD);                              
    motor2.run(BACKWARD);
    motor1.setSpeed(standardSpeedR);
    motor2.setSpeed(fixedSpeedL);
    speedGap();
  }
  else if(a==1 && v==0){           // 우회
    int deg = getDegree();
    int xStandardDeg = (standardDeg + 180)%360; // 보정값을 이용
    int xDeg = (deg + 180)%360; // 보정값을 이용
    if(abs(xStandardDeg - xDeg) >= 88){ // 원래는 90도 간격으로 해야하지만, 멈추는데 약간의 시간이 걸려서 2도 전에 체크를 함.
      motor1.run(BACKWARD);                              
      motor2.run(BACKWARD);
      motor1.setSpeed(standardSpeedR);
      motor2.setSpeed(fixedSpeedL);

    }
    motor1.run(BACKWARD);                              
    motor2.run(BACKWARD);
    motor1.setSpeed(standardSpeedR);
    motor2.setSpeed(fixedSpeedL);

  }
  else if(a==-1 && v==0){           // 좌회
    int deg = getDegree();
    int xStandardDeg = (standardDeg + 180)%360; // 보정값을 이용
    int xDeg = (deg + 180)%360; // 보정값을 이용
    if(abs(xStandardDeg - xDeg) <= 88){ // 원래는 90도 간격으로 해야하지만, 멈추는데 약간의 시간이 걸려서 2도 전에 체크를 함.
      motor1.run(FORWARD);                              
      motor2.run(FORWARD);
      motor1.setSpeed(standardSpeedR);
      motor2.setSpeed(fixedSpeedL);
    }

  }
  else if(a==1 && v==0.5){      // 우회전
    motor2.run(BACKWARD);                              
    motor1.run(RELEASE);
    motor2.setSpeed(standardSpeedR - 5); 
  }
  else if(a==-1 && v==0.5){     // 좌회전

    motor1.run(FORWARD);                              
    motor2.run(RELEASE);
    motor1.setSpeed(standardSpeedR - 5);
  }

  else if(a==0 && v==-0.5){     // 후진
    motor1.run(BACKWARD);                              
    motor2.run(FORWARD);
    motor1.setSpeed(standardSpeedR);
    motor2.setSpeed(fixedSpeedL);
    speedGap();
  }
  else if(a==-1 && v==-0.5){     // 오른쪽으로 후진
    motor1.run(BACKWARD);                              
    motor2.run(FORWARD);
    motor1.setSpeed(standardSpeedR);
    motor2.setSpeed(standardSpeedR-10);
  }
  else if(a==-1 && v==-0.5){    // 왼쪽으로 후진
    motor1.run(BACKWARD);                              
    motor2.run(FORWARD);
    motor1.setSpeed(standardSpeedR-10);
    motor2.setSpeed(standardSpeedR);
  }



}




ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", motor);   // 서브스크라이브

void setup(){
  //엔코더 설정
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(18, wheelSpeedR, CHANGE); 

  pinMode(encoderPinD, INPUT_PULLUP);
  attachInterrupt(19, wheelSpeedL, CHANGE); 

  Wire.beginTransmission(addr); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();


  //Serial.begin(115200);

  HC06.begin(9600);

  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  
  nh.spinOnce();
  
  if (HC06.available())
  {

    char cmd = (char)HC06.read();


    motor3.run(FORWARD);
    motor3.setSpeed(255);


    if(cmd=='w')
    {
      motor1.run(FORWARD);                              
      motor2.run(BACKWARD);
      motor1.setSpeed(standardSpeedR);
      motor2.setSpeed(fixedSpeedL);
      speedGap();
    }
    else if(cmd == 'a')
    {
      int deg = getDegree();
      int xStandardDeg = (standardDeg + 180)%360; // 보정값을 이용
      int xDeg = (deg + 180)%360; // 보정값을 이용
      if(abs(xStandardDeg - xDeg) <= 88){ // 원래는 90도 간격으로 해야하지만, 멈추는데 약간의 시간이 걸려서 2도 전에 체크를 함.
        motor1.run(FORWARD);                              
        motor2.run(FORWARD);
        motor1.setSpeed(standardSpeedR);
        motor2.setSpeed(fixedSpeedL);
      }

    }
    else if(cmd == 'd')
    {
      int deg = getDegree();
      int xStandardDeg = (standardDeg + 180)%360; // 보정값을 이용
      int xDeg = (deg + 180)%360; // 보정값을 이용
      if(abs(xStandardDeg - xDeg) >= 88){ // 원래는 90도 간격으로 해야하지만, 멈추는데 약간의 시간이 걸려서 2도 전에 체크를 함.
        motor1.run(BACKWARD);                              
        motor2.run(BACKWARD);
        motor1.setSpeed(standardSpeedR);
        motor2.setSpeed(fixedSpeedL);

      }
    }
    else if(cmd == 's')
    {
      motor1.run(BACKWARD);                              
      motor2.run(FORWARD);
      motor1.setSpeed(standardSpeedR);
      motor2.setSpeed(fixedSpeedL);
      speedGap();

    }



    else if (cmd == 'o'){
      
  

    }
    
    else if (cmd == 'x'){
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
    
    }


  }


}




