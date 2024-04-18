#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"
#include "kinematics.h"
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15
#define FWD LOW
#define REV HIGH
#define FORWARD 1
#define BACKWARD 0
#define TURN_SPEED 20  
const double encoderCountsPerRevolution = ENCODER_CPR;  
const double countsPerDegree = encoderCountsPerRevolution / ( 3.14159265358979323846);

class Motors {
public:
    Motors();
    void setup();
    void moveForward(float LeftPWM,float RightPWM);
    void moveBackward(float speed);
    void turnLeft(float speed = TURN_SPEED);
    void turnRight(float speed = TURN_SPEED);
    void arcTurn(float LeftPWM, float RightPWM);  
    void stopMotors();
    void turnAroundInPlaceRight(float speed = TURN_SPEED);
    void turnAroundInPlaceLeft(float speed = TURN_SPEED);
    void turnAroundInPlace(float speed = TURN_SPEED);
    void setMotorPower(float LeftPWM, float RightPWM);
    void Motors::turnSpecificAngleRight(float speed, float angle);
//    void turnSpecificAngleRight(float speed, double angle);
};

void Motors::setup() {
    pinMode(L_PWM_PIN, OUTPUT);
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(R_PWM_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);
    stopMotors();
}

void Motors::moveForward(float LeftPWM,float RightPWM) {
    analogWrite(L_PWM_PIN, LeftPWM);
    digitalWrite(L_DIR_PIN, FWD);  
    analogWrite(R_PWM_PIN, RightPWM);
    digitalWrite(R_DIR_PIN, FWD);  
}


void Motors::moveBackward(float speed) {
    analogWrite(L_PWM_PIN, speed);
    digitalWrite(L_DIR_PIN, REV);
    analogWrite(R_PWM_PIN, speed);
    digitalWrite(R_DIR_PIN, REV);
}

void Motors::turnLeft(float speed) {
    analogWrite(L_PWM_PIN,0);
    digitalWrite(L_DIR_PIN,FWD);
    analogWrite(R_PWM_PIN,speed);
    digitalWrite(R_DIR_PIN,FWD);

}

void Motors::turnRight(float speed) {
    analogWrite(R_PWM_PIN,0);
    digitalWrite(R_DIR_PIN,FWD);
    analogWrite(L_PWM_PIN,speed);
    digitalWrite(L_DIR_PIN,FWD);
}

void Motors::arcTurn(float LeftPWM, float RightPWM) {
    analogWrite(L_PWM_PIN, LeftPWM);
    analogWrite(R_PWM_PIN, RightPWM);
    digitalWrite(R_DIR_PIN,FWD);
    digitalWrite(L_DIR_PIN,FWD);
}

void Motors::stopMotors() {
    analogWrite(L_PWM_PIN, 0);
    analogWrite(R_PWM_PIN, 0);
}

void Motors::turnAroundInPlace(float speed) {


    
    analogWrite(L_PWM_PIN, speed);
    digitalWrite(L_DIR_PIN, LOW);
    analogWrite(R_PWM_PIN, speed);
    digitalWrite(R_DIR_PIN, HIGH);
    
    delay(1800);

    stopMotors();
}
   

void Motors::turnAroundInPlaceLeft(float speed) {


    
    analogWrite(L_PWM_PIN, speed);
    digitalWrite(L_DIR_PIN, FWD);
    analogWrite(R_PWM_PIN, speed);
    digitalWrite(R_DIR_PIN, REV);
}
    

void Motors::turnAroundInPlaceRight(float speed) {


    analogWrite(L_PWM_PIN, speed);
    digitalWrite(L_DIR_PIN, REV);
    analogWrite(R_PWM_PIN, speed);
    digitalWrite(R_DIR_PIN, FWD);
    
    delay(1700);  

    stopMotors(); 
}
void Motors::setMotorPower(float LeftPWM, float RightPWM) {
    analogWrite(L_PWM_PIN, LeftPWM);
    analogWrite(R_PWM_PIN, RightPWM);
  }

  


void Motors::turnSpecificAngleRight(float speed, float angle) {
    unsigned long rotateTime = abs(angle * (1.8 * 1000 / 3.14159265358979323846));  // 1.8秒为180度，计算特定角度所需的时间


    if (angle < 0) {
        analogWrite(L_PWM_PIN, speed);
        digitalWrite(L_DIR_PIN, LOW);
        analogWrite(R_PWM_PIN, speed);
        digitalWrite(R_DIR_PIN, HIGH);
    } else {

        analogWrite(L_PWM_PIN, speed);
        digitalWrite(L_DIR_PIN, HIGH);
        analogWrite(R_PWM_PIN, speed);
        digitalWrite(R_DIR_PIN, LOW);
    }


    delay(rotateTime);


    analogWrite(L_PWM_PIN, 0);
    analogWrite(R_PWM_PIN, 0);
}

//void Motors::turnSpecificAngleRight(float speed, double angle) {
//    long targetCounts = round(angle * countsPerDegree);
//    long initialLeftCount = count_leftEncoder;
//    long initialRightCount = count_rightEncoder;
//
//    // 根据角度的正负来确定旋转方向
//    if (angle < 0) {
//        // 顺时针旋转
//        analogWrite(L_PWM_PIN, speed);
//        digitalWrite(L_DIR_PIN, LOW);
//        analogWrite(R_PWM_PIN, speed);
//        digitalWrite(R_DIR_PIN, HIGH);
//    } else {
//        // 逆时针旋转
//        analogWrite(L_PWM_PIN, speed);
//        digitalWrite(L_DIR_PIN, HIGH);
//        analogWrite(R_PWM_PIN, speed);
//        digitalWrite(R_DIR_PIN, LOW);
//    }
//
//    // 等待直到完成旋转
//    while (abs(count_leftEncoder - initialLeftCount) < abs(targetCounts) ||
//           abs(count_rightEncoder - initialRightCount) < abs(targetCounts)) {
//        // 检查编码器计数是否达到目标值
//        // 可以在这里添加一些延迟或其他逻辑
//    }
//
//    // 停止旋转
//    analogWrite(L_PWM_PIN, 0);
//    analogWrite(R_PWM_PIN, 0);
//}

#endif
  
