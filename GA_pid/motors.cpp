#include "motors.h"

Motors::Motors(int leftPwmPin, int leftDirPin, int rightPwmPin, int rightDirPin) {
this->leftPwmPin = leftPwmPin;
this->leftDirPin = leftDirPin;
this->rightPwmPin = rightPwmPin;
this->rightDirPin = rightDirPin;

};
void Motors::initialize() {
pinMode(leftPwmPin, OUTPUT);
pinMode(leftDirPin, OUTPUT);
pinMode(rightPwmPin, OUTPUT);
pinMode(rightDirPin, OUTPUT);
}
void Motors::setMotorPower(int leftPower,int rightPower) {
// 设置左电机的功率
if (leftPower > 255) {
leftPower = 255;
} else if (leftPower < -255) {
leftPower = -255;
}
// 设置左电机的功率
if (rightPower > 255) {
rightPower = 255;
} else if (rightPower < -255) {
rightPower = -255;
}
    // 设置左电机的方向

if (leftPower>=0){
 digitalWrite(leftDirPin, LOW);  // 假设HIGH为正向
}
else{digitalWrite(leftDirPin, HIGH); } // 假设LOW为反向
// 设置右电机的方向
if (rightPower>=0){
  digitalWrite(rightDirPin, LOW); // 假设HIGH为正向
}
else{ digitalWrite(rightDirPin, HIGH);}  // 假设LOW为反向}


analogWrite(leftPwmPin, abs(leftPower));
analogWrite(rightPwmPin, abs(rightPower));
}


