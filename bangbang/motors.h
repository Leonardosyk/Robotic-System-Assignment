#ifndef MOTORS_H
#define MOTORS_H
#include <Arduino.h>

class Motors {
public:
Motors(int leftPwmPin, int leftDirPin, int rightPwmPin, int rightDirPin);
void initialize();
void setMotorPower(int leftPower,int rightPower);

private:
int leftPwmPin;
int leftDirPin;
int rightPwmPin;
int rightDirPin;
};
#endif