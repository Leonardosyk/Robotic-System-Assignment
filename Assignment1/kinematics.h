#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

const double GEAR_RATIO = 29.86; // 电机齿轮比
const double ENCODER_CPR = GEAR_RATIO * 12.0; // 每转编码器的脉冲计数
const double WHEEL_RADIUS = 16.0; // 单位为毫米
const double DISTANCE_BETWEEN_WHEELS = 80.0; // 单位为毫米

int lastEncoderLeft = 0;
int lastEncoderRight = 0;
double positionX = 0.0;
double positionY = 0.0;
double theta = 0.0;

// Class to track robot position.
class Kinematics {

public:
    void update() {
        int deltaEncoderLeft = count_leftEncoder - lastEncoderLeft;
        int deltaEncoderRight = count_rightEncoder - lastEncoderRight;

        double distanceLeft = ((double)deltaEncoderLeft / ENCODER_CPR) * 2.0 * PI * WHEEL_RADIUS;
        double distanceRight = ((double)deltaEncoderRight / ENCODER_CPR) * 2.0 * PI * WHEEL_RADIUS;

        double distanceCenter = (distanceLeft + distanceRight) / 2.0;
        double deltaTheta = (distanceLeft - distanceRight) / DISTANCE_BETWEEN_WHEELS;
    
        double deltaX = distanceCenter * cos(theta);
        double deltaY = distanceCenter * sin(theta);
    
        positionX += deltaX;
        positionY += deltaY;
        theta += deltaTheta;

        // keep theta in the range [-PI, PI)
        while (theta > PI) {
            theta -= 2 * PI;
        }
        while (theta <= -PI) {
            theta += 2 * PI;
        }

        lastEncoderLeft = count_leftEncoder;
        lastEncoderRight = count_rightEncoder;


        
        Serial.print("\nLeft: ");
        Serial.print(deltaEncoderLeft);
        Serial.print("\nRight: ");
        Serial.print(deltaEncoderRight);
//        Serial.print("\nTheta: ");
//        Serial.print(theta);
//        Serial.print("\nPosition X: ");
//        Serial.print(positionX);
//        Serial.print("\nPosition Y: ");
//        Serial.print(positionY);
//        
    }

    double getX() {
        return positionX;
    }

    double getY() {
        return positionY;
    }

    double getTheta() {
        return theta;
    }
};

#endif
