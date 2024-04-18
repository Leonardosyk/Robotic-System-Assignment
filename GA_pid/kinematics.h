// this #ifndef stops this file
// from being included mored than
// once by the com3.14159265358979323846ler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H
extern double time;

double Angle;
extern double Dir;

extern double back_angle_time;
extern double XI;  // 初始X位置
extern double YI;  // 初始Y位置

double thetaI = 0.0;           // 初始角度
int lastLeftEncoderCount = 0;  // 上次左轮编码器的计数
int lastRightEncoderCount = 0;
// Class to track robot position.
class Kinematics_c {
public:
  double back_time;
  double distanceToOrigin;
  double back_angle;


  // 上次右轮编码器的计数
  const double wheelRadius = 16;  // 轮子的半径
  const double l = 41;            // 轮子之间的距离的一半

  const double mmPerCount = (32 * 3.14159265358979323846) / 358.3;  // 线速度
  // Constructor, must exist.
  Kinematics_c() {
  }

  // Use this function to update
  // your kinematics
  void update() {
    int currentLeftEncoderCount = count_lft;     // 读取左轮编码器
    int currentRightEncoderCount = count_right;  // 读取右轮编码器

    double deltaLeft = (currentLeftEncoderCount - lastLeftEncoderCount);
    double deltaRight = (currentRightEncoderCount - lastRightEncoderCount);

    double distance_l = (deltaLeft / 358.3) * 2.0 * 3.14159265358979323846 * wheelRadius;
    double distance_r = (deltaRight / 358.3) * 2.0 * 3.14159265358979323846 * wheelRadius;
    //本次车轮行驶的距离
    double XR_dot = (distance_l + distance_r) / 2.0;
    double thetaR_dot = (distance_l - distance_r) / (2.0 * l);  //旋转的角度


    // 更新全局坐标
    XI += XR_dot * cos(thetaI);
    YI += XR_dot * sin(thetaI);
    Serial.print("XI:");
    Serial.print(XI);
    Serial.print(",");
    Serial.print("YI:");
    Serial.print(YI);
    Serial.print("\n");

    //计算回归角度和旋转时间
    thetaI += thetaR_dot;  //计算累计旋转角度
    while (thetaI > 3.14159265358979323846) {
      thetaI -= 2 * 3.14159265358979323846;
    }
    while (thetaI <= -3.14159265358979323846) {
      thetaI += 2 * 3.14159265358979323846;
    }
    Serial.print(",");
    Serial.print("thetaI:");
    Serial.print(thetaI);
    Serial.print("\n");

    // 更新编码器计数
    lastLeftEncoderCount = currentLeftEncoderCount;
    lastRightEncoderCount = currentRightEncoderCount;
  }
};



#endif
