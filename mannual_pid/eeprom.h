#ifndef eeprom_h
#define eeprom_h
#include <Arduino.h>


// EEPROM的写入地址
int eepromAddress = 0;
//  // 读取速度和方向输入
//  float speedInput_L = readSpeed_L();
//  float speedInput_R = readSpeed_R();
//  float directionInput = count_rightEncoder - count_leftEncoder;
//  
//  // 如果有有效的速度读数
//  if (speedInput_L != -1 && speedInput_R != -1) {
//    // 计算适应度所需的参数
//    float wheelSpeedError = calculateSpeedError(speedInput_L, speedInput_R);//左右线速度与目标速度之差然后求和的平均值
//    float encoderDeviation = calculateEncoderDeviation(count_rightEncoder, count_leftEncoder);//左右编码器计数之差
//    float responseTime = 0.0;  // 根据系统反应计算
//    
//    // 运行遗传算法的单次迭代
//    gaController.runGeneticAlgorithm(wheelSpeedError, encoderDeviation, responseTime);
//
//    // 应用最优PID参数
//    Individual best = gaController.getBestIndividual();
//    spd_pid_left.initialize(best.directionPID.P, best.directionPID.I, best.directionPID.D);   // 方向PID参数
//    spd_pid_right.initialize(best.directionPID.P, best.directionPID.I, best.directionPID.D);  // 方向PID参数
//    speed_pid.initialize(best.speedPID.P, best.speedPID.I, best.speedPID.D);         // 速度PID参数
//
//    // 更新PID控制器
//    float directionOutput = spd_pid_left.update(directionInput, directionSetpoint);
//    float speedOutput = speed_pid.update((speedInput_L + speedInput_R) / 2.0, speedSetpoint);
//    applyMotorControl(speedOutput - directionOutput, speedOutput + directionOutput);
//  }
