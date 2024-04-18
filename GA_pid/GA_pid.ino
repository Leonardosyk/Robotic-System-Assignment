struct RobotData {
  float speedLeft;
  float speedRight;
  long encoderLeft;
  long encoderRight;
  long encoderDifference;
  //左轮PID
  float pidP_L;  // PID的P参数
  float pidI_L;  // PID的I参数
  float pidD_L;  // PID的D参数
  //右轮PID
  float pidP_R;  // PID的P参数
  float pidI_R;  // PID的I参数
  float pidD_R;  // PID的D参数
  //迭代时间
  int Iter_time;
};

#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "timer3.h"
#include "pid.h"
#include <EEPROM.h>
#include "GAPIDController.h"

bool hasStopped = false;  // 全局变量，用于追踪程序是否已经停止

GAPIDController gaController(30, 0.0, 10.0, 0.0, 1.0, 0.0, 10.0);  // 初始化GA控制器

LineSensor_c line_sensors;
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15
#define MAX_SAMPLES 5

Motors motors(L_PWM_PIN, L_DIR_PIN, R_PWM_PIN, R_DIR_PIN);

unsigned long motor_ts;
unsigned long line_sensor_ts;
unsigned long active_threshold;
volatile float l_speed_t3;
volatile float r_speed_3;

PID_c spd_pid_left;
PID_c spd_pid_right;

unsigned long update_ts;
float ave_e1_spd;
long count_rightEncoder_last;
float ave_e0_spd;
long count_leftEncoder_last;
float demand;
unsigned long pid_tms;
int eeprom_address;
unsigned long measurementPeriod = 1000;
float pulses_per_revolution = 358.3;
float revolutions_per_second = 1.2;
float wheelDiameter = 3.1415926535 * 16 * 2;
float encoderPPR = 358.3;
unsigned long sampleInterval = 20;  // Time in milliseconds between encoder readings

// 全局变量
unsigned long lastRecordTime = 0;  // 用于追踪上次记录数据到EEPROM的时间
// 设置目标编码器计数和记录间隔
const int targetCount = 20000;
const unsigned long logInterval = 1000;  // 记录间隔
unsigned long lastLogTime = 0;           // 上次记录的时间
float speed_eeprom_L = 0.0;              //
float speed_eeprom_R = 0.0;
// 设置EEPROM地址初始值
int eepromAddress = 0;

//// 速度PID参数
//float speedSetpoint = 0.5;  // 假设的速度设定值
//float speedInput = 0.0;
//float speedOutput = 0.0;
//float lastSampleTime = 0.0;


// 全局变量
unsigned long lastSampleTime_L = 0;  // 上次左轮速度采样的时间
int lastEncoderCount_L = 0;          // 上次左轮编码器计数
unsigned long lastSampleTime_R = 0;  // 上次右轮速度采样的时间
int lastEncoderCount_R = 0;          // 上次右轮编码器计数
float sumOfSquaredErrors = 0;        // 误差平方和
float sumOfSquaredErrors_L = 0;
float sumOfSquaredErrors_R = 0;
int sampleCount = 0;  //样本迭代次数
int iter_time = 0;//GA迭代次数
//// 全局变量的定义


// 定义按键A的引脚号
#define BUTTON_A_PIN 14

void setup() {
  // 初始化编码器
  setupEncoder0();
  setupEncoder1();

  // 设置按键A的引脚为输入，并启用内部上拉电阻
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  // 初始化GA种群
  gaController.initializePopulation();

  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");

  update_ts = millis();
  ave_e1_spd = 0;
  ave_e0_spd = 0;

  spd_pid_left.reset();
  spd_pid_right.reset();
  pid_tms = millis();//总体时间
}


void loop() {
  // 读取按键A的状态
  int buttonState = digitalRead(BUTTON_A_PIN);

  // 如果按键A被按下（引脚状态为LOW）
  if (buttonState == LOW) {
    // 运行readDataFromEEPROM()函数
    readDataFromEEPROM();

    // 防抖动延时或等待按钮释放
    while (digitalRead(BUTTON_A_PIN) == LOW) {
      // 可以在这里加上短暂的延时来防止抖动，例如：
      delay(50);
    }
  }


  // 如果程序已经停止，则不再执行任何操作
  if (hasStopped) {
    return;
  }
  // 读取当前速度
  float speedInput_L = readSpeed_L();  // 左轮速度
  float speedInput_R = readSpeed_R();  // 右轮速度
  // 计算适应度所需的参数
  //线速度误差(累计误差)
  float wheelSpeedError_L = calculateSpeedError_L(speedInput_L);
  float wheelSpeedError_R = calculateSpeedError_R(speedInput_R);
  sumOfSquaredErrors_L += wheelSpeedError_L * wheelSpeedError_L;  //计算误差平方和
  sumOfSquaredErrors_R += wheelSpeedError_R * wheelSpeedError_R;  //计算误差平方和
  float real_Mp_L = calculateMp_L(speedInput_L);                  //计算超调误差
  float real_Mp_R = calculateMp_R(speedInput_R);                  //计算超调误差
  //反应时间（还没设置）
  sampleCount++;
  float responseTime = 0.0;  // 根据实际反应时间设定
  if (sampleCount >= 10) {
    
    gaController.runGeneticAlgorithm(sumOfSquaredErrors_L, real_Mp_L, responseTime);
    // 应用最优PID参数
    Individual best_L = gaController.getBestIndividual();
    spd_pid_left.initialize(best_L.speedPID.P, best_L.speedPID.I, best_L.speedPID.D);
    gaController.runGeneticAlgorithm(sumOfSquaredErrors_R, real_Mp_R, responseTime);
    Individual best_R = gaController.getBestIndividual();
    spd_pid_right.initialize(best_R.speedPID.P, best_R.speedPID.I, best_R.speedPID.D);
    // 设定值 (setpoint) 是您希望达到的目标速度
    float setpoint = 4.5;  // 假设的目标速度
    iter_time++;
    // 计算PID输出
    float pidOutput_L = spd_pid_left.update(setpoint, speedInput_L);   // 左轮PID调整
    float pidOutput_R = spd_pid_right.update(setpoint, speedInput_R);  // 右轮PID调整

    //  Serial.print("pidOutput_L: ");
    //  Serial.println(pidOutput_L);
    //  Serial.print("pidOutput_R: ");
    //  Serial.println(pidOutput_R);

    // 检查是否达到目标时间
    unsigned long currentMillis = millis();
    if (currentMillis >= targetCount || currentMillis >= targetCount) {
      motors.setMotorPower(0, 0);  // 停止电机
      if (!hasStopped) {
        readDataFromEEPROM();  // 仅在第一次停止时读取数据
        hasStopped = true;     // 设置标志，表明程序已停止
      }
    } else {
      // 根据PID输出调整电机功率
      int motorPower_L = constrain(25 + pidOutput_L, -55, 55);
      int motorPower_R = constrain(25 + pidOutput_R, -55, 55);

      // 应用PID调整到电机
      motors.setMotorPower(motorPower_L, motorPower_R);
      //    motors.setMotorPower(25,25);
    }


    // 记录数据到EEPROM
    currentMillis = millis();
    if (currentMillis - lastRecordTime >= logInterval) {  // 每500ms记录一次数据
      lastRecordTime = currentMillis;

      // 更新RobotData结构体并写入EEPROM
      RobotData data;
      data.speedLeft = readSpeed_L();
      data.speedRight = readSpeed_R();
      // data.encoderLeft = count_leftEncoder;
      // data.encoderRight = count_rightEncoder;
      data.encoderDifference = data.speedLeft - data.speedRight;
      data.pidP_L = best_L.speedPID.P;
      data.pidI_L = best_L.speedPID.I;
      data.pidD_L = best_L.speedPID.D;
      data.pidP_R = best_R.speedPID.P;
      data.pidI_R = best_R.speedPID.I;
      data.pidD_R = best_R.speedPID.D;
      data.Iter_time = iter_time;
      if (eepromAddress < EEPROM.length() - sizeof(RobotData)) {
        EEPROM.put(eepromAddress, data);
        eepromAddress += sizeof(RobotData);
      }
    }

    // 重置计数和总和以进行下一个100个样本点的计算
    sumOfSquaredErrors_L = 0;
    sumOfSquaredErrors_R = 0;
    sampleCount = 0;
  }
  //记录样本迭代次数
}


void applyMotorControl(float leftOutput, float rightOutput) {
  // 根据方向控制器的输出调整电机速度
  motors.setMotorPower(leftOutput, rightOutput);
}
//应用PID和储存
void Apply_EEPROM(float sumOfSquaredErrors_L, float sumOfSquaredErrors_R, float real_Mp_L, float real_Mp_R) {
}

float readSpeed_L() {
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime_L >= sampleInterval) {
    int currentEncoderCount = count_leftEncoder;
    int encoderDiff = currentEncoderCount - lastEncoderCount_L;
    unsigned long timeDiff = currentTime - lastSampleTime_L;

    // 仅在时间差大于0时计算速度，以防止除以0
    if (timeDiff > 0) {
      float speed = (encoderDiff / (float)timeDiff) * 10;  // 将速度转换为每秒脉冲数

      // 调试：检查计算出的速度
      //      Serial.print("Raw Speed L: "); Serial.println(speed);

      // 仅在检测到速度变化时应用低通滤波
      if (encoderDiff != 0) {
        ave_e0_spd = (ave_e0_spd * 0.7) + (speed * 0.3);
      }

      lastEncoderCount_L = currentEncoderCount;
      lastSampleTime_L = currentTime;
    }

    // 返回平滑后的速度
    return ave_e0_spd;
  }
  // 如果时间间隔未到，则返回上次的平均速度
  return ave_e0_spd;
}


float readSpeed_R() {
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime_R >= sampleInterval) {
    int currentEncoderCount = count_rightEncoder;
    int encoderDiff = currentEncoderCount - lastEncoderCount_R;
    unsigned long timeDiff = currentTime - lastSampleTime_R;

    // 仅在时间差大于0时计算速度，以防止除以0
    if (timeDiff > 0) {
      float speed = (encoderDiff / (float)timeDiff) * 10;  // 将速度转换为每秒脉冲数

      // 调试：检查计算出的速度
      //      Serial.print("Raw Speed R: "); Serial.println(speed);

      // 仅在检测到速度变化时应用低通滤波
      if (encoderDiff != 0) {
        ave_e1_spd = (ave_e1_spd * 0.7) + (speed * 0.3);
      }

      lastEncoderCount_R = currentEncoderCount;
      lastSampleTime_R = currentTime;
    }

    // 返回平滑后的速度
    return ave_e1_spd;
  }
  // 如果时间间隔未到，则不更新速度
  return ave_e1_spd;  // 注意：即使未更新速度，也应返回上次的平均速度
}



// 假设的目标速度
const float targetSpeed = 4.5;  // 例如，0.5米/秒

// 计算左右轮速度误差
float calculateSpeedError_L(float speedInput_L) {
  float leftWheelError = abs(speedInput_L - targetSpeed);

  return leftWheelError;
}

float calculateSpeedError_R(float speedInput_R) {

  float rightWheelError = abs(speedInput_R - targetSpeed);
  return rightWheelError;
}

// // 计算编码器偏离度
// float calculateEncoderDeviation(int encoderCount_L, int encoderCount_R) {
//   return abs(encoderCount_L - encoderCount_R);  // 返回左右编码器计数的绝对差值
// }
//
//计算Mp峰值超调
// 假设的目标峰值
const float Mp = 4.5;  // 左右轮之和为9
float over_Mp_L = 0;
float calculateMp_L(float speedInput_L) {

  over_Mp_L = speedInput_L - Mp;
  if (over_Mp_L > 0) {
    return over_Mp_L;  // 返回超调值
  } else {
    return 0;
  }
}
float over_Mp_R = 0;
float calculateMp_R(float speedInput_R) {

  over_Mp_R = speedInput_R - Mp;
  if (over_Mp_R > 0) {
    return over_Mp_R;  // 返回超调值
  } else {
    return 0;
  }
}
void readDataFromEEPROM() {
  RobotData data;
  for (int address = 0; address < eepromAddress; address += sizeof(RobotData)) {
    EEPROM.get(address, data);

    // 打印机器人运动相关的数据
    Serial.print("L Speed: ");
    Serial.print(data.speedLeft);
    Serial.print(", R Speed: ");
    Serial.print(data.speedRight);
    // Serial.print(", L Encoder: ");
    // Serial.print(data.encoderLeft);
    // Serial.print(", R Encoder: ");
    // Serial.print(data.encoderRight);
    Serial.print(", Target_Diff_avg: ");
    Serial.print(4.5 - (data.speedLeft + data.speedRight)/2);
    Serial.print(", Diff: ");
    Serial.print(data.speedLeft - data.speedRight);
    Serial.print("\n");
    // 打印PID参数
    Serial.print(", PID P_L: ");
    Serial.print(data.pidP_L);
    Serial.print(", PID I_L: ");
    Serial.print(data.pidI_L);
    Serial.print(", PID D_L: ");
    Serial.println(data.pidD_L);
    Serial.print("\n");
    // 打印PID参数
    Serial.print(", PID P_R: ");
    Serial.print(data.pidP_R);
    Serial.print(", PID I_R: ");
    Serial.print(data.pidI_R);
    Serial.print(", PID D_R: ");
    Serial.println(data.pidD_R);
    Serial.print(", iter_time: ");
    Serial.println(data.Iter_time);
    Serial.print("\n");
  }
}
