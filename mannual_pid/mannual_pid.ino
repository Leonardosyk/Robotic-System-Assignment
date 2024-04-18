struct RobotData {
  float speedLeft;
  float speedRight;
  long encoderLeft;
  long encoderRight;
  long encoderDifference;
};
#include "motors.h"
#include "encoders.h"
#include "timer3.h"
#include "pid.h"
#include <EEPROM.h>


bool hasStopped = false;  // 全局变量，用于追踪程序是否已经停止
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

PID_c spd_pid_left;   // 方向PID
PID_c spd_pid_right;  // 方向PID


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
// 设置EEPROM地址初始值
int eepromAddress = 0;

// 速度PID参数
float speedSetpoint = 0.5;  // 假设的速度设定值
float speedInput = 0.0;
float speedOutput = 0.0;
float lastSampleTime = 0.0;
// 方向PID参数
float directionSetpoint = 0.0;  // 直线行驶
float directionInput = 0.0;
float directionOutput = 0.0;

// 全局变量
unsigned long lastSampleTime_L = 0;  // 上次左轮速度采样的时间
int lastEncoderCount_L = 0;          // 上次左轮编码器计数
unsigned long lastSampleTime_R = 0;  // 上次右轮速度采样的时间
int lastEncoderCount_R = 0;          // 上次右轮编码器计数

// 定义按键A的引脚号
#define BUTTON_A_PIN 14

void setup() {
  //setup encoders
  setupEncoder0();
  setupEncoder1();

  // 设置按键A的引脚为输入，并启用内部上拉电阻
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  // 初始化PID控制器 GA
  spd_pid_left.initialize(8.95, 0.16, 6.12);   // 方向PID参数
  spd_pid_right.initialize(8.95, 0.16, 6.15);  // 方向PID参数
    // 初始化PID控制器
  // spd_pid_left.initialize(10, 0, 0);   // 方向PID参数
  // spd_pid_right.initialize(15, 0, 0);  // 方向PID参数


  Serial.begin(9600);
  delay(1000);
  Serial.println("***RESET***");
  update_ts = millis();
  //  motors.setMotorPower(25, 25);
  ave_e1_spd = 0;
  ave_e0_spd = 0;

  spd_pid_left.reset();
  spd_pid_right.reset();
  pid_tms = millis();
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

  // 打印诊断信息
  //  Serial.print("count_leftEncoder: ");
  //  Serial.println(count_leftEncoder);
  //  Serial.print("count_rightEncoder: ");
  //  Serial.println(count_rightEncoder);
  //  Serial.print("speedInput_L: ");
  //  Serial.println(speedInput_L);
  //  Serial.print("speedInput_R: ");
  //  Serial.println(speedInput_R);

  // 设定值 (setpoint) 是您希望达到的目标速度
  float setpoint = 4.5;  // 假设的目标速度

  // 计算PID输出
  float pidOutput_L = spd_pid_left.update(setpoint, speedInput_L);   // 左轮PID调整
  float pidOutput_R = spd_pid_right.update(setpoint, speedInput_R);  // 右轮PID调整
  // Serial.print("pidOutput_L: ");
  // Serial.println(pidOutput_L);
  // Serial.print("pidOutput_R: ");
  // Serial.println(pidOutput_R);
  //  // 应用PID调整到电机
  //  applyMotorControl(pidOutput_L, pidOutput_R);
  unsigned long currentMillis = millis();
  // 检查是否达到目标编码器计数
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

    RobotData data;
    data.speedLeft = readSpeed_L();
    data.speedRight = readSpeed_R();
    data.encoderLeft = count_leftEncoder;
    data.encoderRight = count_rightEncoder;
    data.encoderDifference = data.encoderRight - data.encoderLeft;

    if (eepromAddress < EEPROM.length() - sizeof(RobotData)) {
      EEPROM.put(eepromAddress, data);
      eepromAddress += sizeof(RobotData);
    }
  }
}


void applyMotorControl(float leftOutput, float rightOutput) {
  // 根据方向控制器的输出调整电机速度
  motors.setMotorPower(leftOutput, rightOutput);
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
float calculateSpeedError(float speedInput_L, float speedInput_R) {
  float leftWheelError = abs(speedInput_L - targetSpeed);
  float rightWheelError = abs(speedInput_R - targetSpeed);
  return (leftWheelError + rightWheelError) / 2.0;  // 返回平均误差
}

// 计算编码器偏离度
float calculateEncoderDeviation(int encoderCount_L, int encoderCount_R) {
  return abs(encoderCount_L - encoderCount_R);  // 返回左右编码器计数的绝对差值
}

void readDataFromEEPROM() {
  RobotData data;
  for (int address = 0; address < eepromAddress; address += sizeof(RobotData)) {
    EEPROM.get(address, data);
    // 打印数据
    Serial.print("L Speed: ");
    Serial.print(data.speedLeft);
    Serial.print(", R Speed: ");
    Serial.print(data.speedRight);
    Serial.print(", L Encoder: ");
    Serial.print(data.encoderLeft);
    Serial.print(", R Encoder: ");
    Serial.print(data.encoderRight);
    Serial.print(", Diff: ");
    Serial.println(data.encoderDifference);
  }
}
