#include "GAPIDController.h"
#include <stdlib.h>
#include <float.h>

float alpha1 = 2;
float alpha2 = 0.01;
float alpha3 = 0.1;
//extern float wheelSpeedError;
//extern float encoderDeviation;
//extern float responseTime;


GAPIDController::GAPIDController(int populationSize, float pMin, float pMax, float iMin, float iMax, float dMin, float dMax) {
  _populationSize = populationSize;
  _pMin = pMin;
  _pMax = pMax;
  _iMin = iMin;
  _iMax = iMax;
  _dMin = dMin;
  _dMax = dMax;
  _population = new Individual[_populationSize];
}

void GAPIDController::initializePopulation() {
  for (int i = 0; i < _populationSize; i++) {
    _population[i].speedPID = randomPIDValues(_pMin, _pMax, _iMin, _iMax, _dMin, _dMax);
  }
}

PIDParameters GAPIDController::randomPIDValues(float pMin, float pMax, float iMin, float iMax, float dMin, float dMax) {
  PIDParameters params;
  params.P = randomPIDValue(pMin, pMax);
  params.I = randomPIDValue(iMin, iMax);
  params.D = randomPIDValue(dMin, dMax);
  return params;
}
float GAPIDController::calculateFitness(float wheelSpeedError_L, float Mp, float responseTime) {
  float fitness = 0;
  // 速度误差越小越好
  fitness += alpha1 * (1 - abs(wheelSpeedError_L));
  // 编码器偏离度越小越好
  fitness += alpha2 * (1 - abs(Mp));
  // 响应时间越短越好
  fitness += alpha3 * (1 / (1 + responseTime));

  return fitness;
}


void GAPIDController::selection() {
  // 计算总适应度
  float totalFitness = 0.0;
  for (int i = 0; i < _populationSize; i++) {
    totalFitness += _population[i].fitness;
  }

  Individual *newPopulation = new Individual[_populationSize];

  // 进行轮盘赌选择
  for (int i = 0; i < _populationSize; i++) {
    float randValue = (float)random(1000) / 1000.0 * totalFitness;
    float cumulativeFitness = 0.0;

    for (int j = 0; j < _populationSize; j++) {
      cumulativeFitness += _population[j].fitness;
      if (cumulativeFitness >= randValue) {
        newPopulation[i] = _population[j];
        break;
      }
    }
  }

  // 用新种群替换旧种群
  delete[] _population;
  _population = newPopulation;
}



void GAPIDController::crossover(Individual &parent1, Individual &parent2, Individual &child) {
  // 对每个PID参数进行均匀交叉
  child.speedPID.P = uniformCrossoverValue(parent1.speedPID.P, parent2.speedPID.P);
  child.speedPID.I = uniformCrossoverValue(parent1.speedPID.I, parent2.speedPID.I);
  child.speedPID.D = uniformCrossoverValue(parent1.speedPID.D, parent2.speedPID.D);
}


void GAPIDController::mutate(Individual &individual) {
  float mutationRate = 0.07;  // 变异率

  // 为速度PID参数应用变异
  if (random(0, 100) < mutationRate * 100) {
    individual.speedPID.P += randomPIDValue(-0.08, 0.08);  // 随机调整P
    individual.speedPID.I += randomPIDValue(-0.08, 0.08);  // 随机调整I
    individual.speedPID.D += randomPIDValue(-0.08, 0.08);  // 随机调整D
  }
}


Individual GAPIDController::getBestIndividual() {
  Individual best = _population[0];
  float bestFitness = _population[0].fitness;

  for (int i = 1; i < _populationSize; i++) {
    if (_population[i].fitness > bestFitness) {
      best = _population[i];
      bestFitness = _population[i].fitness;
    }
  }

  return best;
}
//均匀交叉
float GAPIDController::uniformCrossoverValue(float value1, float value2) {
  if (random(0, 2) == 0) {  // 随机选择父代的某个参数值
    return value1;
  } else {
    return value2;
  }
}
float GAPIDController::randomPIDValue(float minVal, float maxVal) {
  // 生成[minVal, maxVal]范围内的随机数
  // 生成[minVal, maxVal]范围内的随机数
  float range = maxVal - minVal;
  float randomValue = (float)random(255) / 255.0;
  return minVal + randomValue * range;
}
void GAPIDController::runGeneticAlgorithm(float wheelSpeedError, float encoderDeviation, float responseTime) {
  // 计算当前PID参数的适应度
  for (int i = 0; i < _populationSize; i++) {
    _population[i].fitness = calculateFitness(wheelSpeedError, encoderDeviation, responseTime);
  }

  
  selection();
  // 确定精英个体数量
  const int eliteSize = _populationSize * 0.25;  // 例如，保留10%的精英个体

  // 找出最优个体
  Individual eliteIndividuals[eliteSize];
  for (int i = 0; i < eliteSize; ++i) {
    int maxIndex = 0;
    for (int j = 1; j < _populationSize; ++j) {
      if (_population[j].fitness > _population[maxIndex].fitness) {
        maxIndex = j;
      }
    }
    eliteIndividuals[i] = _population[maxIndex];
    _population[maxIndex].fitness = -FLT_MAX;  // 防止再次选择同一个个体
  }
  // 交叉和变异（简化）
    for (int i = eliteSize; i < _populationSize - 1; i++) {
        int parentIndex1 = random(0, _populationSize);
        int parentIndex2 = random(0, _populationSize);
        while (parentIndex2 == parentIndex1) {
            parentIndex2 = random(0, _populationSize);
        }

        Individual child;
        crossover(_population[parentIndex1], _population[parentIndex2], child);
        mutate(child);

        // 替换种群中的个体
        _population[i] = child;
    }

    // 将精英个体复制回新种群
    for (int i = 0; i < eliteSize; ++i) {
        _population[i] = eliteIndividuals[i];
    }
}
