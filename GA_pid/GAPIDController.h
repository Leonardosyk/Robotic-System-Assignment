#ifndef GAPIDController_h
#define GAPIDController_h
#include <Arduino.h>

struct PIDParameters {
  float P;
  float I;
  float D;
};

struct Individual {
  PIDParameters speedPID;  // 速度PID参数
  float fitness;  // 适应度
};

class GAPIDController {
  public:
    GAPIDController(int populationSize, float pMin, float pMax, float iMin, float iMax, float dMin, float dMax);
    void initializePopulation();
    float calculateFitness(float wheelSpeedError, float encoderDeviation, float responseTime);
    void selection();
    void crossover(Individual &parent1, Individual &parent2, Individual &child);
    void mutate(Individual &individual);
    void runGeneticAlgorithm(float wheelSpeedError,float encoderDeviation,float responseTime);
    float randomPIDValue(float minVal, float maxVal);
    float uniformCrossoverValue(float value1, float value2);
    Individual getBestIndividual();
    Individual randomIndividual();
    
  private:
    int _populationSize;
    float _pMin, _pMax, _iMin, _iMax, _dMin, _dMax;
    Individual *_population;
    // 生成随机PID值的函数
    PIDParameters randomPIDValues(float pMin, float pMax, float iMin, float iMax, float dMin, float dMax);
    
};

#endif
