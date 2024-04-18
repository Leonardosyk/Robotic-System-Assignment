// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H
#define EMIT_PIN    11    
#define LS_LEFT_PIN 12    
#define LS_MIDLEFT_PIN 18 
#define LS_MIDDLE_PIN 20  
#define LS_MIDRIGHT_PIN 21
#define LS_RIGHT_PIN 22   
#define NUM_SENSORS 5
#define THRESHOLD 1100
float Weight;
int BiasPWM = 15;
int MaxTurnPWM = 5;
bool S1, S2, S3, S4, S5; // NUM_SENSORS is a bool variable.
bool sensors[NUM_SENSORS];
int ls_pin[NUM_SENSORS] = {LS_LEFT_PIN ,LS_MIDLEFT_PIN ,LS_MIDDLE_PIN ,LS_MIDRIGHT_PIN ,LS_RIGHT_PIN }; // An array of MAX_SAMPLES length
float ls_reading[NUM_SENSORS];

// Class to operate the linesensor(s).
class LineSensor_c {
  public:

//  void initialise();
//  bool Online();
//  void parallelLineSensorRead();
//  float WeightMeasurement();
//  void readAndSetSensors();


void initialise(){
  for (int i = 0; i < NUM_SENSORS;i ++){
  pinMode( ls_pin[i], INPUT );
  }
  
  pinMode( EMIT_PIN, OUTPUT );   
  digitalWrite( EMIT_PIN , HIGH );
  }

  
bool Online() {
  for (int i = 0; i < NUM_SENSORS; i++)  {
    if (ls_reading[i] > THRESHOLD) {
      return true;  
    }
  }
  return false;  
}

bool Online3() {
  for (int i = 1; i < 4; i++)  {
    if (ls_reading[i] > THRESHOLD) {
      return true;  
    }
  }
  return false;  
}
void parallelLineSensorRead(){
  int which;
  int count;

  for (which = 0; which < NUM_SENSORS; which++){
    pinMode(ls_pin[which],OUTPUT);
    digitalWrite(ls_pin[which],HIGH);
  }
  delay(10);
  for (which = 0; which < NUM_SENSORS; which++){
    pinMode(ls_pin[which],INPUT);
    
  } 
  
  unsigned long start_time;
  start_time = micros();

  unsigned long end_time_ls[NUM_SENSORS];
  
  for (which = 0; which < NUM_SENSORS; which++){
    end_time_ls[which] = 0;
  }
    
  bool done = false;
  count = NUM_SENSORS;
  unsigned long TIMEOUT = 50000;  // set to an appropriate value
  unsigned long current_time = micros();
  
  while (done == false && (micros() - current_time) < TIMEOUT){
    
    for (which = 0; which < NUM_SENSORS; which++){
      if (end_time_ls[which] == 0 && digitalRead(ls_pin[which]) == LOW){
        end_time_ls[which] = micros();
        count = count - 1;
        
        // Add a delay after reading each sensor
//        delayMicroseconds(8);
      }
      if (count == 0){
        done = true;
      }
    }

    for (which = 0; which < NUM_SENSORS; which++){
      unsigned long elapsed_time;
      elapsed_time = end_time_ls[which] - start_time;
      ls_reading[which] = (float)elapsed_time;
    }
  }
}


float WeightMeasurement() {

    parallelLineSensorRead();
    float Sum = (float)(ls_reading[1] + ls_reading[3]);


    if (Sum == 0.0) {
        return 0;
    }

    float N1 = (float)ls_reading[0] / Sum;
    float N3 = (float)ls_reading[2] / Sum;

    // Double weighting
    N1 *= 2.0;
    N3 *= 2.0;

    float Weight = N1 - N3;
    return Weight;
}

void readAndSetSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors[i] = ls_reading[i] > THRESHOLD; // 如果读数大于阈值，则为true，否则为false
  }

  
  S1 = sensors[0];
  S2 = sensors[1];
  S3 = sensors[2];
  S4 = sensors[3];
  S5 = sensors[4];
}
};
#endif
