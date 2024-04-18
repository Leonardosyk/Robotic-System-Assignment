#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"

#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:
    //PID update variables.
    float last_error;
    float p_term;
    float i_term;
    float d_term;
    float i_sum;
    float feedback;

    // To store gains.
    float p_gain;
    float i_gain;
    float d_gain;
    // To determine time elapsed.
    unsigned long ms_last_ts;
    long prevCount_leftEncoder = 0;
    long prevCount_rightEncoder = 0;
    unsigned long prevTime = 0;
    float prev_measurement = 0; 
    
    // Constructor, must exist.
    PID_c() {

    }

    void initialise (float kp, float ki, float kd){
      feedback = 0;
      last_error = 0;
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;

      p_gain = kp;
      i_gain = ki;
      d_gain = kd;

      ms_last_ts = millis();
      }

    void reset(){
      p_term = 0;
      i_term = 0;
      d_term = 0;
      i_sum = 0;
      last_error = 0;
      feedback = 0;
      ms_last_ts = millis();
      }

    float update( float demand, float measurement){
      float error;
      unsigned long ms_now_ts;
      unsigned long ms_dt;
      float float_dt;
      float diff_error;
      long deltaCountsLeft;
      long deltaCountsRight;
      //Grab time to calc elaped time.
      ms_now_ts = millis();
      ms_dt = ms_now_ts - ms_last_ts;

      //ms_last_t has been used, so update
      //it for the next call of this update.
      ms_last_ts = millis();

      //typecassting the different of two
      //unsigned long is safer.
      float_dt = (float)ms_dt;

      if (float_dt == 0) return feedback;

      //Calculate error signal.
      error =  demand - measurement;

      //P term,nice and easy
      p_term = p_gain *error;

      //discrete integration
      i_sum = i_sum +(error * float_dt);

      //i_term
      i_term = i_gain *i_sum;

      //d_term
      diff_error = (error - last_error)/float_dt;
      last_error = error;
      d_term = diff_error*d_gain;


      feedback = p_term + i_term + d_term;

      return feedback;
      }

      
    float apply_lpf(float input) {
        float alpha = 0.3; 
        float filtered = alpha * input + (1.0 - alpha) * prev_measurement;
        prev_measurement = filtered;
        return filtered;
    }

};



#endif
