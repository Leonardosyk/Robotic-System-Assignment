// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  

  public:
  float Kp;             // Proportional gain
  float Ki;         // Proportional term
  float Kd; // Feedback signal (output)
  float p_term;         // Proportional term
  float i_term;         // Integral term
  float d_term;         // Derivative term
  float i_sum ;
  float last_error; // Previous error to calculate derivative
  float feedback; // Feedback signal (output)
  float last_d_term;
  float alpha;
  unsigned long ms_last_ts;

  
  void initialize(float init_Kp,float init_Ki,float init_Kd){
        Kp = init_Kp;
        Ki = init_Ki;
        Kd = init_Kd;
        feedback = 0;
        last_error = 0;
        p_term = 0;
        i_term = 0;
        d_term = 0;
        i_sum = 0;
        ms_last_ts = millis();
        last_d_term = 0;
        alpha = 0.1; // 这只是一个示例值，您可能需要根据实际情况调整
        
  }
  
  void reset(){
    p_term = 0;
    i_term = 0;
    d_term = 0;
    i_sum = 0;
    feedback = 0;
    last_error = 0;
    ms_last_ts = millis();

  }
    // Constructor, must exist.
  float update( float measurement, float demand) {
        // Calculate the error
        
        unsigned long ms_now_ts;
        unsigned long ms_dt;
        float float_dt;
        float diff_error;
        float error;
    //grab time to calr elapsed time
        ms_now_ts = millis();

        ms_dt =  ms_now_ts - ms_last_ts;
        ms_last_ts = millis();
        float_dt = (float)ms_dt;
        if (float_dt == 0) return feedback;

        //calculate error
        error = measurement - demand;

        // Calculate the proportional term
        p_term = Kp * error;

        // Calculate the integral term
        i_sum = i_sum + (error * float_dt);

        i_term = Ki * i_sum;

        // Calculate the derivative term
        diff_error = (error - last_error) / float_dt;
        last_error = error;
        d_term = Kd * diff_error;
        d_term = alpha * d_term + (1 - alpha) * last_d_term;
        last_d_term = d_term;
        
        // Update the feedback signal
        feedback  = p_term + i_term + d_term;
        return feedback;
    }
  PID_c() {
        
    } 



};



#endif
