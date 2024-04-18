// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

# define EMIT_PIN    11    // Documentation says 11.
# define LS_LEFT_PIN A11   // Complete for DN1 pin
# define LS_MIDLEFT_PIN A0  // Complete for DN2 pin
# define LS_MIDDLE_PIN A2  // Complete for DN3 pin
# define LS_MIDRIGHT_PIN A3  // Complete for DN4 pin
# define LS_RIGHT_PIN A4 // Complete for DN5 pin
# define MAX_SAMPLES 10

float results[ MAX_SAMPLES ]; // An array of MAX_SAMPLES length
unsigned long result;
unsigned long count;
int number;
bool done ;
float end_time_ls[5];
// Store our pin numbers into an array, which means
// we can conveniently select one later.
// ls(line sensor)_pin


int ls_pins[5] = {LS_LEFT_PIN,
                  LS_MIDLEFT_PIN,
                  LS_MIDDLE_PIN,
                  LS_MIDRIGHT_PIN,
                  LS_RIGHT_PIN };
// Class to operate the linesensor(s).
class LineSensor_c {
  public:
  
    // Constructor, must exist.
    LineSensor_c() {

    } 
    initialize(){
      number = 0;
      done = false;
      //1)charge the  capacitor by setting pin
      pinMode( EMIT_PIN, OUTPUT ); // Set EMIT as an input (off)
      pinMode( ls_pins[number], INPUT );     // Set line sensor pin to input
      digitalWrite( EMIT_PIN, HIGH );
  // Start Serial, wait to connect, print a debug message.
      Serial.begin(9600);
      delay(1500);
      Serial.println("***RESET***");
    }
    //read line sensor
    //which sensor to read and return result
    

};



#endif
