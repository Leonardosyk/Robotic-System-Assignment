#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#define STATE_START 0
#define STATE_EXIT_SQUARE 1
#define STATE_DRIVE_FORWARDS 2
#define STATE_FOUND_LINE 3
#define STATE_ONLINE 4
#define STATE_END 5
#define STATE_BACK 6
#define STATE_TURN_LEFT 7
#define STATE_TURN_RIGHT 8
#define STATE_TURN_SLEFT 9
#define STATE_TURN_SRIGHT 10
#define STATE_AROUND 11
#define LED_PIN 13  
#define BUZZER_PIN 6 
int state;
int turn_count = 0;
const unsigned long MAX_TIME_OFFLINE = 500; // 3 seconds.
unsigned long timeWentOffline = 0;

Motors::Motors() {}
Motors motor;  // Create an object of class Motors named motor
Kinematics kinematics;
LineSensor_c linesensor;

const float SOME_TOLERANCE_VALUE = 0.01;
PID_c trackPID;
bool hasFinished = false;

float demand = 0; 
int baseSpeed = 21; 


void setup(){
  motor.setup();
  setupEncoder0();
  setupEncoder1();
  linesensor.initialise();
  trackPID.initialise(10,0.001,7);
  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");
  

  
}


  
void loop() {
    if (hasFinished) {
        return; // If the trolley has completed its task, it returns directly without executing the following code
    }
    updateState();
    linesensor.WeightMeasurement();
    linesensor.parallelLineSensorRead();
    linesensor.readAndSetSensors(); 
    kinematics.update();
    double positionX = kinematics.getX();
    double positionY = kinematics.getY();
    double theta = kinematics.getTheta();
//    for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.print("Sensor ");
//    Serial.print(i + 1);
//    Serial.print(": ");
//    Serial.print(sensors[i]);
//    Serial.print(" ");
//    }
    if (state == STATE_START){
//      initialisingBeeps();
      Serial.println("STATE_START");
      }
    else if (state == STATE_EXIT_SQUARE ){
        motor.moveForward(22, 23);
        Serial.println("STATE_EXIT_SQUARE");
        }
    else if(state == STATE_DRIVE_FORWARDS) {
        motor.moveForward(22, 23);
        Serial.println("STATE_DRIVE_FORWARDS");
    } else if(state == STATE_FOUND_LINE) {
        beep(200,50);
        motor.stopMotors();
        motor.turnSpecificAngleRight(20,-1);
        Serial.println("STATE_FOUND_LINE");
    } 
    else if(state == STATE_ONLINE){  
        linesensor.readAndSetSensors();
        motorDecision();      
        Serial.println("STATE_ONLINE");
        
        
   }
      
      else if (state == STATE_END){
        Serial.println("STATE_END");
        }
      else if (state == STATE_BACK){
        Serial.println("STATE_BACK");
        }
      else {
        Serial.print("System Error, Unknown state: ");
        Serial.println(state);
        motor.stopMotors();
    } 
  }
    






static int consecutiveTrueCount = 0;  // Used to track the number of consecutively true
static int leftTurnCount = 0;        // Used to track the number of consecutive true left turns
static int rightTurnCount = 0;       // Used to track the number of consecutive right turns that are true
static bool previousState = false;   // Used to track the status of the last

void motorDecision() {
  float weight = linesensor.WeightMeasurement();
  float measurement = trackPID.apply_lpf(weight);
  float correction = trackPID.update(demand,measurement);
    int turnSpeed = 19;
    if ((S1)||(S3 && S2)||(S3 && S2 && S1)||(S3 && S1)||(S2 && !S3)||(S3 && S1 && S5)||(S1 && S2 && S3 && S4 && S5)||(S1 && S5)||(S3 && S2 && S4)||(S3 && S2 && S4&& S5)||(S2 && S5)) {  // 岔路口，优先左转
            motor.stopMotors();
            motor.turnLeft(23);
            consecutiveTrueCount = 0;

    } 

    else if (linesensor.Online3()&& S2) {
//      else if ((S2)||(S3)||(S4)||(S2&&S3&&S4)){
//        else if (linesensor.Online()) {
        motor.moveForward(baseSpeed - correction, baseSpeed + correction);
        consecutiveTrueCount = 0;  // Reset the count
    } 
//    else if ((S4 && !S3)||(S3 && S4)) {
//        motor.turnRight(18);
//        consecutiveTrueCount = 0;  // Reset the count
//    } 
    else if (S1 && !(S2 || S3)) {
        motor.turnLeft(27);  
        consecutiveTrueCount = 0;  // Reset the count
    } 
    else if (S5 && !(S4 || S3)) {
        motor.turnRight(20);  
        consecutiveTrueCount = 0;  // Reset the count
    } 
    else if (!S1 && !S2 && !S3 && !S4 && !S5) {
        if (abs(positionX)< 2000){
        if (previousState) { 
            consecutiveTrueCount++;
        } else {
            consecutiveTrueCount = 1; 
        }

        if (consecutiveTrueCount >= 20) {
          motor.turnSpecificAngleRight(23,-2.2);
          beep(200,500);
          if (abs(positionX) > 900 || abs(positionY) > 900) {
    state = STATE_BACK;
}

//          if (abs(positionX)>10){
//            turn_count += 1;}
          
          
//            motor.turnAroundInPlace(20);
        }
       }
       else{
        motor.stopMotors();
        }
    } 
    previousState = (!S1 && !S2 && !S3 && !S4 && !S5);
} 


//void motorDecision() {
//  float weight = linesensor.WeightMeasurement();
//  float measurement = trackPID.apply_lpf(weight);
//  float correction = trackPID.update(demand,measurement);
//    int turnSpeed = 19;
//    
//
//    if (linesensor.Online3()) {
//        motor.moveForward(baseSpeed - correction, baseSpeed + correction+1);
//    } 
////    else if (S3 && S1) {  
//////            motor.stopMotors();
////            while (!S3){
//////            motor.turnSpecificAngleRight(23,1.2);
////            
////            motor.turnAroundInPlaceLeft(20);
////            linesensor.parallelLineSensorRead();
////            linesensor.readAndSetSensors();
////            beep(200,200);
////            }
////
////    } 
////
////    else if (S5 && !S2 && !S3) {
//////      motor.stopMotors();
////            while (!S3){
//////            motor.turnSpecificAngleRight(23,-1.2);
////            
////            motor.turnAroundInPlaceRight(20);
////            linesensor.parallelLineSensorRead();
////            linesensor.readAndSetSensors();
////            }
////
////    } 
////    else if (!S3 && S4 && S2) {
////        while (!S3){
//////            motor.turnSpecificAngleRight(23,1.2);
////            
////            motor.turnAroundInPlaceLeft(20);
////            linesensor.parallelLineSensorRead();
////            linesensor.readAndSetSensors();
////            }
////    } 
////    else if (!S1 && !S2 && !S3 && !S4 && !S5) {
////          motor.turnSpecificAngleRight(23,-2.7);
////          beep(200,500);
////          if (abs(positionX) > 900 || abs(positionY) > 900) {
////    state = STATE_BACK;
////}
////
////       }
//       else{
//        motor.stopMotors();
//        }
//    } 
//













unsigned long timeStartedExiting = 0; 
const unsigned long MAX_TIME_EXITING_SQUARE = 1500;  // Set a maximum time (e.g. 3 seconds) for driving out of the square

void updateState() {
    switch(state) {
        case STATE_START:
            state = STATE_EXIT_SQUARE;  // Directly after start-up into the exit from the square
            timeStartedExiting = millis();  // Record the time of commencement of departure
            break;

        case STATE_EXIT_SQUARE:
            if (millis() - timeStartedExiting > MAX_TIME_EXITING_SQUARE) {
                state = STATE_DRIVE_FORWARDS;
            }
            break;

        case STATE_DRIVE_FORWARDS:
            if (S1&&S5) {
                Serial.println("Detected line after exiting square. Moving to STATE_FOUND_LINE.");
                state = STATE_FOUND_LINE;
            }
            break;
        
        case STATE_FOUND_LINE:
            if (linesensor.Online()) {
              
                Serial.println("Detected line. Moving to STATE_ONLINE.");
                state = STATE_ONLINE;
            }
            break;


        case STATE_ONLINE:
            if (!linesensor.Online()) {
                if(timeWentOffline == 0) {
                    timeWentOffline = millis(); // Record the first time you go offline
                } else if ((millis() - timeWentOffline > MAX_TIME_OFFLINE* 5)) {
                    state = STATE_END;
                }
//                else if ((abs(positionX)||abs(positionY)>900)&& previousState){
//                  state = STATE_BACK;}
                
            } 
//            else if ((abs(positionX)||abs(positionY)>900)&& previousState){
//            beep(200,500);
//            state = STATE_BACK;
//            }
            else {
                timeWentOffline = 0; // Back online, reset timer
            }
            break;
          
            
        case STATE_END:
        if (linesensor.Online()) {
                state = STATE_ONLINE; // If the line is detected again, return to the online state
                timeWentOffline = 0;
                
            }
        else  { // No line detected for longer than that
                state = STATE_BACK;
                motor.stopMotors();
             }
            break;
            
         case STATE_BACK:
              beep(200,500);
              Go_back();
              
              break; 
        default:
            Serial.print("System Error, Unknown state: ");
            Serial.println(state);
            motor.stopMotors();
            break;
    }
}



void Go_back() {
    kinematics.update();
    double positionX = kinematics.getX();
    double positionY = kinematics.getY();
    double theta = kinematics.getTheta();

    // Calculate the target angle and rotate to that direction
    float TargetAngle = atan2(-positionY, -positionX);
    float angleDifference = TargetAngle - theta;
    
    // Adjust the angle difference to within [-π, π].
    while (angleDifference > 3.14159265358979323846) {
        angleDifference -= 2.0 * 3.14159265358979323846;
    }
    while (angleDifference <= -3.14159265358979323846) {
        angleDifference += 2.0 * 3.14159265358979323846;
    }

    // Selection of the direction of rotation according to the angle difference
    if (angleDifference > 0) {
        motor.turnSpecificAngleRight(20, angleDifference -0.05); // turn counterclockwise
    } else {
        motor.turnSpecificAngleRight(20, angleDifference -0.1); // turn clockwise
    }
//    Serial.print("angleDifference: ");
//    Serial.print(angleDifference);  
//    Serial.print("X: ");
//    Serial.print(positionX);
//    Serial.print(" Y: ");
//    Serial.println(positionY);
//    delay(10000000000000);
    // After the rotation, the cart goes straight until it returns to the origin.
    while ((sqrt(positionX*positionX ) >7)) { 
        motor.moveForward(25,26);  
        kinematics.update();
        delay(100);
        positionX = kinematics.getX();
        positionY = kinematics.getY();
        theta = kinematics.getTheta();
        
//    Serial.print("angleDifference: ");
//    Serial.print(angleDifference);  
//    Serial.print("X: ");
//    Serial.print(positionX);
//    Serial.print(" Y: ");
//    Serial.println(positionY);
    }

    motor.stopMotors(); 
    hasFinished = true;
}




  
void beep(int duration, int toneFrequency) {
    analogWrite(BUZZER_PIN, toneFrequency);  
    delay(duration);
    analogWrite(BUZZER_PIN, 0);  
}


void initialisingBeeps() {
    
    for(int i=0; i<3; i++) { 
        beep(500,500);  // beep for 100ms
        delay(100);  // wait for 100ms
    }
}
