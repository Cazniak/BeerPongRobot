
// This section just checks the board is the correct type for the timer library
#if ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && defined(ARDUINO_ARCH_MBED)
  #define USING_MBED_RPI_PICO_TIMER_INTERRUPT        true
#else
  #error This code is intended to run on the MBED RASPBERRY_PI_PICO platform! Please check your Tools->Board setting.
#endif

// This define must be placed at the beginning for the timer interrupt library
#define _TIMERINTERRUPT_LOGLEVEL_ 4   // _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4

#include "MBED_RPi_Pico_TimerInterrupt.h"   // Ducumentation (GitHub): https://github.com/khoih-prog/MBED_RPI_PICO_TimerInterrupt
#include <Arduino.h>
#include <QuickPID.h>   // Documentation (GitHub): https://github.com/Dlloydev/QuickPID
#include <math.h>
//#include <Servo.h>      // For controlling the servos


/*------------ GLOBALS AND DEFINITIONS ------------*/
// Define the motors
#define NMOTORS 3   //Number of motor(s) attached to this Arduino
#define M0 0
#define M1 1
#define M2 2


// Pins
const int enca[] = {21, 19, 17, 28, 26};
const int encb[] = {20, 18, 16, 27, 22};
const int MOTOR_PWM[] = {10, 12, 14, 6, 8};
const int MOTOR_DIR[] = {7, 9};         // Direction pins for Azimuth and Elevation drivers respectively (Flywheel motor driver direction pins just connected to MCU 3.3V as only one direction needed)
const int MOTOR_ENABLE = 15;            // Rest pin for the motors (Motor drivers disabled when LOW meaning the motors will just coast to a rest)
const int LAUNCH_FIRE = 4;              // Servo PWM pin to feed ball into flywheels
const int testPin = 5;                  // Monitor this pin with an oscilloscope for control loop frequency testing, etc... Toggle it in code each loop then monitor

// Variables
const float Fs = 150;                   // Sample frequency in Hertz - REMEMBER TO UPDATE THE FILTER WHEN CHANGING FS!!!!!
const float Ts = 1/Fs;                  // Sample period in seconds
const float Ts_us = Ts * 1e6;           // Sample period in microseconds
const float Fc = 150;                   // Control frequency in Hertz
const float Tc = 1/Fc;                  // Control period in seconds
const float Tc_us = Tc * 1e6;           // Control period in microseconds

const float Kp[] = {200, 200, 200, 18000, 20000};   // Proportional coefficients for each motor's controller (900)
const float Ki[] = {500, 500, 500, 555556, 100};   // Integral coefficients for each motor's controller (7563)
const float Kd[] = {0, 0, 0, 145.8, 193.2};             // Derivative coefficients for each motor's controller
float targetRPM[] = {0, 0, 0};                  // Target velocity for each motor in RPM
float targetRamp[] = {0, 0, 0};                 // Target velocity for each motor in RPM in the form of a ramp function
float riseTime = 0.5;                           // Target ramp function rise time in seconds
int step[] = {1, 1, 1};                         // Counter for the number of steps taken for each motor's ramp
int launchStepper = 0;                          // Counter for the number of control cycles before activating launch
int commandType = 0;                            // The type of command being received through Serial

          // COMMANDTYPE OPTIONS AND FORMATS
          // 0 = Do nothing
          //
          // 1 = Calibration command = Sets current azimuth and elevation position to home (zeros their pulseCount)
          // Format = <1> 
          //
          // 2 = Manual move command = Manually move azzimuth, elevation, or the motor speeds
          // Format = <2, (flywheel speed 1 in RPM), (flywheel speed 2 in RPM), (flywheel speed 3 in RPM), (+/- azimuth angle in deg), (elevation angle in deg)>
          //
          // 3 = Shoot command = Contains all angles and speeds, and carries out a shot 
          // Format = <3, (flywheel speed 1 in RPM), (flywheel speed 2 in RPM), (flywheel speed 3 in RPM), (+/- azimuth angle in deg), (elevation angle in deg)>
          //
          // 4 = Return to home command = Returns azimuth and elevation to zero deg to avoid having to calibrate next time machine is turned on
          // Format = <4>
          // 
          // 5 = Push Ball command = Activates the Launch servo to push a ball into the flywheels, then retracts.
          // Format = <5>

float argument[] = {0, 0, 0, 0, 0};           // Arguments for when a commend is received via Serial. 

float controlSignal0 = 0;                     // The output from the PID controller for motor 0 - a PWM signal with duty cycle between 0 and 65535 - the control signal
float controlSignal1 = 0;                     // The output from the PID controller for motor 1 - a PWM signal with duty cycle between 0 and 65535 - the control signal
float controlSignal2 = 0;                     // The output from the PID controller for motor 2 - a PWM signal with duty cycle between 0 and 65535 - the control signal
float controlSignal3 = 0;                     // The output from the PID controller for motor 3 (Azimuth) - a PWM signal with duty cycle between 0 and 65535 - the control signal
float controlSignal4 = 0;                     // The output from the PID controller for motor 4 (Elevation) - a PWM signal with duty cycle between 0 and 65535 - the control signal
float signalAzi = 0;                             // A placeholder variable for taking the absolute value of the Azimuth PID control signal
float signalEle = 0;                             // A placeholder variable for taking the absolute value of the Elevation PID control signal
const float VEL_RATIO = (60 / 59.94) * Fs;    // Ratio for converting velocity from pulses per sampling period to RPM
volatile int currVel[] = {0, 0, 0};           // Current velocity of each motor in pulses per sampling period

volatile int pulseCount[] = {0, 0, 0, 0, 0};    // Variable for storing encoder pulse counts between each sample for each motor
float currVelRPM[] = {0, 0, 0};         // Current velocity of each motor in RPM
float filteredCurrVelRPM[] = {0, 0, 0}; // LP Filtered current velocity for each motor
unsigned int loopTimer1 = 0;            // Variable to keep track of when the control loop should run
unsigned int loopTimer2 = 0;            // Variable to keep track of when the servo rest loop should run
bool REST = LOW;                        // Flag so that program only enters the servo rest loop once when not in use
bool direction[] = {HIGH, HIGH};        // Direction for Azimuth and Elevation motor control signals respectively (flywheels only have one direction, so not stored as variable)
const float ANGLE2PWM = (36000/180);    // For use in equations converting angles to PWM (2^16) (PWM = (ANGLE2PWM * <desired angle>) + 16000)
//int servoPos[] = {90, 90, 0};         // Array to store the current positions of the servos when moving [0] = Azimuth,, [1] = Elevation, [2] = Fire (Alphabetical)


float targetElevation = 0;                      // Target elevation angle in degrees (defaults to flat)
float targetAzimuth = 0;                        // Target azimuth angle in degrees (defaults to straight ahead)
float targetAzimuthPulses = 0;
float targetElevationPulses = 0;
volatile int currAzimuth = 0;
volatile int currElevation = 0;
float currAzimuthPulses = 0;                            // Current number of encoder pulses from 0deg (looking straight ahead)
float currElevationPulses = 0;                          // Current number of encoder pulses from 0deg (flat elevation)
const float AZIMUTH_RATIO = 9.93333333/2;    // Ratio for converting azimuth from degrees to encoder pulses (pulses/degree)
const float AZIMUTH_RATIO_INV = (1/AZIMUTH_RATIO);
const float ELEVATION_RATIO = 17.12643678/2;   // Ratio for converting elevation from degrees to encoder pulses (pulses/degree)
const float ELEVATION_RATIO_INV = (1/ELEVATION_RATIO);

// Serial Receive Variables
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// Variables to hold the parsed data
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;

boolean newData = false;

// Filter Coefficients - The matrices returned from "[B,A] = butter(N, Wn, "low") where N = order and Wn = (fc)/(fs/2)

// fs = 150Hz, fc = 2Hz, N = 2
const float B[] = {0.0402, 0.0402};
const float A[] = {1, -0.9195}; 

// Filter variables
float y = 0;                   // Variable to store the current filtered output
float xprev[] = {0, 0, 0};     // Array variables to store the previous inputs
float xprev2[] = {0, 0, 0};    // Array variables to store the previous previous inputs
float yprev[] = {0, 0, 0};     // Array variables to store the previous outputs
float yprev2[] = {0, 0, 0};    // Array variables to store the previous previous outputs


// Unused filters during development
/*// fs = 150Hz, fc = 5Hz, N = 1
const float B[] = {0.0951, 0.0951};
const float A[] = {1, -0.8098}; 
*/

/*// fs = 150Hz, fc = 10Hz, N = 1
const float B[] = {0.1753, 0.1753};
const float A[] = {1, -0.6494};
*/

/*// fs = 150Hz, fc = 2Hz, N = 2
const float B[] = {0.0017, 0.0033, 0.0017};
const float A[] = {1, -1.8817, 0.8883}; 
*/

/*// fs = 150Hz, fc = 5Hz, N = 2
const float B[] = {0.0095, 0.0191, 0.0095};
const float A[] = {1, -1.7056, 0.7437}; 
*/

/*// fs = 150Hz, fc = 10Hz, N = 2
const float B[] = {0.0336, 0.0671, 0.0336};
const float A[] = {1, -1.4190, 0.5533}; 
*/

/*// fs = 100Hz, fc = 10Hz, N = 1
const float B[] = {0.2452, 0.2452};
const float A[] = {1, -0.5095};
*/

/*// fs = 75Hz, fc = 10Hz, N = 1
const float B[] = {0.3081, 0.3081};
const float A[] = {1, -0.3839};
*/

/*// fs = 75Hz, fc = 5Hz, N = 1
const float B[] = {0.1753, 0.1753};
const float A[] = {1, -0.6494};           // Interestingly has the same coefficients as fs = 150Hz, fc = 10Hz
*/

/*// fs = 100Hz, fc = 5Hz, N = 1
const float B[] = {0.1367, 0.1367};
const float A[] = {1, -0.7265};  
*/

/*// fs = 100Hz, fc = 2Hz, N = 1
const float B[] = {0.0592, 0.0592};
const float A[] = {1, -0.8816}; 
*/

/*// fs = 75Hz, fc = 2Hz, N = 1
const float B[] = {0.0775, 0.0775};
const float A[] = {1, -0.8451}; 
*/

/*// fs = 75Hz, fc = 2Hz, N = 2
const float B[] = {0.0063, 0.0125, 0.0063};
const float A[] = {1, -1.7640, 0.7890}; 
*/

/*// fs = 100Hz, fc = 2Hz, N = 2
const float B[] = {0.0036, 0.0072, 0.0036};
const float A[] = {1, -1.8227, 0.8372}; 
*/

/*// fs = 100Hz, fc = 5Hz, N = 2
const float B[] = {0.0201, 0.0402, 0.0201};
const float A[] = {1, -1.5610, 0.6414}; 
*/

/*// fs = 75Hz, fc = 5Hz, N = 2
const float B[] = {0.0336, 0.0671, 0.0336};
const float A[] = {1, -1.4190, 0.5533}; 
*/

/*// fs = 75Hz, fc = 10Hz, N = 2
const float B[] = {0.1084, 0.2169, 0.1084};
const float A[] = {1, -0.8773, 0.3111}; 
*/

/*// fs = 100Hz, fc = 10Hz, N = 2
const float B[] = {0.0675, 0.1349, 0.0675};
const float A[] = {1, -1.1430, 0.4128}; 
*/


// Generate the QuickPID classes - one PID controller for each motor
QuickPID PID0(&filteredCurrVelRPM[0], &controlSignal0, &targetRamp[0], Kp[0], Ki[0], Kd[0], PID0.pMode::pOnError, PID0.dMode::dOnMeas, PID0.iAwMode::iAwClamp, PID0.Action::direct);
QuickPID PID1(&filteredCurrVelRPM[1], &controlSignal1, &targetRamp[1], Kp[1], Ki[1], Kd[1], PID1.pMode::pOnError, PID1.dMode::dOnMeas, PID1.iAwMode::iAwClamp, PID1.Action::direct);
QuickPID PID2(&filteredCurrVelRPM[2], &controlSignal2, &targetRamp[2], Kp[2], Ki[2], Kd[2], PID2.pMode::pOnError, PID2.dMode::dOnMeas, PID2.iAwMode::iAwClamp, PID2.Action::direct);

// New PID Controllers for Position Control
QuickPID PID3(&currAzimuthPulses, &controlSignal3, &targetAzimuthPulses, Kp[3], Ki[3], Kd[3], PID3.pMode::pOnError, PID3.dMode::dOnMeas, PID3.iAwMode::iAwClamp, PID3.Action::direct);
QuickPID PID4(&currElevationPulses, &controlSignal4, &targetElevationPulses, Kp[4], Ki[4], Kd[4], PID4.pMode::pOnError, PID4.dMode::dOnMeas, PID4.iAwMode::iAwClamp, PID4.Action::direct);

// QuickPID Template
// QuickPID::QuickPID(float* Input, float* Output, float* Setpoint, float Kp, float Ki, float Kd, pMode pMode = pMode::pOnError, dMode dMode = dMode::dOnMeas, iAwMode iAwMode = iAwMode::iAwCondition, Action action = Action::direct)


// Initialise timers
MBED_RPI_PICO_Timer Timer0(0);
MBED_RPI_PICO_Timer Timer1(1);
MBED_RPI_PICO_Timer Timer2(2);

/*
// Create the Servo objects to control
Servo SERVO_ELEVATION;
Servo SERVO_AZIMUTH;
Servo SERVO_FIRE;
*/

/*------------ Functions ------------*/

void calcVelocity0(uint alarm_num) {                      // Called by the three timers every Ts
  TIMER_ISR_START(alarm_num);                             // Always call this for MBED RP2040 before processing ISR
  //digitalWrite(testPin, !digitalRead(testPin));           // testPin toggle for monitoring frequency of interrupt
  currVel[0] = pulseCount[0];                             // Saves pulses counted in current sampling period to currVel array
  pulseCount[0] = 0;                                      // Reset pulse count for next sampling period
  TIMER_ISR_END(alarm_num);                               // Always call this for MBED RP2040 after processing ISR
}

void calcVelocity1(uint alarm_num) {                      // Called by the three timers every Ts
  TIMER_ISR_START(alarm_num);                             // Always call this for MBED RP2040 before processing ISR
  //digitalWrite(testPin, !digitalRead(testPin));         // testPin toggle for monitoring frequency of interrupt
  currVel[1] = pulseCount[1];                             // Saves pulses counted in current sampling period to currVel array
  pulseCount[1] = 0;                                      // Reset pulse count for next sampling period
  TIMER_ISR_END(alarm_num);                               // Always call this for MBED RP2040 after processing ISR
}

void calcVelocity2(uint alarm_num) {                      // Called by the three timers every Ts
  TIMER_ISR_START(alarm_num);                             // Always call this for MBED RP2040 before processing ISR
  //digitalWrite(testPin, !digitalRead(testPin));         // testPin toggle for monitoring frequency of interrupt
  currVel[2] = pulseCount[2];                             // Saves pulses counted in current sampling period to currVel array
  pulseCount[2] = 0;                                      // Reset pulse count for next sampling period
  TIMER_ISR_END(alarm_num);                               // Always call this for MBED RP2040 after processing ISR
}

// Flywheel Encoder Interrupt Functions
template <int j>
void readEncoder() {              // Called every encoder pulse
  pulseCount[j]++;                // Count every encoder pulse as it comes in
}


// Azimuth Encoder Interrupt Functions
void readAzimuthEncoder1() {              // Called every time Azimuth encoder A has a rising edge
  int b = digitalRead(encb[3]);
  if (b > 0) {
    pulseCount[3]--;                // Count every encoder pulse as it comes in
  }
  else {
    pulseCount[3]++;                // Count every encoder pulse as it comes in
  }
}

void readAzimuthEncoder2() {              // Called every time Azimuth encoder B has a rising edge
  int a = digitalRead(enca[3]);
  if (a > 0) {
    pulseCount[3]++;                // Count every encoder pulse as it comes in
  }
  else {
    pulseCount[3]--;                // Count every encoder pulse as it comes in
  }
}

void readAzimuthEncoder3() {              // Called every time Azimuth encoder A has a falling edge
  int b = digitalRead(encb[3]);
  if (b > 0) {
    pulseCount[3]++;                // Count every encoder pulse as it comes in
  }
  else {
    pulseCount[3]--;                // Count every encoder pulse as it comes in
  }
}

void readAzimuthEncoder4() {              // Called every time Azimuth encoder B has a falling edge
  int a = digitalRead(enca[3]);
  if (a > 0) {
    pulseCount[3]--;                // Count every encoder pulse as it comes in
  }
  else {
    pulseCount[3]++;                // Count every encoder pulse as it comes in
  }
}


// Elevation Encoder Interrupt Functions
void readElevationEncoder1() {              // Called every time Elevation encoder A has a rising edge
  int b = digitalRead(encb[4]);
  if (b > 0) {
    pulseCount[4]++;                // Count every encoder pulse as it comes in
  }
  else {
    pulseCount[4]--;                // Count every encoder pulse as it comes in
  }
}

void readElevationEncoder2() {              // Called every time Elevationuth encoder B has a rising edge
  int a = digitalRead(enca[4]);
  if (a > 0) {
    pulseCount[4]--;                // Count every encoder pulse as it comes in
  }
  else {
    pulseCount[4]++;                // Count every encoder pulse as it comes in
  }
}

void readElevationEncoder3() {              // Called every time Elevation encoder A has a falling edge
  int b = digitalRead(encb[4]);
  if (b > 0) {
    pulseCount[4]--;                // Count every encoder pulse as it comes in
  }
  else {
    pulseCount[4]++;                // Count every encoder pulse as it comes in
  }
}

void readElevationEncoder4() {              // Called every time Elevation encoder B has a falling edge
  int a = digitalRead(enca[4]);
  if (a > 0) {
    pulseCount[4]++;                // Count every encoder pulse as it comes in
  }
  else {
    pulseCount[4]--;                // Count every encoder pulse as it comes in
  }
}

// Low Pass Butterworth Filter - Order = 1    (Not used)
/* // Low Pass Butterworth Filter - 1st Order

float LPFilter(float x, int motor) {

  // x = input (current velocity reading), y = output (filtered velocity)

    y = B[0] * x + B[1] * xprev[motor] + (-1 * A[1]) * yprev[motor];
    yprev2[motor] = yprev[motor];
    yprev[motor] = y;
    xprev2[motor] = xprev[motor];
    xprev[motor] = x;

    return(y);
}
*/

// Low Pass Butterworth Filter - Order = 2
float LPFilter(float x, int motor) {

  // x = input (current velocity reading), y = output (filtered velocity)

    y = B[0] * x + B[1] * xprev[motor] + B[2] * xprev2[motor] + (-1 * A[1]) * yprev[motor] + (-1 * A[2]) * yprev2[motor];
    yprev2[motor] = yprev[motor];
    yprev[motor] = y;
    xprev2[motor] = xprev[motor];
    xprev[motor] = x;

    return(y);
}


float Ramp(float Target, int MOTOR) {
    if (step[MOTOR] >= (riseTime/Tc)) {
        return(Target);
    }
    else {
        Target = ((step[MOTOR] * Tc * Target) / (riseTime));
        step[MOTOR] = step[MOTOR] + 1;
        return(Target);
    } 
}


void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}


/*  Old parseData function
void parseData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ","); // Obtain the type of command being received
  commandType = atoi(strtokIndx); // Convert it to an integer

  strtokIndx = strtok(NULL, ","); // Obtain the first target speed in RPM
  targetRPM[0] = atof(strtokIndx); // Convert it to a float

  strtokIndx = strtok(NULL, ","); // Obtain the second target speed in RPM
  targetRPM[1] = atof(strtokIndx); // Convert it to a float

  strtokIndx = strtok(NULL, ","); // Obtain the third target speed in RPM
  targetRPM[2] = atof(strtokIndx); // Convert it to a float

  strtokIndx = strtok(NULL, ","); // Obtain the elevation target in degrees (0 - 180)
  targetElevation = atoi(strtokIndx); // Convert it to an integer

  strtokIndx = strtok(NULL, ","); // Obtain the target azimuth in degrees (0 - 180)
  targetAzimuth = atoi(strtokIndx); // Convert it to an integer

}
*/


void interpretData() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ","); // Obtain the type of command being received
  commandType = atoi(strtokIndx); // Convert it to an integer

  if (commandType < 0 || commandType > 5) {       // Not a valid commandType? Do nothing
    commandType = 0;
    return;
  }

  if (commandType == 0) {                           // commandType = 0 means do nothing

    return;                                         // No targets required to be set

  }

  if (commandType == 1) {                           // commandType = 1 means a calibration command has been received

    return;                                         // No targets required to be set
  
  }

  if (commandType == 2 || commandType == 3) {                     // commandType = 2 means a manual operation command has been received (same format as commandType = 3)
                                                                  // commandType = 3 means a shoot command has been received (same format as commandType = 2)
    strtokIndx = strtok(NULL, ",");                               // Obtain the first argument
    argument[0] = atof(strtokIndx);                               // Convert it to a float
    targetRPM[0] = argument[0];                                   // Write argument to targetRPM[0]

    strtokIndx = strtok(NULL, ",");                               // Obtain the second argument
    argument[1] = atof(strtokIndx);                               // Convert it to a float
    targetRPM[1] = argument[1];                                   // Write argument to targetRPM[1]

    strtokIndx = strtok(NULL, ",");                               // Obtain the third argument
    argument[2] = atof(strtokIndx);                               // Convert it to a float
    targetRPM[2] = argument[2];                                   // Write argument to targetRPM[2]

    strtokIndx = strtok(NULL, ",");                               // Obtain the fourth argument
    argument[3] = atof(strtokIndx);                               // Convert it to a float
    targetAzimuth = targetAzimuth + argument[3];                  // targetAzimuth (deg) = current target (deg) + argument[3] (deg);
    targetAzimuthPulses = (targetAzimuth * AZIMUTH_RATIO);        // Convert target to encoder pulses

    strtokIndx = strtok(NULL, ",");                               // Obtain the fifth argument
    argument[4] = atof(strtokIndx);                               // Convert it to a float
    targetElevation = targetElevation + argument[4];              // targetElevation (deg) = current target (deg) + argument[4] (deg);
    targetElevationPulses = (targetElevation * ELEVATION_RATIO);  // Convert target to encoder pulses

  }

  if (commandType == 4) {                           // commandType = 4 means a Return-to-home command has been received

    targetAzimuth = 0;                              // Set the target Azimuth to the home position
    targetAzimuthPulses = 0;

    targetElevation = 0;                            // Set the Elevation to the home position
    targetElevationPulses = 0;

  }

    if (commandType == 5) {                           // commandType = 1 means a calibration command has been received

    return;                                         // No targets required to be set
  
  }

}


void calibrateAzimuth() {

  pulseCount[3] = 0;                                // Sets current azimuth position to home (zero pulseCount)
  targetAzimuth = 0;
  targetAzimuthPulses = 0;

}


void calibrateElevation() {
  
  pulseCount[4] = 0;                                // Sets current elevation position to home (zero pulseCount)
  targetElevation = 0;
  targetElevationPulses = 0;

}


void setPosition(int directionPin, bool directionSignal, int controlPin, float controlSignal){

  analogWrite(controlPin, controlSignal);
  digitalWrite(directionPin, directionSignal);

}


void serialOutput() {

  // Serial.print(F("targetRamp[0] = ")); 
  Serial.print(targetRamp[0]);
  Serial.print(" "); 
  // Serial.print(F("filteredCurrVelRPM[0] = ")); 
  Serial.print(filteredCurrVelRPM[0]);
  Serial.print(" "); 
  // Serial.print(F("targetRamp[1] = ")); 
  Serial.print(targetRamp[1]);
  Serial.print(" "); 
  // Serial.print(F("filteredCurrVelRPM[1] = ")); 
  Serial.print(filteredCurrVelRPM[1]);
  Serial.print(" "); 
  // Serial.print(F("targetRamp[2] = ")); 
  Serial.print(targetRamp[2]);
  Serial.print(" "); 
  // Serial.print(F("filteredCurrVelRPM[2] = ")); 
  Serial.print(filteredCurrVelRPM[2]);
  Serial.print(" "); 
  // Serial.print("targetAzimuth = "); 
  Serial.print(targetAzimuth);
  Serial.print(" "); 
  // Serial.print("currAzimoth = "); 
  Serial.print(currAzimuthPulses * AZIMUTH_RATIO_INV);      // convert back to degrees
  Serial.print(" "); 
  // Serial.print("targetElevation = "); 
  Serial.print(targetElevation);
  Serial.print(" "); 
  // Serial.print("currElevation = "); 
  Serial.println(currElevationPulses * ELEVATION_RATIO_INV);      // convert back to degrees
}


/*
void aimAzimuth(int targetAzimuth) {

  if (servoPos[0] > targetAzimuth) {
    for (int pos = servoPos[0]; pos > targetAzimuth; pos -= 1) {   // goes from 180 degrees to 0 degrees
      SERVO_AZIMUTH.write(pos);                           // tell servo to go to position in variable 'pos'
      servoPos[0] = pos;
      //delay(10);                                          // waits 10 ms for the servo to reach the position
    }
  }

  else if (servoPos[0] < targetAzimuth) {
    for (int pos = servoPos[0]; pos < targetAzimuth; pos += 1) {   // goes from 0 degrees to 180 degrees
      SERVO_AZIMUTH.write(pos);                           // tell servo to go to position in variable 'pos'
      servoPos[0] = pos;
      //delay(10);                                          // waits 10 ms for the servo to reach the position
    }
  }
  else {
    return;
  }
}

void aimElevation(int targetElevation) {

  if (servoPos[1] > targetElevation) {
    for (int pos = servoPos[1]; pos > targetElevation; pos -= 1) { // goes from 180 degrees to 0 degrees
      SERVO_ELEVATION.write(pos);                         // tell servo to go to position in variable 'pos'
      servoPos[1] = pos;
      //delay(10);                                          // waits 10 ms for the servo to reach the position
    }
  }

  else if (servoPos[1] < targetElevation) {
    for (int pos = servoPos[1]; pos < targetElevation; pos += 1) { // goes from 0 degrees to 180 degrees
      SERVO_ELEVATION.write(pos);                         // tell servo to go to position in variable 'pos'
      servoPos[1] = targetElevation;
      //delay(10);                                          // waits 10 ms for the servo to reach the position
    }
  }
  else {
    return;
  }
}

void movePiston(int targetAngle) {

  if (servoPos[2] > targetAngle) {
    for (int pos = servoPos[2]; pos > targetElevation; pos -= 1) { // goes from 180 degrees to 0 degrees
      SERVO_FIRE.write(pos);                         // tell servo to go to position in variable 'pos'
      servoPos[2] = pos;
      //delay(5);                                          // waits 10 ms for the servo to reach the position
    }
  }

  else if (servoPos[2] < targetAngle) {
    for (int pos = servoPos[2]; pos < targetElevation; pos += 1) { // goes from 0 degrees to 180 degrees
      SERVO_FIRE.write(pos);                         // tell servo to go to position in variable 'pos'
      servoPos[2] = pos;
      //delay(5);                                          // waits 10 ms for the servo to reach the position
    }
  }
  else {
    return;
  }
}
*/



/*------------ Setup ------------*/

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  while(!Serial); // Waiting for USB Serial

  // Declare pins as input or output (non-arrays)
  // pinMode(LAUNCH_ELEVATION, OUTPUT);
  // pinMode(LAUNCH_AZIMUTH, OUTPUT);
  pinMode(LAUNCH_FIRE, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(testPin, OUTPUT);                         // Test pin to monitor with oscilloscope for timing checks during development

  analogWriteResolution(16);    // PWM output for RPi Pico is 2^16 = 65535
  analogReadResolution(12);

  digitalWrite(LAUNCH_FIRE, LOW);

  for (int k = 0; k < NMOTORS; k++) {

    // Declare pins as input or output (arrays)
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(MOTOR_PWM[k], OUTPUT);
  }

  // Set all motor drivers to initially be at rest by setting their sleep pins LOW
  digitalWrite(MOTOR_ENABLE, LOW);

  // analogWrite(LAUNCH_AZIMUTH, (ANGLE2PWM * 90 + 16000));
  // analogWrite(LAUNCH_ELEVATION, (ANGLE2PWM * 90 + 16000));

  // Azimuth pin declarations
  pinMode(enca[3], INPUT);
  pinMode(encb[3], INPUT);
  pinMode(MOTOR_PWM[3], OUTPUT);
  pinMode(MOTOR_DIR[0], OUTPUT);

  // Elevation in Declarations
  pinMode(enca[4], INPUT);
  pinMode(encb[4], INPUT);
  pinMode(MOTOR_PWM[4], OUTPUT);
  pinMode(MOTOR_DIR[1], OUTPUT);

  // Confirm settings for each PID controller
  PID0.SetOutputLimits(0, 65535);
  PID0.SetSampleTimeUs(Ts_us);
  PID0.SetTunings(Kp[0], Ki[0], Kd[0]);
  PID0.SetMode(PID0.Control::automatic);

  PID1.SetOutputLimits(0, 65535);
  PID1.SetSampleTimeUs(Ts_us);
  PID1.SetTunings(Kp[1], Ki[1], Kd[1]);
  PID1.SetMode(PID1.Control::automatic);

  PID2.SetOutputLimits(0, 65535);
  PID2.SetSampleTimeUs(Ts_us);
  PID2.SetTunings(Kp[2], Ki[2], Kd[2]);
  PID2.SetMode(PID2.Control::automatic);

  PID3.SetOutputLimits(-65535, 65535);
  PID3.SetSampleTimeUs(Ts_us);
  PID3.SetTunings(Kp[3], Ki[3], Kd[3]);
  PID3.SetMode(PID3.Control::automatic);

  PID4.SetOutputLimits(-65535, 65535);
  PID4.SetSampleTimeUs(Ts_us);
  PID4.SetTunings(Kp[4], Ki[4], Kd[4]);
  PID4.SetMode(PID4.Control::automatic);

  // Generate the interrupts for each motor's encoders, rising and falling (CHANGE)
  attachInterrupt(digitalPinToInterrupt(enca[M0]), readEncoder<M0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encb[M0]), readEncoder<M0>, CHANGE);

  attachInterrupt(digitalPinToInterrupt(enca[M1]), readEncoder<M1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encb[M1]), readEncoder<M1>, CHANGE);

  attachInterrupt(digitalPinToInterrupt(enca[M2]), readEncoder<M2>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encb[M2]), readEncoder<M2>, CHANGE);

  // Generate the interrupts for Azimuth motor's encoders, rising and falling (Direction needed)
  attachInterrupt(digitalPinToInterrupt(enca[3]), readAzimuthEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encb[3]), readAzimuthEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readAzimuthEncoder3, FALLING);
  attachInterrupt(digitalPinToInterrupt(encb[3]), readAzimuthEncoder4, FALLING);

  // Generate the interrupts for Elevation motor's encoders, rising and falling (Direction needed)
  attachInterrupt(digitalPinToInterrupt(enca[4]), readElevationEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encb[4]), readElevationEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[4]), readElevationEncoder3, FALLING);
  attachInterrupt(digitalPinToInterrupt(encb[4]), readElevationEncoder4, FALLING);

  // Set the timer intervals in microseconds
  if (Timer0.attachInterruptInterval(Ts_us, calcVelocity0)) {
    Serial.print(F("Starting  Timer0 OK, millis() = ")); Serial.println(millis());
  }
  else
  Serial.println(F("Can't set Timer0. Select another freq. or timer"));

   if (Timer1.attachInterruptInterval(Ts_us, calcVelocity1)) {
    Serial.print(F("Starting  Timer1 OK, millis() = ")); Serial.println(millis());
  }
  else
  Serial.println(F("Can't set Timer1. Select another freq. or timer"));

   if (Timer2.attachInterruptInterval(Ts_us, calcVelocity2)) {
    Serial.print(F("Starting  Timer2 OK, millis() = ")); Serial.println(millis());
  }
  else
  Serial.println(F("Can't set Timer2. Select another freq. or timer"));

  
}


/*------------ Main Loop ------------*/

void loop() {
  // put your main code here, to run repeatedly:

  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // This temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0

    interpretData();    // Parses data received via Serial and assigns targets appropriately

    newData = false;
  }


  if ((micros() - loopTimer1) > Tc_us) {                    // Only run the rest of the main loop once every Tc
    loopTimer1 = micros();

    digitalWrite(testPin, !digitalRead(testPin));           // testPin toggle for monitoring frequency
    serialOutput();                                         // Prints all target and current positions to USB serial


    if (commandType == 0) {                                 // commandType = 0 means do nothing

      digitalWrite(MOTOR_ENABLE, LOW);                      // Set all motor drivers to be at rest by setting their sleep pins LOW
      targetRPM[0] = 0;
      targetRPM[1] = 0;
      targetRPM[2] = 0;
      targetRamp[0] = 0;
      targetRamp[1] = 0;
      targetRamp[2] = 0;

    }




    if (commandType == 1) {                                 // commandType = 1 means a calibration command has been received

      calibrateAzimuth();                                   // Sets current Azimuth position to be zero degrees and zero pulse counts

      calibrateElevation();                                 // Sets current Elevation position to be zero degrees and zero pulse counts

      commandType = 0;                                      // Launcher to go back to doing nothing after home points have been set

    }




    if (commandType == 2) {                                 // commandType = 2 means a manual operation command has been received

      digitalWrite(MOTOR_ENABLE, HIGH);                     // Take the motors out of sleep mode

      for (int k = 0; k < NMOTORS; k++) {

        targetRamp[k] = Ramp(targetRPM[k], k);                  // Convert target velocity step into a ramp function

        currVelRPM[k] = currVel[k] * VEL_RATIO;                 // Convert latest motor velocity sample from pulses per sample period to RPM
          
        filteredCurrVelRPM[k] = LPFilter(currVelRPM[k], k);     // Filter the incoming velocity measurements to smooth the readings

      }

      // Compute control signal for each flywheel and send to motor driver

      PID0.Compute();                                           // Compute the control signal for the motors
      analogWrite(MOTOR_PWM[0], controlSignal0);                // Send the control signal from the PID controller to the motor

      PID1.Compute();                                           // Compute the control signal for the motors
      analogWrite(MOTOR_PWM[1], controlSignal1);                // Send the control signal from the PID controller to the motor

      PID2.Compute();                                           // Compute the control signal for the motors
      analogWrite(MOTOR_PWM[2], controlSignal2);                // Send the control signal from the PID controller to the motor

      // Compute control signal for Azimuth motor and send to motor driver
      currAzimuthPulses = pulseCount[3];
      PID3.Compute();
      direction[0] = LOW;
      if (controlSignal3 < 0) {
        direction[0] = HIGH;
      }
      signalAzi = fabs(controlSignal3);
      setPosition(MOTOR_DIR[0], direction[0], MOTOR_PWM[3], signalAzi);

      // Compute control signal for Elevation motor and send to motor driver
      currElevationPulses = pulseCount[4];
      PID4.Compute();
      direction[1] = LOW;
      if (controlSignal4 < 0) {
        direction[1] = HIGH;
      }
      signalEle = fabs(controlSignal4);
      setPosition(MOTOR_DIR[1], direction[1], MOTOR_PWM[4], signalEle);

    }




    if (commandType == 3) {                                 // commandType = 3 means a shoot command has been received

      digitalWrite(MOTOR_ENABLE, HIGH);                     // Take the motors out of sleep mode

      for (int k = 0; k < NMOTORS; k++) {

        targetRamp[k] = Ramp(targetRPM[k], k);                  // Convert target velocity step into a ramp function

        currVelRPM[k] = currVel[k] * VEL_RATIO;                 // Convert latest motor velocity sample from pulses per sample period to RPM
          
        filteredCurrVelRPM[k] = LPFilter(currVelRPM[k], k);     // Filter the incoming velocity measurements to smooth the readings

      }

      // Compute control signal for each flywheel and send to motor driver

      PID0.Compute();                                           // Compute the control signal for the motors
      analogWrite(MOTOR_PWM[0], controlSignal0);                // Send the control signal from the PID controller to the motor

      PID1.Compute();                                           // Compute the control signal for the motors
      analogWrite(MOTOR_PWM[1], controlSignal1);                // Send the control signal from the PID controller to the motor

      PID2.Compute();                                           // Compute the control signal for the motors
      analogWrite(MOTOR_PWM[2], controlSignal2);                // Send the control signal from the PID controller to the motor

      // Compute control signal for Azimuth motor and send to motor driver
      currAzimuthPulses = pulseCount[3];
      PID3.Compute();
      direction[0] = LOW;
      if (controlSignal3 < 0) {
        direction[0] = HIGH;
      }
      signalAzi = fabs(controlSignal3);
      setPosition(MOTOR_DIR[0], direction[0], MOTOR_PWM[3], signalAzi);

      // Compute control signal for Elevation motor and send to motor driver
      currElevationPulses = pulseCount[4];
      PID4.Compute();
      direction[1] = LOW;
      if (controlSignal4 < 0) {
        direction[1] = HIGH;
      }
      signalEle = fabs(controlSignal4);
      setPosition(MOTOR_DIR[1], direction[1], MOTOR_PWM[4], signalEle);

      launchStepper = launchStepper + 1;

      
      if ((launchStepper) >= (3*Fc)) {                          // Wait 3 seconds for steady state
        
        digitalWrite(LAUNCH_FIRE, HIGH);                            // Launch! (rotate feed mechanism servo 180 deg to extend piston and push ball into flywheels)
      
      }

      if ((launchStepper) >= (4*Fc)) {                              // Wait a second for the ball to clear the launcher
        
        digitalWrite(LAUNCH_FIRE, LOW);                             // Retract the piston to allow the next ball to fall into the launcher

        targetRPM[0] = 0;
        targetRPM[1] = 0;
        targetRPM[2] = 0;                 // Reset the targets to zero to leave the control loop and let the motors coast to a stop
        loopTimer2 = millis();            // Give the LAUNCH_FIRE servo time to retract before putting the servo back to sleep
        commandType = 4;                  // Return Azimuth and Elevation to home position
        digitalWrite(MOTOR_ENABLE, LOW);                       // Allow the servo rest loop to run 
        launchStepper = 0;
      }

    }




    if (commandType == 4) {                                 // commandType = 4 means a return-to-home command has been received

      targetAzimuth = 0;
      targetAzimuthPulses = 0;

      currAzimuthPulses = pulseCount[3];

      PID3.Compute();

      direction[0] = LOW;

      if (controlSignal3 < 0) {
        direction[0] = HIGH;
      }

      signalAzi = fabs(controlSignal3);

      setPosition(MOTOR_DIR[0], direction[0], MOTOR_PWM[3], signalAzi);


      targetElevation = 0;
      targetElevationPulses = 0;

      currElevationPulses = pulseCount[4];

      PID4.Compute();

      direction[1] = LOW;

      if (controlSignal4 < 0) {
        direction[1] = HIGH;
      }

      signalEle = fabs(controlSignal4);

      setPosition(MOTOR_DIR[1], direction[1], MOTOR_PWM[4], signalEle);

    }


    if (commandType == 5) {                                 // commandType = 1 means a calibration command has been received

      launchStepper = launchStepper + 1;

      // analogWrite(LAUNCH_FIRE, 60000);                           // Launch! (rotate feed mechanism servo 180 deg to extend piston and push ball into flywheels)
        digitalWrite(LAUNCH_FIRE, HIGH);                            // Launch! (rotate feed mechanism servo 180 deg to extend piston and push ball into flywheels)

      if ((launchStepper) >= (1*Fc)) {                          // Wait a second for the ball to clear the launcher
      
        // analogWrite(LAUNCH_FIRE, (13250));       // Retract the piston to allow the next ball to fall into the launcher
        digitalWrite(LAUNCH_FIRE, LOW);             // Retract the piston to allow the next ball to fall into the launcher

        commandType = 0;                                      // Launcher to go back to doing nothing after home points have been set
        launchStepper = 0;

      }

    }

  }

}
