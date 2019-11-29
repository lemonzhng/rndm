/*#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]

const float dt = 1.0f / 500.0f; //[s] period between successive calls to MainLoop

//initializes length and coefficient coupling propeller's force to torque
//about propeller's axis of rotation
const float l = 33e-3f;
const float k = 0.01f;

float desHeight = 0.0f;


//*Before* Main Loop

//* this initializes the estimated Gyroscope Bias vector and rate Gyroscope_corrected vector
Vec3f estGyroBias = Vec3f(0, 0, 0);
Vec3f rateGyro_corr = Vec3f(0, 0, 0);

//* initializes command angular accelerations and moment torque vector
Vec3f cmdAngAcc = Vec3f(0,0,0);
Vec3f n = Vec3f(0,0,0);
Vec3f cmdAngVel = Vec3f(0,0,0);

//initializes estimated roll, pitch, and yaw variables
//as well as measured pitch and roll variables and weighting constant ro

float estRoll = 0.0f;
float estPitch = 0.0f;
float estYaw = 0.0f;
float measPitch = 0.0f;
float measRoll = 0.0f;
float ro = 0.01f;

// initializes desired total force and forces on each propeller from mixer
//float des_tot_force = 8.0f*mass;
float cp1 = 0.0f;
float cp2 = 0.0f;
float cp3 = 0.0f;
float cp4 = 0.0f;

// initializes torque variables in x,y,z directions
float nx = 0.0f;
float ny = 0.0f;
float nz = 0.0f;

// initializes angular velocity and angular acceleration in x,y,z directions
float p_des = 0.0f;
float r_des = 0.0f;
float q_des = 0.0f;
float Roll_des = 0.0f;
float Pitch_des = 0.0f;
float Yaw_des = 0.0f;

// defining constant for vertical velocity
float estHeight = 0.0f;
float estVelocity_1 = 0.0f;
float estVelocity_2 = 0.0f;
float estVelocity_3 = 0.0f;
float lastHeightMeas_meas = 0.0f;
float lastHeightMeas_time = 0.0f;

// defining constant for position estimators
float estPosition_x = 0.0f;
float estPosition_y = 0.0f;
float desPosition_x = 0.0f;
float desPosition_y = 0.0f;
float desVelocity_1 = 0.0f;
float desVelocity_2 = 0.0f;



MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Your code goes here


  //*inside* Main Loop

  //initializes time constants for roll, pitch, and yaw rates
  float const timeConstant_rollRate = 0.04f;
  float const timeConstant_pitchRate = timeConstant_rollRate;
  float const timeConstant_yawRate = 0.1f;

  //initializes time constants for roll, pitch, and yaw angle
  float const timeConstant_rollAngle = 0.12f;
  float const timeConstant_pitchAngle = timeConstant_rollAngle;
  float const timeConstant_yawAngle = 0.2f;

  //initializes time constants for horizontal velocity, height , position
  float const timeConst_horizVel = 1.5f;
  float const natFreq_height= 2.0f;
  float const dampingRatio_height= 0.7f;

  float const timeConstant_position= 2.0f;



  //calculated rate gyroscope bias by measuring the rate gyroscope for a second's worth of measurement and assuming
  //the drone wasn't moving
  //the corrected rate gyroscope data was found by subtracting the measured rate gyroscope with the estimated
  //gyroscope bias
  if(in.currentTime < 1.0f) {
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
  }
  rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

  //implements a single axis attitude estimator for the pitch, roll, and yaw angles
  //by integrating the corrected rate gyroscope data with a time step of 0.02
  // and adding it to the current estimated pitch, roll, and yaw values
  // estPitch  = estPitch + dt*rateGyro_corr.y;
  // estRoll  = estRoll + dt*rateGyro_corr.x;



  //calculates pitch and roll angles from accelerometer data in x and y
    measPitch = -in.imuMeasurement.accelerometer.x / gravity;
    measRoll = in.imuMeasurement.accelerometer.y / gravity;

  //implements roll and pitch estimators that factor in measured pitch and roll angles
  // to prevent drifting from occuring
  // ro and (1-ro) show confidence in whether we trust our estimated Euler angles or our measured Euler angles
  // with higher ro meaning that we trust our measured roll and pitch angles more while a smaller ro
  // means we trust our estimated roll and pitch angles more

  // reset all the estimator zero
    if ( in.joystickInput.buttonYellow == true) {
        estRoll= 0.0f;
        estPitch= 0.0f;
        estYaw= 0.0f;
        estPosition_x=0.0f;
        estPosition_y=0.0f;
        estHeight=0.0f;
        estVelocity_1=0.0f;
        estVelocity_2=0.0f;
        estVelocity_3=0.0f;

      }

  //  integrates corrected rate gyro to obtain estimated angle
  estRoll = (estRoll +dt*rateGyro_corr.x) * (1.0f-ro) + ro*(measRoll);
  estPitch = (estPitch +dt*rateGyro_corr.y) * (1.0f-ro) + ro*(measPitch);
  estYaw = estYaw + dt*rateGyro_corr.z;


  // height estimator
  // prediction step
  estHeight= estHeight+estVelocity_3*dt;
  estVelocity_3= estVelocity_3+0.0f*dt;

  // the correction step
  float const mixHeight =0.3f;
  if (in.heightSensor.updated) {
    if (in.heightSensor.value<5.0f) {
      float hMeas= in.heightSensor.value * cosf(estRoll) * cosf(estPitch);
      estHeight = (1-mixHeight)* estHeight +mixHeight*hMeas;

      float v3Meas = (hMeas - lastHeightMeas_meas)
          /(in.currentTime-lastHeightMeas_time);

      estVelocity_3= (1-mixHeight)*estVelocity_3+mixHeight*v3Meas;
      lastHeightMeas_meas= hMeas;
      lastHeightMeas_time=in.currentTime;
    }
  }

  //prediction
  //(just assume velocity is constant)
  estVelocity_1=estVelocity_1+0.0f*dt;
  estVelocity_2=estVelocity_2+0.0f*dt;

  //correction step
  //float const mixHorizVel = 0.1f;
  float const mixHorizVel = 0.3f;

  if (in.opticalFlowSensor.updated) {
    float sigma_1 = in.opticalFlowSensor.value_x;
    float sigma_2 = in.opticalFlowSensor.value_y;
    float div = (cosf(estRoll)*cosf(estPitch));
    if (div>0.5f) {
      float deltaPredict = estHeight/div;
      float v1Meas= (-sigma_1 + in.imuMeasurement.rateGyro.y)*deltaPredict;
      float v2Meas= (-sigma_2 - in.imuMeasurement.rateGyro.x)*deltaPredict;
      estVelocity_1= (1-mixHorizVel)* estVelocity_1 + mixHorizVel* v1Meas;
      estVelocity_2= (1-mixHorizVel)* estVelocity_2 + mixHorizVel* v2Meas;

    }
  }

 // position estimator by integrating velocity

    estPosition_x=estVelocity_1*dt+estPosition_x;
    estPosition_y=estVelocity_2*dt+estPosition_y;

    desVelocity_1= -(1.0f/timeConstant_position)*( estPosition_x - desPosition_x);
    desVelocity_2= -(1.0f/timeConstant_position)*( estPosition_y - desPosition_y);

 // obtaining desired  acceleration from velocity
  float desAcc1= -(1.0f/ timeConst_horizVel)*(estVelocity_1-desVelocity_1);
  float desAcc2= -(1.0f/ timeConst_horizVel)*(estVelocity_2-desVelocity_2);

 // obtain desired roll, pitch and yaw
  Roll_des= -desAcc2/ gravity;
  Pitch_des= desAcc1/ gravity;
  Yaw_des= 0.0f;

 // desired height
  desHeight = 0.75f;

 // for smooth landing
  if ( in.joystickInput.buttonGreen == true) {
     //desHeight = 0.5f;
    desPosition_x = 1.0f;
  }
 // obtaining desired total acceleration from height choosen

  const float desAcc3 = -2.0f *dampingRatio_height* natFreq_height*estVelocity_3
      -natFreq_height*natFreq_height*(estHeight-desHeight);
  float desNormalizedAcceleration = (gravity+desAcc3)/ (cosf(estRoll)*cosf(estPitch));

  // float Pitch_des = 0.0f;
  // implements 30 angle degree tilt when pressing blue button
  //if ( in.joystickInput.buttonBlue == true) {
  //  Roll_des = -0.081f;
  //}

  // Commmand angular velocity
    cmdAngVel.x = (-1.0f/timeConstant_rollAngle)*( estRoll - Roll_des+0.0523f);
    cmdAngVel.y = (-1.0f/timeConstant_pitchAngle)*( estPitch - Pitch_des);
    cmdAngVel.z = (-1.0f/timeConstant_yawAngle)*( estYaw - Yaw_des);

  // Commmand angular acceleration
    cmdAngAcc.x = (-1.0f/timeConstant_rollRate)*(rateGyro_corr.x - cmdAngVel.x);
    cmdAngAcc.y = (-1.0f/timeConstant_pitchRate)*(rateGyro_corr.y - cmdAngVel.y);
    cmdAngAcc.z = (-1.0f/timeConstant_yawRate)*(rateGyro_corr.z - cmdAngVel.z);


  // Calculating torque in x,y,z direction
  nx = 16.0e-6f*cmdAngAcc.x+13.0e-6f*cmdAngVel.y*cmdAngVel.z;
  ny = 16.0e-6f*cmdAngAcc.y-13.0e-6f*cmdAngVel.x*cmdAngVel.z;
  nz = 29.0e-6f*cmdAngAcc.z;

  // Mixer to convert desired torque vector and total force to four individual
  // motor forces
  cp1 = (1.0f/4.0f)*(desNormalizedAcceleration*mass + (nx/l) -(ny/l) + ( nz/k));
  cp2 = (1.0f/4.0f)*(desNormalizedAcceleration*mass - (nx/l) -(ny/l) - ( nz/k));
  cp3 = (1.0f/4.0f)*(desNormalizedAcceleration*mass - (nx/l) +(ny/l) + ( nz/k));
  cp4 = (1.0f/4.0f)*(desNormalizedAcceleration*mass + (nx/l) +(ny/l) - ( nz/k));


  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.joystickInput.buttonBlue" is true if the
  // joystick's blue button is pushed, false otherwise.

  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;
//  motorCommand1 -> located at body +x +y
//  motorCommand2 -> located at body +x -y
//  motorCommand3 -> located at body -x -y
//  motorCommand4 -> located at body -x +y
//  outVals.motorCommand1 = 0.0f;
//  outVals.motorCommand2 = 0.0f;
//  outVals.motorCommand3 = 0.0f;
//  outVals.motorCommand4 = 0.0f;

  //adds estimate angles to the telemetry channels
    outVals.telemetryOutputs_plusMinus100[0] = estRoll;
    outVals.telemetryOutputs_plusMinus100[1] = estPitch;
    outVals.telemetryOutputs_plusMinus100[2] = estHeight;


 // adds command angular accelerations to the telemetry channels
    outVals.telemetryOutputs_plusMinus100[3] = Roll_des;
    outVals.telemetryOutputs_plusMinus100[4] = Pitch_des;
 //   outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
    outVals.telemetryOutputs_plusMinus100[5] = estVelocity_1;
 // adds command angular velocity to the telemetry channels
    outVals.telemetryOutputs_plusMinus100[6] = estVelocity_2;
    outVals.telemetryOutputs_plusMinus100[7] = desVelocity_1;
    outVals.telemetryOutputs_plusMinus100[8] = desVelocity_2;

 // adds command pitch desired to the telemetry channels
 //   outVals.telemetryOutputs_plusMinus100[9] = desNormalizedAcceleration;
    outVals.telemetryOutputs_plusMinus100[9] = estPosition_x;
    outVals.telemetryOutputs_plusMinus100[10] = estPosition_y;
    outVals.telemetryOutputs_plusMinus100[11] = estYaw;

  //if ( in.joystickInput.buttonBlue == true) {
      outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(cp1));
      outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(cp2));
      outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(cp3));
      outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(cp4));

  //if ( in.joystickInput.buttonBlue == true) {
  //  outVals.motorCommand1 = 130.0f;
  //  outVals.motorCommand2 = 130.0f;
  //  outVals.motorCommand3 = 130.0f;
  //  outVals.motorCommand4 = 130.0f;
  // }


  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;
  return outVals;
}


void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));


  //prints accelerometer measurement and rate gyroscope measurements in x,y,z directions
  printf("Acc: ");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("\n");  //new line
  printf("y=%6.3f, ",
           double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("\n");  //new line
  printf("z=%6.3f, ",
           double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
  printf("\n");  //new line
  printf("Gyro: ");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("\n");  //new line
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("\n");  //new line
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
  printf("\n");  //new line



//prints estimated rateGyro bias in x,y,z directions

  printf("Estimated Bias: ");
  printf("x=%6.3f, ",
             double(estGyroBias.x));
  printf("\n");  //new line
  printf("y=%6.3f, ",
             double(estGyroBias.y));
  printf("\n");  //new line
  printf("z=%6.3f, ",
             double(estGyroBias.z));
  printf("\n");  //new line

//prints corrected rate gyroscope measurements
  printf("Corrected Rate Gyro: ");
    printf("x=%6.3f, ",
             double(rateGyro_corr.x));
    printf("\n");  //new line
    printf("y=%6.3f, ",
             double(rateGyro_corr.y));
    printf("\n");  //new line
    printf("z=%6.3f, ",
             double(rateGyro_corr.z));
    printf("\n");  //new line

 // prints estimated Euler angles
    printf("Estimated Roll: ");
       printf("x=%6.3f, ",
                double(estRoll));
       printf("\n");  //new line
       printf("Estimated Pitch: ");
       printf("y=%6.3f, ",
                  double(estPitch));
       printf("\n");  //new line
       printf("Estimated Yaw: ");
       printf("z=%6.3f, ",
                  double(estYaw));
       printf("\n");  //new line

 // prints cp constants
          printf("cp1: ");
             printf("cp1=%6.3f, ",
                      double(cp1));
          printf("cp2: ");
             printf("cp2=%6.3f, ",
                      double(cp2));
          printf("cp3: ");
             printf("cp3=%6.3f, ",
                      double(cp3));
          printf("cp4: ");
             printf("cp4=%6.3f, ",
                      double(cp4));




 // printf("Example variable values:\n");
 // printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
 // printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //We print the Vec3f by printing it's three components independently:
  //printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
  //       double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
  //       double(exampleVariable_Vec3f.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");
  if (lastMainLoopInputs.joystickInput.buttonRed)
    printf("buttonRed ");
  if (lastMainLoopInputs.joystickInput.buttonGreen)
    printf("buttonGreen ");
  if (lastMainLoopInputs.joystickInput.buttonBlue)
    printf("buttonBlue ");
  if (lastMainLoopInputs.joystickInput.buttonYellow)
    printf("buttonYellow ");
  if (lastMainLoopInputs.joystickInput.buttonStart)
    printf("buttonStart ");
  if (lastMainLoopInputs.joystickInput.buttonSelect)
    printf("buttonSelect ");
  printf("\n");
  printf("Last main loop outputs:\n");

  //prints out the motor commands for all 4 motors
  printf("  motor command 1 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand1));
  printf("  motor command 2 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand2));
  printf("  motor command 3 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand3));
  printf("  motor command 4 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand4));
  printf("  Last range = %6.3fm, ",
           double(lastMainLoopInputs.heightSensor.value));
  printf("  Last flow: x= %6.3f, y=%6.3f\n",
           double(lastMainLoopInputs.opticalFlowSensor.value_x),
           double(lastMainLoopInputs.opticalFlowSensor.value_y));

  // prints position values
           printf("est position x: ");
              printf("est position x=%6.3f, ",
                       double(estPosition_x));
           printf("est position y: ");
              printf("est position y=%6.3f, ",
                       double(estPosition_y));
           printf("des position x: ");
              printf("des position x=%6.3f, ",
                       double(desPosition_x));
           printf("des position y: ");
              printf("des position y=%6.3f, ",
                       double(desPosition_y));

} */

#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]

const float dt = 1.0f / 500.0f; //[s] period between successive calls to MainLoop

//initializes length and coefficient coupling propeller's force to torque
//about propeller's axis of rotation
const float l = 33e-3f;
const float k = 0.01f;

float desHeight = 0.0f;


//*Before* Main Loop

//* this initializes the estimated Gyroscope Bias vector and rate Gyroscope_corrected vector
Vec3f estGyroBias = Vec3f(0, 0, 0);
Vec3f rateGyro_corr = Vec3f(0, 0, 0);

//* initializes command angular accelerations and moment torque vector
Vec3f cmdAngAcc = Vec3f(0,0,0);
Vec3f n = Vec3f(0,0,0);
Vec3f cmdAngVel = Vec3f(0,0,0);

//initializes estimated roll, pitch, and yaw variables
//as well as measured pitch and roll variables and weighting constant ro

float estRoll = 0.0f;
float estPitch = 0.0f;
float estYaw = 0.0f;
float measPitch = 0.0f;
float measRoll = 0.0f;
float ro = 0.01f;

// initializes desired total force and forces on each propeller from mixer
//float des_tot_force = 8.0f*mass;
float cp1 = 0.0f;
float cp2 = 0.0f;
float cp3 = 0.0f;
float cp4 = 0.0f;

// initializes torque variables in x,y,z directions
float nx = 0.0f;
float ny = 0.0f;
float nz = 0.0f;

// initializes angular velocity and angular acceleration in x,y,z directions
float p_des = 0.0f;
float r_des = 0.0f;
float q_des = 0.0f;
float Roll_des = 0.0f;
float Pitch_des = 0.0f;
float Yaw_des = 0.0f;

// defining constant for vertical velocity
float estHeight = 0.0f;
float estVelocity_1 = 0.0f;
float estVelocity_2 = 0.0f;
float estVelocity_3 = 0.0f;
float lastHeightMeas_meas = 0.0f;
float lastHeightMeas_time = 0.0f;

// defining constant for position estimators
float estPosition_x = 0.0f;
float estPosition_y = 0.0f;
float const desPosition_x = 0.0f;
float const desPosition_y = 0.0f;
float desVelocity_1 = 0.0f;
float desVelocity_2 = 0.0f;



MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Your code goes here


  //*inside* Main Loop

  //initializes time constants for roll, pitch, and yaw rates
  float const timeConstant_rollRate = 0.04f;
  float const timeConstant_pitchRate = timeConstant_rollRate;
  float const timeConstant_yawRate = 0.1f;

  //initializes time constants for roll, pitch, and yaw angle
  float const timeConstant_rollAngle = 0.12f;
  float const timeConstant_pitchAngle = timeConstant_rollAngle;
  float const timeConstant_yawAngle = 0.2f;

  //initializes time constants for horizontal velocity, height , position
  float const timeConst_horizVel = 1.5f;
  float const natFreq_height= 2.0f;
  float const dampingRatio_height= 0.7f;

  float const timeConstant_position= 2.0f;



  //calculated rate gyroscope bias by measuring the rate gyroscope for a second's worth of measurement and assuming
  //the drone wasn't moving
  //the corrected rate gyroscope data was found by subtracting the measured rate gyroscope with the estimated
  //gyroscope bias
  if(in.currentTime < 1.0f) {
    estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
  }
  rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;

  //implements a single axis attitude estimator for the pitch, roll, and yaw angles
  //by integrating the corrected rate gyroscope data with a time step of 0.02
  // and adding it to the current estimated pitch, roll, and yaw values
  // estPitch  = estPitch + dt*rateGyro_corr.y;
  // estRoll  = estRoll + dt*rateGyro_corr.x;



  //calculates pitch and roll angles from accelerometer data in x and y
    measPitch = -in.imuMeasurement.accelerometer.x / gravity;
    measRoll = in.imuMeasurement.accelerometer.y / gravity;

  //implements roll and pitch estimators that factor in measured pitch and roll angles
  // to prevent drifting from occuring
  // ro and (1-ro) show confidence in whether we trust our estimated Euler angles or our measured Euler angles
  // with higher ro meaning that we trust our measured roll and pitch angles more while a smaller ro
  // means we trust our estimated roll and pitch angles more

  // reset all the estimator zero
    if ( in.joystickInput.buttonYellow == true) {
        estRoll= 0.0f;
        estPitch= 0.0f;
        estYaw= 0.0f;
        estPosition_x=0.0f;
        estPosition_y=0.0f;
        estHeight=0.0f;
        estVelocity_1=0.0f;
        estVelocity_2=0.0f;
        estVelocity_3=0.0f;

      }

  //  integrates corrected rate gyro to obtain estimated angle
  estRoll = (estRoll +dt*rateGyro_corr.x) * (1.0f-ro) + ro*(measRoll);
  estPitch = (estPitch +dt*rateGyro_corr.y) * (1.0f-ro) + ro*(measPitch);
  estYaw = estYaw + dt*rateGyro_corr.z;


  // height estimator
  // prediction step
  estHeight= estHeight+estVelocity_3*dt;
  estVelocity_3= estVelocity_3+0.0f*dt;

  // the correction step
  float const mixHeight =0.3f;
  if (in.heightSensor.updated) {
    if (in.heightSensor.value<5.0f) {
      float hMeas= in.heightSensor.value * cosf(estRoll) * cosf(estPitch);
      estHeight = (1-mixHeight)* estHeight +mixHeight*hMeas;

      float v3Meas = (hMeas - lastHeightMeas_meas)
          /(in.currentTime-lastHeightMeas_time);

      estVelocity_3= (1-mixHeight)*estVelocity_3+mixHeight*v3Meas;
      lastHeightMeas_meas= hMeas;
      lastHeightMeas_time=in.currentTime;
    }
  }

  //prediction
  //(just assume velocity is constant)
  estVelocity_1=estVelocity_1+0.0f*dt;
  estVelocity_2=estVelocity_2+0.0f*dt;

  //correction step
  float const mixHorizVel = 0.1f;
  if (in.opticalFlowSensor.updated) {
    float sigma_1 = in.opticalFlowSensor.value_x;
    float sigma_2 = in.opticalFlowSensor.value_y;
    float div = (cosf(estRoll)*cosf(estPitch));
    if (div>0.5f) {
      float deltaPredict = estHeight/div;
      float v1Meas= (-sigma_1 + in.imuMeasurement.rateGyro.y)*deltaPredict;
      float v2Meas= (-sigma_2 - in.imuMeasurement.rateGyro.x)*deltaPredict;
      estVelocity_1= (1-mixHorizVel)* estVelocity_1 + mixHorizVel* v1Meas;
      estVelocity_2= (1-mixHorizVel)* estVelocity_2 + mixHorizVel* v2Meas;

    }
  }

 // position estimator by integrating velocity

    estPosition_x=estVelocity_1*dt+estPosition_x;
    estPosition_y=estVelocity_2*dt+estPosition_y;

    desVelocity_1= -(1.0f/timeConstant_position)*( estPosition_x - desPosition_x);
    desVelocity_2= -(1.0f/timeConstant_position)*( estPosition_y - desPosition_y);

 // obtaining desired  acceleration from velocity
  float desAcc1= -(1.0f/ timeConst_horizVel)*(estVelocity_1-desVelocity_1);
  float desAcc2= -(1.0f/ timeConst_horizVel)*(estVelocity_2-desVelocity_2);

 // obtain desired roll, pitc and yaw
  Roll_des= -desAcc2/ gravity;
  Pitch_des= desAcc1/ gravity;
  Yaw_des= 0.0f;

 // desired height
  desHeight = 1.0f;

 // for smooth landing
  if ( in.joystickInput.buttonGreen == true) {
     desHeight = 0.5f;
  }
 // obtaining desired total acceleration from height choosen

  const float desAcc3 = -2.0f *dampingRatio_height* natFreq_height*estVelocity_3
      -natFreq_height*natFreq_height*(estHeight-desHeight);
  float desNormalizedAcceleration = (gravity+desAcc3)/ (cosf(estRoll)*cosf(estPitch));

  // float Pitch_des = 0.0f;
  // implements 30 angle degree tilt when pressing blue button
  //if ( in.joystickInput.buttonBlue == true) {
  //  Roll_des = -0.081f;
  //}

  // Commmand angular velocity
    cmdAngVel.x = (-1.0f/timeConstant_rollAngle)*( estRoll - Roll_des+0.0523f);
    cmdAngVel.y = (-1.0f/timeConstant_pitchAngle)*( estPitch - Pitch_des);
    cmdAngVel.z = (-1.0f/timeConstant_yawAngle)*( estYaw - Yaw_des);

  // Commmand angular acceleration
    cmdAngAcc.x = (-1.0f/timeConstant_rollRate)*(rateGyro_corr.x - cmdAngVel.x);
    cmdAngAcc.y = (-1.0f/timeConstant_pitchRate)*(rateGyro_corr.y - cmdAngVel.y);
    cmdAngAcc.z = (-1.0f/timeConstant_yawRate)*(rateGyro_corr.z - cmdAngVel.z);


  // Calculating torque in x,y,z direction
  nx = 16.0e-6f*cmdAngAcc.x+13.0e-6f*cmdAngVel.y*cmdAngVel.z;
  ny = 16.0e-6f*cmdAngAcc.y-13.0e-6f*cmdAngVel.x*cmdAngVel.z;
  nz = 29.0e-6f*cmdAngAcc.z;

  // Mixer to convert desired torque vector and total force to four individual
  // motor forces
  cp1 = (1.0f/4.0f)*(desNormalizedAcceleration*mass + (nx/l) -(ny/l) + ( nz/k));
  cp2 = (1.0f/4.0f)*(desNormalizedAcceleration*mass - (nx/l) -(ny/l) - ( nz/k));
  cp3 = (1.0f/4.0f)*(desNormalizedAcceleration*mass - (nx/l) +(ny/l) + ( nz/k));
  cp4 = (1.0f/4.0f)*(desNormalizedAcceleration*mass + (nx/l) +(ny/l) - ( nz/k));


  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.joystickInput.buttonBlue" is true if the
  // joystick's blue button is pushed, false otherwise.

  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;
//  motorCommand1 -> located at body +x +y
//  motorCommand2 -> located at body +x -y
//  motorCommand3 -> located at body -x -y
//  motorCommand4 -> located at body -x +y
//  outVals.motorCommand1 = 0.0f;
//  outVals.motorCommand2 = 0.0f;
//  outVals.motorCommand3 = 0.0f;
//  outVals.motorCommand4 = 0.0f;

  //adds estimate angles to the telemetry channels
    outVals.telemetryOutputs_plusMinus100[0] = estRoll;
    outVals.telemetryOutputs_plusMinus100[1] = estPitch;
    outVals.telemetryOutputs_plusMinus100[2] = estHeight;


 // adds command angular accelerations to the telemetry channels
    outVals.telemetryOutputs_plusMinus100[3] = Roll_des;
    outVals.telemetryOutputs_plusMinus100[4] = Pitch_des;
 //   outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
    outVals.telemetryOutputs_plusMinus100[5] = estVelocity_1;
 // adds command angular velocity to the telemetry channels
    outVals.telemetryOutputs_plusMinus100[6] = estVelocity_2;
    outVals.telemetryOutputs_plusMinus100[7] = desVelocity_1;
    outVals.telemetryOutputs_plusMinus100[8] = desVelocity_2;

 // adds command pitch desired to the telemetry channels
 //   outVals.telemetryOutputs_plusMinus100[9] = desNormalizedAcceleration;
    outVals.telemetryOutputs_plusMinus100[9] = estPosition_x;
    outVals.telemetryOutputs_plusMinus100[10] = estPosition_y;
    outVals.telemetryOutputs_plusMinus100[11] = estYaw;

  //if ( in.joystickInput.buttonBlue == true) {
      outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(cp1));
      outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(cp2));
      outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(cp3));
      outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(cp4));

  //if ( in.joystickInput.buttonBlue == true) {
  //  outVals.motorCommand1 = 130.0f;
  //  outVals.motorCommand2 = 130.0f;
  //  outVals.motorCommand3 = 130.0f;
  //  outVals.motorCommand4 = 130.0f;
  // }


  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;
  return outVals;
}


void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));


  //prints accelerometer measurement and rate gyroscope measurements in x,y,z directions
  printf("Acc: ");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("\n");  //new line
  printf("y=%6.3f, ",
           double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("\n");  //new line
  printf("z=%6.3f, ",
           double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
  printf("\n");  //new line
  printf("Gyro: ");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("\n");  //new line
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("\n");  //new line
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
  printf("\n");  //new line



//prints estimated rateGyro bias in x,y,z directions

  printf("Estimated Bias: ");
  printf("x=%6.3f, ",
             double(estGyroBias.x));
  printf("\n");  //new line
  printf("y=%6.3f, ",
             double(estGyroBias.y));
  printf("\n");  //new line
  printf("z=%6.3f, ",
             double(estGyroBias.z));
  printf("\n");  //new line

//prints corrected rate gyroscope measurements
  printf("Corrected Rate Gyro: ");
    printf("x=%6.3f, ",
             double(rateGyro_corr.x));
    printf("\n");  //new line
    printf("y=%6.3f, ",
             double(rateGyro_corr.y));
    printf("\n");  //new line
    printf("z=%6.3f, ",
             double(rateGyro_corr.z));
    printf("\n");  //new line

 // prints estimated Euler angles
    printf("Estimated Roll: ");
       printf("x=%6.3f, ",
                double(estRoll));
       printf("\n");  //new line
       printf("Estimated Pitch: ");
       printf("y=%6.3f, ",
                  double(estPitch));
       printf("\n");  //new line
       printf("Estimated Yaw: ");
       printf("z=%6.3f, ",
                  double(estYaw));
       printf("\n");  //new line

 // prints cp constants
          printf("cp1: ");
             printf("cp1=%6.3f, ",
                      double(cp1));
          printf("cp2: ");
             printf("cp2=%6.3f, ",
                      double(cp2));
          printf("cp3: ");
             printf("cp3=%6.3f, ",
                      double(cp3));
          printf("cp4: ");
             printf("cp4=%6.3f, ",
                      double(cp4));




 // printf("Example variable values:\n");
 // printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
 // printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //We print the Vec3f by printing it's three components independently:
  //printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
  //       double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
  //       double(exampleVariable_Vec3f.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");
  if (lastMainLoopInputs.joystickInput.buttonRed)
    printf("buttonRed ");
  if (lastMainLoopInputs.joystickInput.buttonGreen)
    printf("buttonGreen ");
  if (lastMainLoopInputs.joystickInput.buttonBlue)
    printf("buttonBlue ");
  if (lastMainLoopInputs.joystickInput.buttonYellow)
    printf("buttonYellow ");
  if (lastMainLoopInputs.joystickInput.buttonStart)
    printf("buttonStart ");
  if (lastMainLoopInputs.joystickInput.buttonSelect)
    printf("buttonSelect ");
  printf("\n");
  printf("Last main loop outputs:\n");

  //prints out the motor commands for all 4 motors
  printf("  motor command 1 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand1));
  printf("  motor command 2 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand2));
  printf("  motor command 3 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand3));
  printf("  motor command 4 = %6.3f\n",
           double(lastMainLoopOutputs.motorCommand4));
  printf("  Last range = %6.3fm, ",
           double(lastMainLoopInputs.heightSensor.value));
  printf("  Last flow: x= %6.3f, y=%6.3f\n",
           double(lastMainLoopInputs.opticalFlowSensor.value_x),
           double(lastMainLoopInputs.opticalFlowSensor.value_y));

  // prints position values
           printf("est position x: ");
              printf("est position x=%6.3f, ",
                       double(estPosition_x));
           printf("est position y: ");
              printf("est position y=%6.3f, ",
                       double(estPosition_y));
           printf("des position x: ");
              printf("des position x=%6.3f, ",
                       double(desPosition_x));
           printf("des position y: ");
              printf("des position y=%6.3f, ",
                       double(desPosition_y));

}

