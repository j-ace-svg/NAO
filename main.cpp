#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.



// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"

// Included libraries (some redundent because of config block)
#include <math.h>
#include <array>
using std::array;
  
// Allows for easier use of the VEX Library
using namespace vex;

/* --------------- Start drive library --------------- */

#pragma region Custom Drive Library

#define INF std::numeric_limits<float>::infinity()
#define DT 10

using transformMatrix = array<array<float, 2>, 2>;

struct coordinate {
  float x;
  float y;

  coordinate(float x, float y) : x(x), y(y) {
  }

  coordinate& operator=(const coordinate& a) {
    x = a.x;
    y = a.y;
    return *this;
  }

  coordinate operator+(const coordinate& a) {
    return coordinate(a.x + x, a.y + y);
  }

  coordinate operator-(const coordinate& a) {
    return coordinate(x - a.x, y - a.y);
  }

  bool operator==(const coordinate& a) {
    return (x == a.x && y == a.y);
  }

  coordinate operator*(const transformMatrix& trans) {
    return coordinate(x * trans[0][0] + y * trans[1][0], x * trans[0][1] + y * trans[1][1]);
  }
};

class Odometry {
  private:
    float resetOrientGlobal; // Global orientation at last reset
    //float orientGlobalDrift;
    float oldTimestamp;
    float oldLeftAngle;
    float oldRightAngle;
    float oldOrientGlobal;
    coordinate oldGlobalPosition = {0, 0}; // Previous global position vector
    float timestamp;
    float leftAngle;
    float rightAngle;
    float orientGlobal;
    coordinate globalPosition = {0, 0};
    float slippingEpsilon = 0.01;

  public:
    motor_group* leftDrive;
    motor_group* rightDrive;
    inertial* inertialSensor;
    float orientGlobalDrift;
    float inertialDriftEpsilon; // Minimum threshold used to determine whether or not turning is drift
    float distLeft; // Distance from tracking center to left tracking wheel
    float distRight; // Distance from tracking center to right tracking wheel
    float distBack; // Distance from tracking center to back tracking wheel
    float leftWheelRadius;
    float rightWheelRadius;

    Odometry(motor_group &_leftDrive, motor_group &_rightDrive, inertial &_inertialSensor, float _inertialDriftEpsilon, float _distLeft, float _distRight, float _distBack, float _leftWheelRadius, float _rightWheelRadius) {
      leftDrive = &_leftDrive;
      rightDrive = &_rightDrive;
      inertialSensor = &_inertialSensor;
      inertialDriftEpsilon = _inertialDriftEpsilon;
      distLeft = _distLeft;
      distRight = _distRight;
      leftWheelRadius = _leftWheelRadius;
      rightWheelRadius = _rightWheelRadius;
    }

    void initSensorValues() {
      timestamp = Brain.Timer.time(seconds);
      leftAngle = leftDrive->position(turns) * 2 * M_PI;
      rightAngle = rightDrive->position(turns) * 2 * M_PI;
      orientGlobal = inertialSensor->rotation(turns) * 2 * M_PI;
      orientGlobalDrift = 0;
      globalPosition = {0, 0};
      oldTimestamp = timestamp;
      oldLeftAngle = leftAngle;
      oldRightAngle = rightAngle;
      oldOrientGlobal = orientGlobal;
      resetOrientGlobal = orientGlobal;
      oldGlobalPosition = globalPosition;
    }

    void pollSensorValues() {
      oldTimestamp = timestamp;
      oldLeftAngle = leftAngle;
      oldRightAngle = rightAngle;
      oldOrientGlobal = orientGlobal;
      timestamp = Brain.Timer.time(seconds);
      leftAngle = leftDrive->position(turns) * 2 * M_PI;
      rightAngle = rightDrive->position(turns) * 2 * M_PI;
      orientGlobal = inertialSensor->rotation(turns) * 2 * M_PI;

      if (fabs(orientGlobal - oldOrientGlobal) < inertialDriftEpsilon) {
        orientGlobalDrift += orientGlobal - oldOrientGlobal;
      }

      // Calculate odometry
      //oldGlobalPosition = globalPosition;
      //globalPosition = globalPosition + getGlobalPositionChange();
    }

    // Setter methods

    void resetOrientation() {
      resetOrientGlobal = orientGlobal;
    }

    void resetOrientation(float newOrientation) {
      resetOrientGlobal = orientGlobal - newOrientation;
    }

    // Getter methods

    float getLeftAngle() {
      return leftAngle;
    }

    float getLeftDistance() {
      return leftAngle * leftWheelRadius;
    }

    float getRightAngle() {
      return rightAngle;
    }

    float getRightDistance() {
      return rightAngle * rightWheelRadius;
    }

    float getOrientation() {
      return orientGlobal - resetOrientGlobal - orientGlobalDrift;
    }

    float getOldOrientation() {
      return oldOrientGlobal;
    }

    coordinate getOldGlobalPosition() {
      return oldGlobalPosition;
    }

    coordinate getGlobalPosition() {
      return globalPosition;
    }

    float getDeltaTime() {
      return timestamp - oldTimestamp;
    }

    float getDeltaLeftAngle() {
      return leftAngle - oldLeftAngle;
    }

    float getDeltaLeftDistance() {
      return getDeltaLeftAngle() * leftWheelRadius;
    }

    float getDeltaRightAngle() {
      return rightAngle - oldRightAngle;
    }

    float getDeltaRightDistance() {
      return getDeltaRightAngle() * rightWheelRadius;
    }

    float getDeltaOrientation() {
      return orientGlobal - oldOrientGlobal;
    }

    coordinate getDeltaGlobalPosition() {
      return globalPosition - oldGlobalPosition;
    }

    float getPredictedDeltaOrientation() {
      float leftRightAngleDifference = getDeltaLeftAngle() - getDeltaRightAngle();
      float drivetrainWidth = distLeft + distRight;
      return leftRightAngleDifference / drivetrainWidth;
    }

    float getPathArcRadius() {
      if (getDeltaOrientation() == 0) return INF;
      if (getDeltaLeftAngle() == 0 && getDeltaRightAngle() == 0) return INF;
      float leftRadius = getDeltaLeftDistance() / getDeltaOrientation() - distLeft;
      float leftVelocity = leftRadius * getDeltaOrientation() / getDeltaTime();
      float leftAcceleration = pow(leftVelocity, 2) / leftRadius;
      float rightRadius = getDeltaRightDistance() / getDeltaOrientation() + distRight;
      float rightVelocity = rightRadius * getDeltaOrientation() / getDeltaTime();
      float rightAcceleration = pow(rightVelocity, 2) / rightRadius;
      
      float inertialAcceleration = inertialSensor->acceleration(yaxis);

      float minSideRadiusAcceleration = leftAcceleration;
      float minSideRadiusVelocity = leftVelocity;
      float minSideRadius = leftRadius;
      float minSideDeltaAngle = getDeltaLeftDistance();
      float minSideDist = distLeft;

      if (leftRadius > rightRadius) { // One side slipped more than the other
        minSideRadiusAcceleration = rightAcceleration;
        minSideRadiusVelocity = rightVelocity;
        minSideRadius = rightRadius;
        minSideDeltaAngle = getDeltaRightDistance();
        minSideDist = distRight;
      }
      if (fabs(minSideRadiusAcceleration - inertialAcceleration) > slippingEpsilon) { // Slippage on both sides
        minSideRadiusAcceleration = inertialAcceleration;
        minSideRadiusVelocity = sqrt(minSideRadiusAcceleration * minSideRadius);
        minSideRadius = minSideRadiusVelocity * getDeltaTime() / getDeltaOrientation();
      }

      return minSideRadius;
    }

    coordinate getGlobalPositionChange() {
      coordinate localChange = {0, 2 * sinf(getDeltaOrientation() / 2) * getPathArcRadius()};

      float localRotationOffset = getOldOrientation() + getDeltaOrientation() / 2;

      // The extra pair of curly braces around the initializer in the line below is actually necessary (because c funkiness)
      transformMatrix rotationMatrix = {{{cosf(-localRotationOffset), -sinf(-localRotationOffset)}, {sinf(-localRotationOffset), cosf(-localRotationOffset)}}};

      coordinate globalChange = localChange * rotationMatrix;
      if (getPathArcRadius() == INF) {
        float deltaLeftDist = getDeltaLeftDistance();
        float deltaRightDist = getDeltaRightDistance();
        if ((deltaLeftDist < 0 && deltaRightDist > 0) || (deltaLeftDist > 0 && deltaRightDist < 0)) { // Forwards and Backwards (no movement)
          globalChange = {0, 0};
        } else if (deltaLeftDist == 0 || deltaRightDist == 0) { // No movement
          globalChange = {0, 0};
        } else if (deltaLeftDist > 0) { // Forwards
          if (deltaLeftDist > deltaRightDist) {
            globalChange = {0, deltaRightDist};
          } else {
            globalChange = {0, deltaLeftDist};
          }
        } else { // Backwards
          if (deltaLeftDist > deltaRightDist) {
            globalChange = {0, deltaLeftDist};
          } else {
            globalChange = {0, deltaRightDist};
          }
        }
      }
      return globalChange;
    }
};

class PID {
  //private:
  public:
    float kp;
    float ki;
    float kd;
    float integralRange;
    float previousError;
    float accumulatedError;
    float timeRunning;
    float timeout;
    float settleThreshold;
    float settleTime;
    float timeSettled;

  //public:

    PID(float startError, float _kp, float _ki, float _kd, float _integralRange, float _settleThreshold, float _settleTime) {
      kp = _kp;
      ki = _ki;
      kd = _kd;
      integralRange = _integralRange;
      settleThreshold = _settleThreshold;
      settleTime = _settleTime;
      accumulatedError = 0;
      previousError = startError;
      timeRunning = 0;
      timeSettled = 0;
    }

    bool isSettled() {
      if ((timeRunning > timeout && timeout != 0) || timeSettled > settleTime) return true;
      return false;
    }

    float calculateNextStep(float error) {
      // Integral
      accumulatedError = accumulatedError + error;
      if (fabs(error) > integralRange) accumulatedError = 0; // Error outside of range for accumulating integral
      if (error == 0 || (error > 0 &&  previousError < 0) || (error < 0 && previousError > 0)) accumulatedError = 0; // Error crossed 0

      // Derivative
      float deltaError = error - previousError;
      previousError = error;

      // Output
      float outputPower = kp * error + ki * accumulatedError + kd * deltaError;
      if (fabs(error) < settleThreshold) timeSettled += DT;
      timeRunning += DT;
      return outputPower;
    }
};

struct odomParameters {
  float kp;
  float ki;
  float kd;
  float integralRange;
  float settleThreshold;
  float settleTime;
  float maxVelocity;

  odomParameters(float kp, float ki, float kd, float integralRange, float settleThreshold, float settleTime, float maxVelocity) : kp(kp), ki(ki), kd(kd), integralRange(integralRange), settleThreshold(settleThreshold), settleTime(settleTime), maxVelocity(maxVelocity) {
  }

};

class Drive {
  public:
    motor_group* leftDrive;
    motor_group* rightDrive;
    directionType leftDirection = forward;
    directionType rightDirection = reverse;
    inertial* inertialSensor;
    
    controller* remoteControl;

    Odometry* odom;
    odomParameters straightParameters = {0, 0, 0, 0, 0, 0, 0};
    odomParameters turnParameters = {0, 0, 0, 0, 0, 0, 0};
    odomParameters headingParameters = {0, 0, 0, 0, 0, 0, 0};

  Drive(motor_group &_leftDrive, motor_group &_rightDrive, inertial &_inertialSensor, controller &_remoteControl) {
    leftDrive = &_leftDrive;
    rightDrive = &_rightDrive;
    inertialSensor = &_inertialSensor;
    remoteControl = &_remoteControl;
  }

  Drive(motor_group &_leftDrive, motor_group &_rightDrive, directionType _leftDirection, directionType _rightDirection, inertial &_inertialSensor, controller &_remoteControl) {
    leftDrive = &_leftDrive;
    rightDrive = &_rightDrive;
    leftDirection = _leftDirection;
    rightDirection = _rightDirection;
    inertialSensor = &_inertialSensor;
    remoteControl = &_remoteControl;
  }

  void initOdom(float inertialDriftEpsilon, float distLeft, float distRight, float distBack, float leftWheelRadius, float rightWheelRadius, odomParameters _straightParameters, odomParameters _turnParameters, odomParameters _headingParameters) {
    odom = new Odometry(*leftDrive, *rightDrive, *inertialSensor, inertialDriftEpsilon, distLeft, distRight, distBack, leftWheelRadius, rightWheelRadius);
    odom->initSensorValues();
    straightParameters = _straightParameters;
    turnParameters = _turnParameters;
    headingParameters = _headingParameters;
  }

  void driverControl() {
    leftDrive->setVelocity(remoteControl->Axis3.position(), percent);
    if (abs(remoteControl->Axis3.position()) > 3) {
      leftDrive->spin(leftDirection);
    }
    rightDrive->setVelocity(remoteControl->Axis2.position(), percent);
    if (abs(remoteControl->Axis2.position()) > 3) {
      rightDrive->spin(rightDirection);
    }
  }

  // General drive functions
  void driveVelocity(float leftMotorPower, float rightMotorPower) {
    leftDrive->setVelocity(leftMotorPower, percent);
    if (leftMotorPower == 0) {
      leftDrive->stop(hold);
    } else {
      leftDrive->spin(leftDirection);
    }

    rightDrive->setVelocity(rightMotorPower, percent);
    if (rightMotorPower == 0) {
      rightDrive->stop(hold);
    } else {
      rightDrive->spin(rightDirection);
    }
  }

  void driveVelocity(float motorPower) {
    driveVelocity(motorPower, motorPower);
  }

  float clampStraightVelocity(float motorVelocity) {
    if (motorVelocity > straightParameters.maxVelocity) {
      return straightParameters.maxVelocity;
    } else if (motorVelocity < -straightParameters.maxVelocity) {
      return -straightParameters.maxVelocity;
    } else {
      return motorVelocity;
    }
  }

  float clampTurnVelocity(float motorVelocity) {
    if (motorVelocity > turnParameters.maxVelocity) {
      return turnParameters.maxVelocity;
    } else if (motorVelocity < -turnParameters.maxVelocity) {
      return -turnParameters.maxVelocity;
    } else {
      return motorVelocity;
    }
  }

  float clampHeadingVelocity(float motorVelocity) {
    if (motorVelocity > headingParameters.maxVelocity) {
      return headingParameters.maxVelocity;
    } else if (motorVelocity < -headingParameters.maxVelocity) {
      return -headingParameters.maxVelocity;
    } else {
      return motorVelocity;
    }
  }

  float reduceAngleNegPiToPi(float headingAngle) { // Inspired by a pretty convenient utility function by Jackson Area Robotics
    // Not using modulo because negative inputs are implementation dependent...
    while (headingAngle > M_PI || headingAngle <= -M_PI) {
      if (headingAngle > M_PI) {
        headingAngle -= 2 * M_PI;
      } else {
        headingAngle += 2 * M_PI;
      }
    }
    return headingAngle;
  }

  float degreesToRadians(float angleDegrees) {
    return angleDegrees * M_PI / 180;
  }

  // Drivetrain autonomous functions
  void odometryStep() {
    wait(DT, msec);
    odom->pollSensorValues();
  }
  
  void driveDistance(float dist) {
    float driveSetPoint = dist + (odom->getLeftDistance() + odom->getRightDistance()) / 2;
    PID* drivePID = new PID(dist, straightParameters.kp, straightParameters.ki, straightParameters.kd, straightParameters.integralRange, straightParameters.settleThreshold, straightParameters.settleTime);
    float headingSetPoint = odom->getOrientation();
    PID* headingPID = new PID(0, headingParameters.kp, headingParameters.ki, headingParameters.kd, headingParameters.integralRange, headingParameters.settleThreshold, headingParameters.settleTime);
    while (!drivePID->isSettled()) {
      float distanceError = driveSetPoint - (odom->getLeftDistance() + odom->getRightDistance()) / 2;
      //remoteControl->Screen.clearScreen();
      //remoteControl->Screen.setCursor(1,1);
      //remoteControl->Screen.print("Time settled: %f", turnPID->timeSettled);
      //remoteControl->Screen.print("Error: %f", distanceError);
      float driveMotorVelocity = drivePID->calculateNextStep(distanceError);
      
      driveMotorVelocity = clampStraightVelocity(driveMotorVelocity);

      float headingError = reduceAngleNegPiToPi(headingSetPoint - odom->getOrientation());
      float headingMotorVelocity = headingPID->calculateNextStep(headingError);

      Brain.Screen.clearLine(2);
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("Motor velocity: %f", driveMotorVelocity);

      headingMotorVelocity = clampHeadingVelocity(headingMotorVelocity);
      
      driveVelocity(driveMotorVelocity + headingMotorVelocity, driveMotorVelocity - headingMotorVelocity);

      odometryStep();
    }
    driveVelocity(0);
  }

  // Drivetrain autonomous functions
  void turnToAngle(float targetAngle) {
    float turnSetPoint = targetAngle;
    PID* turnPID = new PID(turnSetPoint - odom->getOrientation(), turnParameters.kp, turnParameters.ki, turnParameters.kd, turnParameters.integralRange, turnParameters.settleThreshold, turnParameters.settleTime);
    while (!turnPID->isSettled()) {
      float turnError = reduceAngleNegPiToPi(turnSetPoint - odom->getOrientation());
      //remoteControl->Screen.clearScreen();
      //remoteControl->Screen.setCursor(1,1);
      //remoteControl->Screen.print("Time settled: %f", turnPID->timeSettled);
      //remoteControl->Screen.print("Error: %f", turnError);
      //remoteControl->Screen.newLine();
      //remoteControl->Screen.print("Orientation: %f", odom->getOrientation());
      float turnMotorVelocity = turnPID->calculateNextStep(turnError);

      turnMotorVelocity = clampTurnVelocity(turnMotorVelocity);
      
      driveVelocity(turnMotorVelocity, -turnMotorVelocity);

      odometryStep();
    }
    driveVelocity(0);
  }

  void turnToAngleDegrees(float targetAngleDegrees) {
    turnToAngle(degreesToRadians(targetAngleDegrees));
  }
  
  void driveArcDistance(float ang, float dist) {
    float driveSetPoint = dist + (odom->getLeftDistance() + odom->getRightDistance()) / 2;
    PID* drivePID = new PID(dist, straightParameters.kp, straightParameters.ki, straightParameters.kd, straightParameters.integralRange, straightParameters.settleThreshold, straightParameters.settleTime);
    float headingSetPoint = odom->getOrientation();
    PID* turnPID = new PID(0, turnParameters.kp, turnParameters.ki, turnParameters.kd, turnParameters.integralRange, turnParameters.settleThreshold, turnParameters.settleTime);
    while (!turnPID->isSettled()) {
      float distanceError = driveSetPoint - (odom->getLeftDistance() + odom->getRightDistance()) / 2;
      //remoteControl->Screen.clearScreen();
      //remoteControl->Screen.setCursor(1,1);
      //remoteControl->Screen.print("Time settled: %f", turnPID->timeSettled);
      //remoteControl->Screen.print("Error: %f", distanceError);
      float driveMotorVelocity = drivePID->calculateNextStep(distanceError);
      
      driveMotorVelocity = clampStraightVelocity(driveMotorVelocity);

      float turnError = reduceAngleNegPiToPi(turnSetPoint - odom->getOrientation());
      float turnMotorVelocity = turnPID->calculateNextStep(turnError);

      Brain.Screen.clearLine(2);
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("Motor velocity: %f", driveMotorVelocity);

      turnMotorVelocity = clampTurnVelocity(turnMotorVelocity);
      
      driveVelocity(driveMotorVelocity + turnMotorVelocity, driveMotorVelocity - turnMotorVelocity);

      odometryStep();
    }
    driveVelocity(0);
  }
  
  void driveArcRadius(float ang, float radius) {
    // s = r * theta
    float dist = radius * ang;

    driveArcDistance(ang, dist)
};

#pragma endregion Custom Drive Library

/* --------------- Start robot configuration --------------- */

// Drivetrain
motor LeftDriveMotorFront = motor(PORT1, ratio6_1, true);
motor LeftDriveMotorMiddle = motor(PORT2, ratio6_1, true);
motor LeftDriveMotorBack = motor(PORT3, ratio6_1, true);
motor_group LeftDrive = motor_group(LeftDriveMotorFront, LeftDriveMotorMiddle, LeftDriveMotorBack);

motor RightDriveMotorFront = motor(PORT4, ratio6_1, false);
motor RightDriveMotorMiddle = motor(PORT5, ratio6_1, false);
motor RightDriveMotorBack = motor(PORT6, ratio6_1, false);
motor_group RightDrive = motor_group(RightDriveMotorFront, RightDriveMotorMiddle, RightDriveMotorBack);

inertial InertialSensor = inertial(PORT7);

// Intake
motor IntakeRollerMotor = motor(PORT10, ratio36_1, true);
motor IntakeBeltMotor = motor(PORT9, ratio36_1, true);

// Arm
digital_out ArmPneumatic = digital_out(Brain.ThreeWirePort.F);

// MoGo Mech
digital_out LeftMoGoPneumatic = digital_out(Brain.ThreeWirePort.G);
digital_out RightMoGoPneumatic = digital_out(Brain.ThreeWirePort.H);
controller RemoteControl = controller(primary);


// Odometry
float InertialDriftEpsilon = 0.000025;
float DistLeft = 7.5;
float DistRight = 7.5;
float DistBack = 0;
float LeftWheelRadius = 1.625;
float RightWheelRadius = 1.625;
/* kp, ki, kd, integralRange, settleThreshold, settleTime, maxVelocity */
odomParameters StraightParameters = {5, 0, 0, 0, 0.25, 0.25, 80};
odomParameters TurnParameters = {26.2, 0.009, 40, M_PI / 2, 0.035, 0.2, 50}; // kU = 34, pU = 1.398
odomParameters HeadingParameters = {40, 0.020, 40, M_PI / 2, 0.035, 0.2, 0};

/* --------------- Start autons --------------- */

void odomDebugAuton(Drive* robotDrivetrain, motor &intakeBeltMotor, digital_out &armPneumatic, motor &IntakeRollerMotor, digital_out &leftMoGoPneumatic, digital_out &rightMoGoPneumatic) {
  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    coordinate globalPosition = robotDrivetrain->odom->getGlobalPosition();
    float xCoordinate = globalPosition.x;
    float yCoordinate = globalPosition.y;
    float leftRotation = robotDrivetrain->odom->getDeltaLeftAngle();
    float rightRotation = robotDrivetrain->odom->getDeltaRightAngle();
    float robotOrientation = robotDrivetrain->odom->getOrientation();
    float arcRadius = robotDrivetrain->odom->getPathArcRadius();
    Brain.Screen.print("X coordinate: %f", xCoordinate);
    Brain.Screen.newLine();
    Brain.Screen.print("Y coordinate: %f", yCoordinate);
    Brain.Screen.newLine();
    Brain.Screen.print("Left rotation: %f", leftRotation);
    Brain.Screen.newLine();
    Brain.Screen.print("Right rotation: %f", rightRotation);
    Brain.Screen.newLine();
    Brain.Screen.print("Orientation: %f", robotOrientation);
    Brain.Screen.newLine();
    Brain.Screen.print("Arc Radius: %f", arcRadius);
    Brain.Screen.newLine();
    Brain.Screen.print("X: %f", globalPosition.x);
    Brain.Screen.newLine();
    Brain.Screen.print("Y: %f", globalPosition.y);



    robotDrivetrain->odometryStep();
  }
}

void redLowAlly(Drive* robotDrivetrain, motor &intakeBeltMotor, digital_out &armPneumatic, motor &IntakeRollerMotor, digital_out &leftMoGoPneumatic, digital_out &rightMoGoPneumatic) {
  robotDrivetrain->straightParameters.maxVelocity = 35;
  robotDrivetrain->turnParameters.maxVelocity = 40;
  robotDrivetrain->driveDistance(-17.3);
  robotDrivetrain->turnToAngle(-M_PI/2);
  robotDrivetrain->driveDistance(-1.6);
  LeftMoGoPneumatic.set(true);
  RightMoGoPneumatic.set(true);
  IntakeRollerMotor.setVelocity(100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  IntakeRollerMotor.spin(forward);
  wait(1000, msec);
  intakeBeltMotor.stop();
  IntakeRollerMotor.stop();
  LeftMoGoPneumatic.set(false);
  RightMoGoPneumatic.set(false);
  robotDrivetrain->driveDistance(3);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->turnParameters.maxVelocity = 80;
  robotDrivetrain->turnToAngle(M_PI*.665);
  robotDrivetrain->driveDistance(-40);
  robotDrivetrain->straightParameters.maxVelocity = 35;
  robotDrivetrain->driveDistance(-9);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  LeftMoGoPneumatic.set(true);
  RightMoGoPneumatic.set(true);
  wait(500, msec);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->driveDistance(6.5);
  robotDrivetrain->turnToAngle(-M_PI*.08);
  intakeBeltMotor.spin(forward);
  IntakeRollerMotor.spin(forward);
  robotDrivetrain->driveDistance(37);
  wait(1500, msec);
  robotDrivetrain->turnParameters.maxVelocity = 50;
  robotDrivetrain->turnToAngle(M_PI*1.075);
}

void blueLowAlly(Drive* robotDrivetrain, motor &intakeBeltMotor, digital_out &armPneumatic, motor &IntakeRollerMotor, digital_out &leftMoGoPneumatic, digital_out &rightMoGoPneumatic) {
  robotDrivetrain->straightParameters.maxVelocity = 35;
  robotDrivetrain->turnParameters.maxVelocity = 40;
  robotDrivetrain->driveDistance(-17.3);
  robotDrivetrain->turnToAngle(M_PI/2);
  robotDrivetrain->driveDistance(-1.85);
  LeftMoGoPneumatic.set(true);
  RightMoGoPneumatic.set(true);
  IntakeRollerMotor.setVelocity(100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  IntakeRollerMotor.spin(forward);
  wait(1000, msec);
  intakeBeltMotor.stop();
  IntakeRollerMotor.stop();
  LeftMoGoPneumatic.set(false);
  RightMoGoPneumatic.set(false);
  robotDrivetrain->driveDistance(3.25);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->turnParameters.maxVelocity = 80;
  robotDrivetrain->turnToAngle(-M_PI*.68);
  robotDrivetrain->driveDistance(-37);
  robotDrivetrain->straightParameters.maxVelocity = 35;
  robotDrivetrain->driveDistance(-12);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  LeftMoGoPneumatic.set(true);
  RightMoGoPneumatic.set(true);
  wait(500, msec);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->driveDistance(6.5);
  robotDrivetrain->turnToAngle(M_PI*.08);
  intakeBeltMotor.spin(forward);
  IntakeRollerMotor.spin(forward);
  robotDrivetrain->driveDistance(37);
  wait(1500, msec);
  robotDrivetrain->turnParameters.maxVelocity = 50;
  robotDrivetrain->turnToAngle(M_PI);
}

void soloHighRed(Drive* robotDrivetrain, motor &intakeBeltMotor, digital_out &armPneumatic, motor &IntakeRollerMotor, digital_out &leftMoGoPneumatic, digital_out &rightMoGoPneumatic) {
  /* Demo functions:
     Drive forward: robotDrivetrain->driveDistance({distance});
     Turn to angle: robotDrivetrain->turnToAngle({angle});
     Delay: wait({time}, msec);
     */
  /* [Removed auton]
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->turnParameters.maxVelocity = 60;
  robotDrivetrain->driveDistance(-24);
  robotDrivetrain->straightParameters.maxVelocity = 20;
  robotDrivetrain->driveDistance(-7);
  LeftMoGoPneumatic.set(true);
  RightMoGoPneumatic.set(true);
  IntakeRollerMotor.setVelocity(100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  IntakeRollerMotor.spin(forward);
  robotDrivetrain->driveDistance(3);
  wait(1000, msec);
  robotDrivetrain->turnToAngle(-M_PI*.64);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->driveDistance(33);
  robotDrivetrain->straightParameters.maxVelocity = 17.5;
  robotDrivetrain->driveDistance(3);
  robotDrivetrain->driveDistance(-10);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->turnToAngle(M_PI*.45);
  robotDrivetrain->driveDistance(20);
  robotDrivetrain->straightParameters.maxVelocity = 20;
  robotDrivetrain->driveDistance(39);
  */
  /* [Removed auton]

  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->turnParameters.maxVelocity = 80;
  robotDrivetrain->driveDistance(-24);
  robotDrivetrain->straightParameters.maxVelocity = 20;
  robotDrivetrain->driveDistance(-7);
  LeftMoGoPneumatic.set(true);
  RightMoGoPneumatic.set(true);
  IntakeRollerMotor.setVelocity(100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  IntakeRollerMotor.spin(forward);
  wait(750, msec);
  robotDrivetrain->turnToAngle(M_PI*.76);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->driveDistance(27);
  wait(800, msec);
  robotDrivetrain->straightParameters.maxVelocity = 17.5;
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->driveDistance(-21);
  robotDrivetrain->turnToAngle(M_PI/2);
  robotDrivetrain->driveDistance(26);
  wait(200, msec);
  robotDrivetrain->straightParameters.maxVelocity = 20;
  robotDrivetrain->driveDistance(-6);
  robotDrivetrain->turnToAngle(M_PI*.855);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->driveDistance(19.5);
  wait(600, msec);
  robotDrivetrain->driveDistance(-18);
  robotDrivetrain->turnToAngle(-M_PI/2);
  robotDrivetrain->driveDistance(55);
  */
  
  robotDrivetrain->straightParameters.maxVelocity = 40;
  robotDrivetrain->turnParameters.maxVelocity = 40;
  robotDrivetrain->driveDistance(-17.75);
  robotDrivetrain->turnToAngle(M_PI/2);
  robotDrivetrain->driveDistance(-2.25);
  LeftMoGoPneumatic.set(true);
  RightMoGoPneumatic.set(true);
  IntakeRollerMotor.setVelocity(100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  IntakeRollerMotor.spin(forward);
  wait(550, msec);
  intakeBeltMotor.stop();
  IntakeRollerMotor.stop();
  LeftMoGoPneumatic.set(false);
  RightMoGoPneumatic.set(false);
  robotDrivetrain->driveDistance(3);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->turnParameters.maxVelocity = 40;
  robotDrivetrain->turnToAngle(-M_PI*.69);
  robotDrivetrain->turnParameters.maxVelocity = 10;
  robotDrivetrain->turnToAngle(-M_PI*.69);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->driveDistance(-34);
  robotDrivetrain->straightParameters.maxVelocity = 35;
  robotDrivetrain->turnParameters.maxVelocity = 50;
  robotDrivetrain->driveDistance(-12);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  LeftMoGoPneumatic.set(true);
  RightMoGoPneumatic.set(true);
  wait(100, msec);
  robotDrivetrain->turnToAngle(M_PI*.285);
  IntakeRollerMotor.setVelocity(100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  IntakeRollerMotor.spin(forward);
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->driveDistance(27);
  wait(400, msec);
  robotDrivetrain->straightParameters.maxVelocity = 30;
  robotDrivetrain->driveDistance(-22);
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->turnToAngle(0);
  robotDrivetrain->driveDistance(28);
  wait(500, msec);
  robotDrivetrain->turnToAngle(M_PI);
  robotDrivetrain->turnParameters.maxVelocity = 5;
  robotDrivetrain->turnToAngle(M_PI);
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->driveDistance(60);
  /* [bad auton]
  
  robotDrivetrain->straightParameters.maxVelocity = 40;
  robotDrivetrain->turnParameters.maxVelocity = 40;
  robotDrivetrain->driveDistance(-17.5);
  robotDrivetrain->turnToAngle(M_PI/2);
  robotDrivetrain->driveDistance(-2);
  LeftMoGoPneumatic.set(true);
  RightMoGoPneumatic.set(true);
  IntakeRollerMotor.setVelocity(100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  IntakeRollerMotor.spin(forward);
  wait(650, msec);
  intakeBeltMotor.stop();
  IntakeRollerMotor.stop();
  LeftMoGoPneumatic.set(false);
  RightMoGoPneumatic.set(false);
  robotDrivetrain->driveDistance(3.3);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->turnParameters.maxVelocity = 80;
  robotDrivetrain->turnToAngle(-M_PI*.665);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->driveDistance(-40);
  robotDrivetrain->straightParameters.maxVelocity = 35;
  robotDrivetrain->driveDistance(-9);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  LeftMoGoPneumatic.set(true);
  RightMoGoPneumatic.set(true);
  wait(100, msec);
  robotDrivetrain->turnToAngle(M_PI*.26);
  IntakeRollerMotor.setVelocity(100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  IntakeRollerMotor.spin(forward);
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->driveDistance(28);
  wait(200, msec);
  robotDrivetrain->straightParameters.maxVelocity = 17.5;
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->driveDistance(-26);
  robotDrivetrain->turnToAngle(0);
  robotDrivetrain->driveDistance(28);
  robotDrivetrain->straightParameters.maxVelocity = 30;
  robotDrivetrain->driveDistance(-7);
  robotDrivetrain->turnToAngle(M_PI*.37);
  robotDrivetrain->straightParameters.maxVelocity = 75;
  robotDrivetrain->driveDistance(19);
  wait(400, msec);
  robotDrivetrain->driveDistance(-26);
  robotDrivetrain->turnToAngle(M_PI);
  robotDrivetrain->driveDistance(54);
  */
}

void oldRed4RingSkills(Drive* robotDrivetrain, motor &intakeBeltMotor, digital_out &armPneumatic, motor &intakeRollerMotor, digital_out &leftMoGoPneumatic, digital_out &rightMoGoPneumatic) {
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->turnParameters.maxVelocity = 80;
  robotDrivetrain->driveDistance(-24);
  robotDrivetrain->straightParameters.maxVelocity = 20;
  robotDrivetrain->driveDistance(-7);
  leftMoGoPneumatic.set(true);
  rightMoGoPneumatic.set(true);
  intakeRollerMotor.setVelocity(100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  intakeRollerMotor.spin(forward);
  leftMoGoPneumatic.set(true);
  rightMoGoPneumatic.set(true);
  intakeRollerMotor.setVelocity(-100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  intakeRollerMotor.spin(forward);
  wait(750, msec);
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->turnParameters.maxVelocity = 20;
  robotDrivetrain->turnToAngle(M_PI*.772);
  intakeRollerMotor.setVelocity(-100,percent);
  intakeBeltMotor.setVelocity(100,percent);
  intakeBeltMotor.spin(forward);
  intakeRollerMotor.spin(forward);
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->driveDistance(29.75);
  wait(600, msec);
  robotDrivetrain->straightParameters.maxVelocity = 50;
  robotDrivetrain->driveDistance(-22.25);
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->turnToAngle(M_PI/2);
  robotDrivetrain->driveDistance(28);
  robotDrivetrain->turnParameters.maxVelocity = 30;
  robotDrivetrain->turnToAngle(M_PI*.98);
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->driveDistance(15.5);
   wait(600, msec);
  robotDrivetrain->driveDistance(-16);
  robotDrivetrain->turnParameters.maxVelocity = 30;
  robotDrivetrain->turnToAngle(3*M_PI/2);
  robotDrivetrain->straightParameters.maxVelocity = 80;
  robotDrivetrain->driveDistance(65);
  robotDrivetrain->driveDistance(-100);
  leftMoGoPneumatic.set(false);
  rightMoGoPneumatic.set(false);

  
}

void bigSkills(Drive* robotDrivetrain, motor &intakeBeltMotor, digital_out &armPneumatic, motor &intakeRollerMotor, digital_out &leftMoGoPneumatic, digital_out &rightMoGoPneumatic) {
  robotDrivetrain->odom->resetOrientation(); // Start using this in autons to set the starting angle, this will just make the starting angle 0
  intakeBeltMotor.setVelocity(100, percent);
  intakeRollerMotor.setVelocity(100, percent);

  intakeBeltMotor.spin(forward);
  wait(750, msec);
  intakeBeltMotor.stop();

  robotDrivetrain->driveDistance(14);
  robotDrivetrain->turnToAngle(-M_PI/4);
  intakeRollerMotor.spin(reverse);
  robotDrivetrain->driveDistance(50);
  wait(300, msec);
  robotDrivetrain->turnParameters.maxVelocity = 30;
  robotDrivetrain->turnToAngle(0.004*M_PI);
  robotDrivetrain->turnParameters.maxVelocity = 50;
  wait(300, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.print("Orientation: %f", robotDrivetrain->odom->getOrientation());
  robotDrivetrain->driveDistance(-15);
  robotDrivetrain->straightParameters.maxVelocity = 20;
  robotDrivetrain->driveDistance(-7);
  robotDrivetrain->straightParameters.maxVelocity = 80;
  leftMoGoPneumatic.set(true);
  rightMoGoPneumatic.set(true);
  intakeBeltMotor.spin(forward);
  Brain.Screen.newLine();
  Brain.Screen.print("Orientation: %f", robotDrivetrain->odom->getOrientation());
  
}

void straightDebug(Drive* robotDrivetrain, motor &intakeBeltMotor, digital_out &armPneumatic, motor &intakeRollerMotor, digital_out &leftMoGoPneumatic, digital_out &rightMoGoPneumatic) {
  robotDrivetrain->odom->resetOrientation(); // Start using this in autons to set the starting angle, this will just make the starting angle 0

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Orientation: %f", robotDrivetrain->odom->getOrientation());
  robotDrivetrain->driveDistance(30);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("Orientation: %f", robotDrivetrain->odom->getOrientation());

  wait(500, msec);
  robotDrivetrain->leftDrive->stop(coast);
  robotDrivetrain->rightDrive->stop(coast);
}

/* --------------- Start driver control ---------------*/

void driverControl(Drive* robotDrivetrain, motor &intakeBeltMotor, digital_out &armPneumatic, motor &intakeRollerMotor, digital_out &leftMoGoPneumatic, digital_out &rightMoGoPneumatic) {
  robotDrivetrain->leftDrive->stop(coast);
  robotDrivetrain->rightDrive->stop(coast);
  while (true) {
    robotDrivetrain->driverControl();

    // Intake
    bool r1 = robotDrivetrain->remoteControl->ButtonR1.pressing();
    bool r2 = robotDrivetrain->remoteControl->ButtonR2.pressing();
    
    int intakeSpinDirection = r2 - r1;

    intakeBeltMotor.setVelocity(100 * (float) intakeSpinDirection, percent);
    intakeBeltMotor.spin(forward);
  

    
    bool x = robotDrivetrain->remoteControl->ButtonX.pressing();
    bool b = robotDrivetrain->remoteControl->ButtonB.pressing();
    
    
    int intakeState = x - b;

    if (intakeState == 1) {
      armPneumatic.set(true);
    } else if (intakeState == -1) {
      armPneumatic.set(false);
    }

    // Arm
    bool bDown = robotDrivetrain->remoteControl->ButtonR1.pressing();
    bool bUp = robotDrivetrain->remoteControl->ButtonR2.pressing();
    
    int armSpinDirection = bUp - bDown;

    intakeRollerMotor.setVelocity(100 * (float) armSpinDirection, percent);
    intakeRollerMotor.spin(reverse);

    // MoGo Mech
    bool l1 = robotDrivetrain->remoteControl->ButtonL1.pressing();
    bool l2 = robotDrivetrain->remoteControl->ButtonL2.pressing();
    
    int mogoState = l2 - l1;

    if (mogoState == 1) {
      leftMoGoPneumatic.set(true);
      rightMoGoPneumatic.set(true);
    } else if (mogoState == -1) {
      leftMoGoPneumatic.set(false);
      rightMoGoPneumatic.set(false);
    }

    wait(DT, msec);
  }
}

/* --------------- Start auton selector --------------- */



void autonSelector() {

}

/* --------------- Start main program --------------- */

void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  InertialSensor.calibrate();
  wait(3, seconds);
}

void templateAutonomous(void) { // Dummy wrapper function to call the desired autonomous (because the competition template can't take parameters)
  Drive* robotDrivetrain = new Drive(LeftDrive, RightDrive, forward, forward, InertialSensor, RemoteControl);
  robotDrivetrain->initOdom(InertialDriftEpsilon, DistLeft, DistRight, DistBack, LeftWheelRadius, RightWheelRadius, StraightParameters, TurnParameters, HeadingParameters);

  red4RingSkills(robotDrivetrain, IntakeBeltMotor, ArmPneumatic, IntakeRollerMotor, LeftMoGoPneumatic, RightMoGoPneumatic);
}

void templateDriverControl(void) { // Dummy wrapper function to call the desired driver control (because the competition template can't take parameters)
  Drive* robotDrivetrain = new Drive(LeftDrive, RightDrive, forward, forward, InertialSensor, RemoteControl);
  robotDrivetrain->initOdom(InertialDriftEpsilon, DistLeft, DistRight, DistBack, LeftWheelRadius, RightWheelRadius, StraightParameters, TurnParameters, HeadingParameters);

  driverControl(robotDrivetrain, IntakeBeltMotor, ArmPneumatic, IntakeRollerMotor, LeftMoGoPneumatic, RightMoGoPneumatic);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // create competition instance
  competition Competition;

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(templateAutonomous);
  Competition.drivercontrol(templateDriverControl);

  // Run the pre-autonomous function.
  preAutonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}