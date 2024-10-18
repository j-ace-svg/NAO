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
      oldGlobalPosition = globalPosition;
      globalPosition = globalPosition + getGlobalPositionChange();
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
  private:
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

  public:

    PID(float startError, float _kp, float _ki, float _kd, float _integralRange) {
      kp = _kp;
      ki = _ki;
      kd = _kd;
      integralRange = _integralRange;
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

  void initOdom(float inertialDriftEpsilon, float distLeft, float distRight, float distBack, float leftWheelRadius, float rightWheelRadius, odomParameters _straightParameters, odomParameters _turnParameters) {
    odom = new Odometry(*leftDrive, *rightDrive, *inertialSensor, inertialDriftEpsilon, distLeft, distRight, distBack, leftWheelRadius, rightWheelRadius);
    odom->initSensorValues();
    straightParameters = _straightParameters;
    turnParameters = _turnParameters;
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
    leftDrive->spin(leftDirection);

    rightDrive->setVelocity(rightMotorPower, percent);
    rightDrive->spin(rightDirection);
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

  float reduceHeadingNegPiToPi(float headingAngle) { // Inspired by a pretty convenient utility function by Jackson Area Robotics
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

  // Drivetrain autonomous functions
  void driveDistance(float dist) {
    float driveSetPoint = dist + (odom->getLeftDistance() + odom->getRightDistance()) / 2;
    PID* drivePID = new PID(dist, straightParameters.kp, straightParameters.ki, straightParameters.kd, straightParameters.integralRange);
    float headingSetPoint = odom->getOrientation();
    PID* headingPID = new PID(0, headingParameters.kp, headingParameters.ki, headingParameters.kd, headingParameters.integralRange);
    while (!drivePID->isSettled()) {
      float distanceError = driveSetPoint - (odom->getLeftDistance() + odom->getRightDistance()) / 2;
      float driveMotorVelocity = drivePID->calculateNextStep(distanceError);
      
      driveMotorVelocity = clampStraightVelocity(driveMotorVelocity);

      float headingError = reduceHeadingNegPiToPi(headingSetPoint - odom->getOrientation());
      float headingMotorVelocity = headingPID->calculateNextStep(headingError);

      headingMotorVelocity = clampHeadingVelocity(headingMotorVelocity);
      
      driveVelocity(driveMotorVelocity + headingMotorVelocity, driveMotorVelocity - headingMotorVelocity);

      wait(DT, msec);
    }
  }

  // Drivetrain autonomous functions
  void turnToAngle(float targetAngle) {
    float headingSetPoint = targetAngle;
    PID* headingPID = new PID(headingSetPoint - odom->getOrientation, headingParameters.kp, headingParameters.ki, headingParameters.kd, headingParameters.integralRange);
    while (!headingPID->isSettled()) {
      float headingError = reduceHeadingNegPiToPi(headingSetPoint - odom->getOrientation());
      float headingMotorVelocity = headingPID->calculateNextStep(headingError);

      headingMotorVelocity = clampHeadingVelocity(headingMotorVelocity);
      
      driveVelocity(headingMotorVelocity, headingMotorVelocity);

      wait(DT, msec);
    }
  }

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
motor IntakeMotor = motor(PORT10, ratio36_1, false);
digital_out IntakePneumatic = digital_out(Brain.ThreeWirePort.F);

// Arm
motor ArmMotor = motor(PORT9, ratio36_1, false);

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
odomParameters StraightParameters = {0, 0, 0, 0, 0, 0, 0};
odomParameters TurnParameters = {0, 0, 0, 0, 0, 0, 0};

/* --------------- Start autons --------------- */

void odomDebugAuton(Drive* robotDrivetrain, motor &intakeMotor, digital_out &intakePneumatic, motor &armMotor, digital_out &leftMoGoPneumatic, digital_out &rightMoGoPneumatic) {
  while (true) {
    robotDrivetrain->odom->pollSensorValues();
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    coordinate globalPosition = robotDrivetrain->odom->getGlobalPosition();
    float xCoordinate = globalPosition.x;
    float yCoordinate = globalPosition.y;
    float leftRotation = robotDrivetrain->odom->getDeltaLeftAngle();
    float rightRotation = robotDrivetrain->odom->getDeltaRightAngle();
    float robotOrientation = robotDrivetrain->odom->getOrientation();
    float arcRadius = robotDrivetrain->odom->getPathArcRadius();
    Brain.Screen.print("X coordinate: %f\n", xCoordinate);
    Brain.Screen.newLine();
    Brain.Screen.print("Y coordinate: %f\n", yCoordinate);
    Brain.Screen.newLine();
    Brain.Screen.print("Left rotation: %f\n", leftRotation);
    Brain.Screen.newLine();
    Brain.Screen.print("Right rotation: %f\n", rightRotation);
    Brain.Screen.newLine();
    Brain.Screen.print("Orientation: %f\n", robotOrientation);
    Brain.Screen.newLine();
    Brain.Screen.print("Arc Radius: %f\n", arcRadius);
    Brain.Screen.newLine();
    Brain.Screen.print("X: %f\n", globalPosition.x);
    Brain.Screen.newLine();
    Brain.Screen.print("Y: %f\n", globalPosition.y);



    wait(DT, msec);
  }
}

/* --------------- Start driver control ---------------*/

void driverControl(Drive* robotDrivetrain, motor &intakeMotor, digital_out &intakePneumatic, motor &armMotor, digital_out &leftMoGoPneumatic, digital_out &rightMoGoPneumatic) {
  while (true) {
    robotDrivetrain->driverControl();

    // Intake
    bool r1 = robotDrivetrain->remoteControl->ButtonR1.pressing();
    bool r2 = robotDrivetrain->remoteControl->ButtonR2.pressing();
    
    int intakeSpinDirection = r2 - r1;

    intakeMotor.setVelocity(100 * (float) intakeSpinDirection, percent);
    intakeMotor.spin(forward);

    
    bool bX = robotDrivetrain->remoteControl->ButtonX.pressing();
    bool bB = robotDrivetrain->remoteControl->ButtonB.pressing();
    
    /*
    int intakeState = bX - bB;

    if (intakeState == 1) {
      intakePneumatic.set(true);
    } else if (intakeState == -1) {
      intakePneumatic.set(false);
    }*/

    // Arm
    bool bDown = robotDrivetrain->remoteControl->ButtonDown.pressing();
    bool bUp = robotDrivetrain->remoteControl->ButtonUp.pressing();
    
    int armSpinDirection = bUp - bDown;

    armMotor.setVelocity(100 * (float) armSpinDirection, percent);
    armMotor.spin(forward);

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

/* --------------- Start main program --------------- */



int main() {
  Drive* robotDrivetrain = new Drive(LeftDrive, RightDrive, forward, forward, InertialSensor, RemoteControl);
  robotDrivetrain->initOdom(InertialDriftEpsilon, DistLeft, DistRight, DistBack, LeftWheelRadius, RightWheelRadius, StraightParameters, TurnParameters);

  odomDebugAuton(robotDrivetrain, IntakeMotor, IntakePneumatic, ArmMotor, LeftMoGoPneumatic, RightMoGoPneumatic);
}