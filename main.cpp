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
using std::array
  
// Allows for easier use of the VEX Library
using namespace vex;

/* --------------- Start drive library --------------- */

#pragma region Custom Drive Library

#define INF std::numeric_limits<float>::infinity()

class Drive {
  public:
    motor_group* leftDrive;
    motor_group* rightDrive;
    directionType leftDirection = forward;
    directionType rightDirection = reverse;
    inertial* inertialSensor;
    
    controller* remoteControl;

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

};

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
    float oldTimestamp;
    float oldLeftAngle;
    float oldRightAngle;
    float oldOrientGlobal;
    coordinate oldGlobalPosition; // Previous global position vector
    float timestamp;
    float leftAngle;
    float rightAngle;
    float orientGlobal;
    coordinate globalPosition;
    float slippingEpsilon = 0.01;

  public:
    motor_group leftDrive;
    motor_group rightDrive;
    inertial inertialSensor;
    float distLeft; // Distance from tracking center to left tracking wheel
    float distRight; // Distance from tracking center to right tracking wheel
    float distBack; // Distance from tracking center to back tracking wheel
    float leftWheelRadius;
    float rightWheelRadius;

    void initSensorValues() {
      timestamp = Brain.Timer.time(seconds);
      leftAngle = leftDrive.position(turns) * 2 * M_PI;
      rightAngle = rightDrive.position(turns) * 2 * M_PI;
      orientGlobal = inertialSensor.rotation(turns) * 2 * M_PI;
      oldTimestamp = timestamp;
      oldLeftAngle = leftAngle;
      oldRightAngle = rightAngle;
      oldOrientGlobal = orientGlobal;
      resetOrientGlobal = orientGlobal;
    }

    void pollSensorValues() {
      oldTimestamp = timestamp;
      oldLeftAngle = leftAngle;
      oldRightAngle = rightAngle;
      oldOrientGlobal = orientGlobal;
      timestamp = Brain.Timer.time(seconds);
      leftAngle = leftDrive.position(turns) * 2 * M_PI;
      rightAngle = rightDrive.position(turns) * 2 * M_PI;
      orientGlobal = inertialSensor.rotation(turns) * 2 * M_PI;

      // Calculate odometry
      oldGlobalPosition = globalPosition;
      globalPosition = globalPosition + getGlobalPositionChange();
    }

    // Getter methods

    float getLeftAngle() {
      return leftAngle;
    }

    float getRightAngle() {
      return rightAngle;
    }

    float getOrientation() {
      return orientGlobal - resetOrientGlobal;
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
      
      float inertialAcceleration = inertialSensor.acceleration(yaxis);

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
      return globalChange;
    }
};

#pragma endregion Custom Drive Library

/* --------------- Start robot configuration --------------- */

motor LeftDriveMotorFront = motor(PORT1, ratio6_1, false);
motor LeftDriveMotorMiddle = motor(PORT11, ratio6_1, false);
motor LeftDriveMotorBack = motor(PORT12, ratio6_1, false);
motor_group LeftDrive = motor_group(LeftDriveMotorFront, LeftDriveMotorMiddle, LeftDriveMotorBack);

motor RightDriveMotorFront = motor(PORT10, ratio6_1, true);
motor RightDriveMotorMiddle = motor(PORT19, ratio6_1, true);
motor RightDriveMotorBack = motor(PORT20, ratio6_1, true);
motor_group RightDrive = motor_group(RightDriveMotorFront, RightDriveMotorMiddle, RightDriveMotorBack);

inertial InertialSensor = inertial(PORT5);

controller RemoteControl = controller(primary);

/* --------------- Start main program --------------- */



int main() {
  Drive * robotDrivetrain = new Drive(LeftDrive, RightDrive, forward, forward, InertialSensor, RemoteControl);

  while (true) {
    robotDrivetrain->driverControl();

    wait(10, msec);
  }
}
