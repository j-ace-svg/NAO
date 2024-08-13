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
#include <vector>
  
// Allows for easier use of the VEX Library
using namespace vex;

/* --------------- Start drive library --------------- */

#pragma region Custom Drive Library

class Drive {
  public:
    motor_group* leftDrive;
    motor_group* rightDrive;
    directionType leftDirection = forward;
    directionType rightDirection = reverse;
    inertial* gyroscope;
    
    controller* remoteControl;

  Drive(motor_group &_leftDrive, motor_group &_rightDrive, inertial &_gyroscope, controller &_remoteControl) {
    leftDrive = &_leftDrive;
    rightDrive = &_rightDrive;
    gyroscope = &_gyroscope;
    remoteControl = &_remoteControl;
  }

  Drive(motor_group &_leftDrive, motor_group &_rightDrive, directionType _leftDirection, directionType _rightDirection, inertial &_gyroscope, controller &_remoteControl) {
    leftDrive = &_leftDrive;
    rightDrive = &_rightDrive;
    leftDirection = _leftDirection;
    rightDirection = _rightDirection;
    gyroscope = &_gyroscope;
    remoteControl = &_remoteControl;
  }

  void driverControl() {
    leftDrive->setVelocity(remoteControl->Axis3.position(), percent);
    if (abs(remoteControl->Axis3.position()) > 3) {
      leftDrive->spin(forward);
    }
    rightDrive->setVelocity(remoteControl->Axis2.position(), percent);
    if (abs(remoteControl->Axis2.position()) > 3) {
      rightDrive->spin(reverse);
    }
  }

};

struct coordinate {
  float x;
  float y;
};

class Odometry {
  private:
    float resetOrientGlobal; // Global orientation at last reset
    float oldLeftAngle;
    float oldRightAngle;
    float oldOrientGlobal;
    float leftAngle;
    float rightAngle;
    float orientGlobal;

  public:
    motor_group leftDrive;
    motor_group rightDrive;
    inertial gyroscope;
    float distLeft; // Distance from tracking center to left tracking wheel
    float distRight; // Distance from tracking center to right tracking wheel
    float distBack; // Distance from tracking center to back tracking wheel
    coordinate globalPositionPrev; // Previous global position vector

    void pollSensorValues() {
      oldLeftAngle = leftAngle;
      oldRightAngle = rightAngle;
      oldOrientGlobal = orientGlobal;
      leftAngle = leftDrive.position(turns) * 2 * M_PI;
      rightAngle = rightDrive.position(turns) * 2 * M_PI;
      orientGlobal = gyroscope.rotation(turns) * 2 * M_PI;
    }

    // Getter methods

    float getLeftAngle() {
      return leftAngle;
    }

    float getRightAngle() {
      return rightAngle;
    }

    float getOrientation() {
      return orientGlobal;
    }

    float getDeltaLeftAngle() {
      return leftAngle - oldLeftAngle;
    }

    float getDeltaRightAngle() {
      return rightAngle - oldRightAngle;
    }

    float getDeltaOrientation() {
      return orientGlobal - oldOrientGlobal;
    }

    float getPredictedDeltaOrientation() {
      float leftRightAngleDifference = getDeltaLeftAngle() - getDeltaRightAngle();
      float drivetrainWidth = distLeft + distRight;
      return leftRightAngleDifference / drivetrainWidth;
    }

    /*float getPathArcRadius() {

    }*/
};

#pragma endregion Custom Drive Library

/* --------------- Start robot configuration --------------- */

motor LeftDriveMotorFront = motor(PORT1, ratio18_1, false);
motor LeftDriveMotorMiddle = motor(PORT2, ratio18_1, false);
motor LeftDriveMotorBack = motor(PORT3, ratio18_1, false);
motor_group LeftDrive = motor_group(LeftDriveMotorFront, LeftDriveMotorMiddle, LeftDriveMotorBack);

motor RightDriveMotorFront = motor(PORT1, ratio18_1, true);
motor RightDriveMotorMiddle = motor(PORT2, ratio18_1, true);
motor RightDriveMotorBack = motor(PORT3, ratio18_1, true);
motor_group RightDrive = motor_group(RightDriveMotorFront, RightDriveMotorMiddle, RightDriveMotorBack);

inertial Gyroscope = inertial(PORT1);

controller RemoteControl = controller(primary);

/* --------------- Start main program --------------- */

int main() {
  Drive robotDrivetrain = new Drive(LeftDrive, RightDrive, forward, forward, Gyroscope, RemoteControl);

  while (true) {
    robotDrivetrain.driverControl();
    wait(10, msec);
  }
}
