#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

//VEXcode Devices
controller Controller = controller(primary);

// motors
motor DriveMotorLeftFront (PORT11, ratio18_1,false);
motor DriveMotorLeftBack (PORT13, ratio18_1,false);
motor DriveMotorRightFront (PORT19, ratio18_1,true);
motor DriveMotorRightBack (PORT8, ratio18_1,true);
motor ConveyorMotor (PORT12, ratio6_1,true);
motor SortingMotor (PORT6, ratio6_1,true);
motor IntakeMotorLeft (PORT15, ratio18_1,false);
motor IntakeMotorRight (PORT10, ratio18_1,true);

// sensors
pot LeftIntakePot = pot(Brain.ThreeWirePort.B);
pot RightIntakePot = pot(Brain.ThreeWirePort.A);
line LeftLineTracker = line(Brain.ThreeWirePort.G);
line RightLineTracker = line(Brain.ThreeWirePort.H);
inertial Inertial = inertial(PORT16);
optical OpticalPos1 = optical(PORT17);
optical OpticalPos2 = optical(PORT1);
distance DistancePos0 = distance(PORT2);
distance DistancePos1 = distance(PORT18);
distance DistancePos2 = distance(PORT3);
distance DistanceScoring = distance(PORT5);


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}