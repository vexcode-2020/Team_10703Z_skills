using namespace vex;

extern brain Brain;
extern controller Controller;
extern motor DriveMotorLeftFront;
extern motor DriveMotorLeftBack;
extern motor DriveMotorRightFront;
extern motor DriveMotorRightBack;
extern motor ConveyorMotor;
extern motor SortingMotor;
extern motor IntakeMotorLeft;
extern motor IntakeMotorRight;

extern pot LeftIntakePot;
extern pot RightIntakePot;
extern line LeftLineTracker;
extern line RightLineTracker;
extern inertial Inertial;
extern optical OpticalPos1;
extern optical OpticalPos2;
extern distance DistancePos0;
extern distance DistancePos1;
extern distance DistancePos2;
extern distance DistanceScoring;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
