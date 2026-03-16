/*GIVEN LESS RAM LIMITATIONS AKA A GEN 2 BRAIN
Features worth adding: 
 - Objects instead of ints to the array
  (you could store how many times an object is seen at a location, rather than overwriting every time, which would act as a confidence val)
 - Tighter precision levels
  (Ideal world, 5cm precision, 8.6 feels like it's pushing our luck a bit when it's roughly the size of the pins themselves)
*/
#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "iq_cpp.h"
#include "iq_apitypes.h"
#include "iq_vcs.h"
#include "vex_brain.h"
#include "vex_bumper.h"
#include "vex_colorsensor.h"
#include "vex_console.h"
#include "vex_controller.h"
#include "vex_device.h"
#include "vex_distance.h"
#include "vex_drivetrain.h"
#include "vex_event.h"
#include "vex_generic.h"
#include "vex_global.h"
#include "vex_gyro.h"
#include "vex_motor.h"
#include "vex_motorgroup.h"
#include "vex_optical.h"
#include "vex_pneumatic.h"
#include "vex_smartdrive.h"
#include "vex_sonar.h"
#include "vex_task.h"
#include "vex_task.h"
#include "vex_thread.h"
#include "vex_timer.h"
#include "vex_touchled.h"
#include "vex_units.h"
#include "vex_vision.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START IQ MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
motor LeftDriveSmart = motor(PORT1, 1, false);
motor RightDriveSmart = motor(PORT6, 1, true);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 200, 173, 76, mm, 1);
gyro MainGyro = gyro(PORT4);
motor Arm = motor(PORT10, false);
bumper Bumper8 = bumper(PORT8);
motor Claw = motor(PORT11, false);
touchled TouchLED2 = touchled(PORT2);
sonar RearDistance = sonar(PORT7);
sonar RightDistance = sonar(PORT5);
sonar LeftDistance = sonar(PORT9);
sonar FrontDistance = sonar(PORT12);
colorsensor Color3 = colorsensor(PORT3);


// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double BrainTime = Brain.Timer.system() * 1000;
  double BrainVoltage = Brain.Battery.voltage();
  // Combine these values into a single integer
  int seed = int(
    BrainTime + BrainVoltage
  );
  // Set the seed
  srand(seed); 
}


// Convert colorType to string
const char* convertColorToString(colorType col) {
  if (col == colorType::red) return "red";
  else if (col == colorType::green) return "green";
  else if (col == colorType::blue) return "blue";
  else if (col == colorType::white) return "white";
  else if (col == colorType::yellow) return "yellow";
  else if (col == colorType::orange) return "orange";
  else if (col == colorType::purple) return "purple";
  else if (col == colorType::cyan) return "cyan";
  else if (col == colorType::black) return "black";
  else if (col == colorType::transparent) return "transparent";
  else if (col == colorType::red_violet) return "red_violet";
  else if (col == colorType::violet) return "violet";
  else if (col == colorType::blue_violet) return "blue_violet";
  else if (col == colorType::blue_green) return "blue_green";
  else if (col == colorType::yellow_green) return "yellow_green";
  else if (col == colorType::yellow_orange) return "yellow_orange";
  else if (col == colorType::red_orange) return "red_orange";
  else if (col == colorType::none) return "none";
  else return "unknown";
}


void vexcodeInit() {

  // Initializing random seed.
  initializeRandomSeed(); 
}

#pragma endregion VEXcode Generated Robot Configuration

// Include the IQ Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;

float forwardDistance, rearDistance, leftDistance, rightDistance; //Distance read by each sensor, stored in millimeters
float curHeading; //The heading read 
double x, y; //Max X,Y 1828.8mm, 2438.4mm

//Key vals, all stored in mm
const float maxX = 1828.8;
const float maxY = 2438.4;
const int robotWidth = 155; //Stored in mm, may need to be accounted for in sensor readouts... not using yet but still
const int precisionLevel = 86; // /10 mm -> cm, /5 1cm to per 5cm, /50 total, still stored in mm
const float radConv = 0.0174533; //Constant for converting stored degrees to radians for math stuff
int fieldGrid[(int)((maxX/precisionLevel)+.5)][(int)((maxY/precisionLevel)+.5)] = {}; //Accurate to every 50 millimeters of precision, rounded to nearest whole number
const float acceptableError = 50; //Amount of error, in millimeters, that we're willing to accept in the sensor measurement
// Rated Error rate is 15mm @ 200mm, 5% +- above that, for the lidar... Gen 1 has no official rating I can find in 20 seconds of research
// So my assumption is that it's considerably worse

const int confWall = 2;
const int unknownObst = 1;
const int robotPos = 0;

//Timing Vars
float pollingDelay;
int calibrateTime; //May be a dead var

///Function to scan the area around the Robot in a specific direction, and map what is seen from that sensor
///All scanned info from this will be saved to the nearest field grid for future processing
void scan(float distance, int direction) {
  //direction:
  /*
   - 0 = Forward
   - 1 = Right
   - 2 = Down
   - 3 = Left
  */
  int relativeTheta;

  switch(direction) { //This is necessary, cuz all of these sensors are looking different ways. If you look right, you need to know you're looking right
    case 0: //Front sensor
    relativeTheta = MainGyro.heading();
    break;

    case 1: //Right sensor
    relativeTheta = MainGyro.heading()+90; //Since sensor is on the right side, the angle needs to be calibrated
    break;

    case 2: //Rear Sensor
    relativeTheta = MainGyro.heading()+180; //Bottom sensor, angle calibrated
    break;

    case 3: //Left Sesnor 
    relativeTheta = MainGyro.heading()+270; //Left sensor, angle calibrated
    break;
  }

  relativeTheta %= 360;
  //Calculating the relative distances of a scanned object
  float relativeX = sin(relativeTheta*radConv)*distance;
  float relativeY = cos(relativeTheta*radConv)*distance;

  float newObjX = x+relativeX;
  float newObjY = y+relativeY;

  //Not sure if I'm going to use these or not, this was AI suggested, AI is stinky
  //But I ran it by it to verify things, and it is saying my direction confirmations are flawed for wall detection
  //Apparently, using these vals and just comparing to 0 would solve this, but I want to at least test this mess before 100%
  //Determining that is the case, cause in my head, this all still makes sense
  float dirX = sin(relativeTheta * radConv);
  float dirY = cos(relativeTheta * radConv);

  //Checks if we're looking at a wall, if we are, we can use that wall to reposition ourselves in the grid, and recalibrate everything nicey nice
  //Should add function to recalibrate the gyro at some point here
  //Smth along the lines of IF we can see 2 walls, then we can use trig to perfectly calculate the current heading to deal with drift
  if(maxX+acceptableError>newObjX && newObjX>maxX-acceptableError) { //If within error rate of being a wall on the +X axis, we can assume the scanned object is in fact, a wall
    fieldGrid[(int)(newObjX/precisionLevel+.5)][(int)(newObjY/precisionLevel+.5)] = confWall;
    //If we know we're looking at a wall, then we can trust the relativeX to represent our actual X coordinate
    if(relativeTheta >= 0 && relativeTheta <= 180) { //Should verify facing right, and if we are, updates X val accurately
      x = maxX-relativeX; //If we know there's a wall on the X axis, we can assume the relativeX is accurate as a readout of the opposite wall to our axis
    } else { //Shouldn't be possible to reach here, since we're explicitly checkign if we're looking at the right wall before reaching this check
      x = relativeX; //Else, we're just facing the negative direction, and relativeX just is our X coordinates
    }
  } else if(acceptableError>newObjX && newObjX>0-acceptableError) { //If within error rate of being a wall on the -X axis
    fieldGrid[(int)(newObjX/precisionLevel+.5)][(int)(newObjY/precisionLevel+.5)] = confWall;
    if(relativeTheta <= 360 && relativeTheta >= 180) { //Same as previous case
      x = relativeX;
    } else { //Same as previous case
      x = maxX-relativeX; 
    }
  } else if(maxY+acceptableError>newObjY && newObjY>maxY-acceptableError) { //If within error rate of being a wall on the +Y axis
    fieldGrid[(int)(newObjX/precisionLevel+.5)][(int)(newObjY/precisionLevel+.5)] = confWall;
    if(relativeTheta <= 360 && relativeTheta >= 270 || relativeTheta >= 0 && relativeTheta <= 90) { //Same as previous case
      y = maxY-relativeY;
    } else { //Same as previous case
      y = relativeY;
    }
  } else if(acceptableError>newObjY && newObjY>0-acceptableError) { //If all wall checks fail, it's gotta be a game piece, but we don't know what piece, nor do we care, so mark it as unknown
    fieldGrid[(int)(newObjX/precisionLevel+.5)][(int)(newObjY/precisionLevel+.5)] = confWall;
    if(relativeTheta <= 270 && relativeTheta >= 90) { //Same as previous cases
      y = relativeY;
    } else {
      y = maxY-relativeY;
    }
  } else { //If it's not any of the walls, then we can assume that we're looking at a game object in the field
    fieldGrid[(int)(newObjX/precisionLevel+.5)][(int)(newObjY/precisionLevel+.5)] = unknownObst;
  }
}
void scanAll(float frontDist, float rearDist, float leftDist, float rightDist) {
  if(frontDist <= 1000) { //Smth seen in front
    scan(forwardDistance, 0);
  }
  if(leftDist <= 1000) { //Smth seen to left
    scan(leftDistance, 3);
  }
  if(rightDist <= 1000) { //Smth seen to right
    scan(rightDistance, 1);
  }
  if(rearDist <= 1000) { //Smth seen behind
    scan(rearDistance, 2);
  }
}
void driveTo(float newX, float newY) { //Drive directly to given coordinates, ignoring walls, objects, etc
  int newAngle = (atan2(newY-y, newX-x))/radConv;

  // if(newAngle >= 0 && newAngle < 90) {

  // } else if(newAngle >= 90 && newAngle < 180) {

  // } else if(newAngle >= 180 && newAngle < 270) {

  // } else if(newAngle >=270 && newAngle < 360) {

  // }
  if(abs(MainGyro.heading()-newAngle) > abs(360-newAngle+MainGyro.heading())) {
    Drivetrain.turn(right);
  } else {
    Drivetrain.turn(left);
  }
  while (MainGyro.heading() != newAngle) {
    //Meant to wait until we've reached our target heading
  }
  //Next step, GO 
  Drivetrain.drive(forward);
  //While driving forwards, continue to scan area and attempt to keep track of where I am in the field until arriving to destination
  while(!(x <= newX + acceptableError) && (x >= newX-acceptableError) && ((y <= newY+acceptableError) && y >= newY-acceptableError)) { //Realistically could probably ditch Y, field is taller than wide, and there's a better chance of X readout being accurate
    scanAll(FrontDistance.distance(mm), RearDistance.distance(mm), LeftDistance.distance(mm), RightDistance.distance(mm));
  }
  Drivetrain.stop();
}


/// When run, this code will poll the sensors at a preset rate, constantly refreshing
int whenStarted1() {

  //Primary run loop
  while(true) {
    //Reading Sensor Data
    wait(pollingDelay, seconds);
    forwardDistance = FrontDistance.distance(mm);
    rearDistance = RearDistance.distance(mm);
    leftDistance = LeftDistance.distance(mm);
    rightDistance = RightDistance.distance(mm);
    curHeading = MainGyro.heading();

    //Heading cases: (Version 1, going to attempt another method)
    // if(MainGyro.heading() == 0 || MainGyro.heading() == 360) {
    //   //Current sensor readout is flawed, doesn't account for pins)
    //   //Assumes the field is a perfect square with nothing else in it
    //   //SOLUTION: Rather than making that assumption, map everything to a grid in an array, then save where the robot goes in that grid?
    //   // x = (leftDistance + (maxX-leftDistance-robotWidth))/2 // Averaging both sensors data for a more accurate reading
    //   // y =

    //   MainGyro.calibrate(calNormal);
    // } else if(MainGyro.heading() >= 1 && MainGyro.heading() <= 89) {

    // }

    //Version 2, Real time mapping
    //If smth is seen out of sensor X:
    /*
     - Check heading 
     - 
    */

    scanAll(forwardDistance, rearDistance, leftDistance, rightDistance);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("CurX: %f", x);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("CurY: %f", y);
  }

  return 0;
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //Variable initialization:
  pollingDelay = .1; 
  calibrateTime = 2;

  Brain.Screen.print("Loading Gyro...");
  //Gyro Setup & Position Calibration
  MainGyro.calibrate(calExtended);
  MainGyro.setHeading(270, degrees); //Robot starts facing west, this needs to be set to 270 to be accurate on start
  x = FrontDistance.distance(mm);
  y = LeftDistance.distance(mm);
  //To adjust speed
  Drivetrain.setDriveVelocity(50, percent);
  Drivetrain.setTurnVelocity(50, percent);

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Gyro Calibrated! Ready to drive");
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("CurX: %f", x);
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("CurY: %f", y);

  whenStarted1();
}

