// Improves on V1 by including a stewart platform builder, and hence allows control by inputting the 6 DOF of the SP, rather than
// just the leg lengths directly.

#include <Wire.h>
#include <AccelStepper.h>
#include <ams_as5048b.h>

// ###################################################### //
//                    Define Constants                    //
// ###################################################### //

// Initialise motor pins. Order is: motor N step, motor N dir, motor N+1 step etc.
byte motorPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; 
AccelStepper *motorArray[6] = {NULL, NULL, NULL, NULL, NULL, NULL};

// Motor setup stuff
int motMaxSpeed = 14000;      // Hard limit for how fast the motor will go (steps/s). Change with caution.
int motAccel = 3000;           // Max acceleration (steps/s^2) (use 3000)
float motMinPulseWidth = 2.5; // Min pulse width to excite motors {us}

// Measured Geometry for SP
float initialHeight = 312;        // Distance from the base plane (bottom joint of legs) to the platform in its lowest position {mm}
float baseRad = 247.8;            // Radius of base {mm}
float platRad = 229.5;            // Radius of platform {mm}
float baseSmallAngle = 24;        // Gamma2 on main sketch {deg}
float platSmallAngle = 7.494967;  // Lambda1 on main sketch {deg}
float startPos[3] = {0, 0, initialHeight}; // Starting position of platform in base frame {mm}

// Set max position / rotation for each axis
int maxHorizontalPos = 145; // Max distance to be travelled in x/y direction {mm}
int maxAlpha = 45; // Max rotation about z axis {deg}
int maxPhiTheta = 45; // Max rotation about x / y axes {deg}

// Values to map between {mm} and {steps}
float legStroke = 155; // {mm} (limit is 155)
int maxSteps = 1200;   // Max number of steps for full leg extension {steps} (limit is 1200)

// Initialise encoders
AMS_AS5048B encoders[] = {AMS_AS5048B(0x40), AMS_AS5048B(0x44), AMS_AS5048B(0x48), AMS_AS5048B(0x4C), AMS_AS5048B(0x5C), AMS_AS5048B(0x5D)};
float encPosPrev[6];   // Previous encoder positions
float encPos[6];       // Current encoder positions
int revCounter[6];     // Counter of how many revolutions each encoder has made
float encPosActual[6]; // Actual position of the encoder [eg. full angle rather than mod(2*pi) or mod(360) ]

// ###################################################### //
//       Define arrays that will be populated later       //
// ###################################################### //

// Arrays to hold base and platform nodes. Last element is the hand
float baseNodes[6][3];  // Position vectors of base nodes in base frame
float platNodes[6][3];  // Position vectors of platform nodes in platform frame
float initLegLens[6];   // Lengths of the legs (and string) in their initial (origin) state {mm}
float rot[3][3];        // Rotation matrix

// Next position of the platform
float nextPlatPos[6];   // (x, y, z, phi, theta, alpha) coordinates of next platform position {mm / deg}
                        // Note that phi, theta and alpha are rotations about x, y and z, respectively.

// Array that will be used to move motors
int stepNumToMoveMotorsTo[6];  // Position to move motors to to achieve desired platform location {steps}

// ###################################################### //
//                  Receiving Serial Data                 //
// ###################################################### //

const byte numChars = 64;  // Max number of characters
char receivedChars[numChars];
char tempChars[numChars];  // Temporary array for parsing
bool newData = false;


// ###################################################### //
//                        Functions                       //
// ###################################################### //

// ---------------------- Startup ----------------------- //

void SetupMotorsAndEncoders() {
  // Initialise all motors. Set max speed/accel.
  for (byte i = 0; i < 6; i++) {
    motorArray[i] = new AccelStepper(AccelStepper(1, motorPins[i * 2], motorPins[i * 2 + 1]));
    encoders[i].begin();
    encoders[i].setClockWise(false);
    encoders[i].setZeroReg();

    // Set limits for motors
    motorArray[i]->setMaxSpeed(motMaxSpeed);  // Steps per second
    motorArray[i]->setAcceleration(motAccel); // Steps per second^2
    motorArray[i]->setMinPulseWidth(motMinPulseWidth);  // {us}
  }
}

void startupMotors(){
  // Run through startup sequence for all 6 motors, one at a time. Assumes the legs start at/near their shortest position.
  // Sequence looks like this:
    // 1) Move motor to extend leg
    //   1a) If no change in any encoder readings, try again. If still no reading, throw error
    // 2) Move motor to shorten leg until encoder value stops changing. Record this step value
    // 3) Move back and forth a few times to be sure. Record each "0" step value. Take average and move there.
    // 4) Zero the stepper and the encoder that was changing and record that it is linked to the relevant leg

  int neg = 1;  // Corrects for the mirrored nature of the motors (eg. odd might be CW while even is CCW for leg extension)
  byte temp = 99;  // Stores the two-digit byte from the moveMotorAndCheckEncoders function
  byte flag = 9;  // Checks status of encoders. 1 indicates one encoder has become more +ve, 2 is multiple encoders increased and 3 is none changed.
  byte enco = 9;  // Stores the encoder ID num (position in array) that is linked to the motor
  byte motEncMap[6];  // Will populate with mapping between encoders and leg motors. Item is motor, value is encoder.
  float encPosActualPrev; // Stores the previous reading for the encoder.
  byte numIterations = 3; // The number of times that the motor will cycle through the startup movement
  int steps[numIterations]; // Stores the step values that correspond to the zero points of each iteration

  for (byte mot = 0; mot < 6; mot++){
    motorArray[mot]->setMaxSpeed(200); // Keep it nice and slow for this process

    if (mot % 2 == 1){neg = 1;}
    else if (mot % 2 == 0){neg = -1;}

    temp = moveMotorAndCheckEncoders(mot, 200 * neg);
    // Serial.println();
    // Serial.print("Receieved flag + encoder: ");
    // Serial.println(temp);
    // Serial.println();
    flag = (temp / 10) % 10;
    enco = temp % 10;

    if (flag == 2){
      Serial.print("ERROR! Multiple encoders changed as a result of operating motor ");
      Serial.print(mot);
      Serial.println(". Trying again...");
      delay(500);

      temp = moveMotorAndCheckEncoders(mot, 50 * neg);
      flag = (temp / 10) % 10;
      enco = temp % 10;

      if (flag == 2){
        Serial.print("Unsuccessful reattempt at moving motor. Multiple encoders were still triggered by moving motor ");
        Serial.print(mot);
        Serial.println(". Moving to the next motor.");
        continue; // skip to next motor
      }

    } else if (flag == 3){
      Serial.print("Error! No encoders changed as a result of operating motor ");
      Serial.println(mot);
      Serial.println("Trying again...");
      delay(500);

      temp = moveMotorAndCheckEncoders(mot, 400 * neg);
      flag = (temp / 10) % 10;
      enco = temp % 10;

      if (flag == 3){
        Serial.print("Unsuccessful reattempt at moving motor ");
        Serial.print(mot);
        Serial.println(". Moving to the next motor.");
        Serial.println();
        continue; // skip to next motor
      }
    }

    motEncMap[mot] = enco; // Register this encoder as being related to motor number "mot"
    Serial.print("Registered encoder ");
    Serial.print(enco);
    Serial.print(" as being associated with motor ");
    Serial.println(mot);
    Serial.println();

    for (byte iter = 0; iter < numIterations; iter++){
      // Serial.println();
      // Serial.print("Iteration: ");
      // Serial.println(iter);
      // Serial.println();
      encPosActualPrev = 100; // Reset to some high value
      while (encPosActualPrev - encPosActual[enco] > 0.003){
        readEncoders();
        // Serial.println("Pre-movement: ");
        // Serial.println(encPosActualPrev);
        // Serial.println(encPosActual[enco]);
        // Serial.println();
        encPosActualPrev = encPosActual[enco];
        temp = moveMotorAndCheckEncoders(mot, -10 * neg);
        // Serial.print("Encoder ");
        // Serial.print(enco);
        // Serial.print(" post shrink movement = ");
        // Serial.println(encPosActual[enco]);
        // delay(100);
      }
      // Serial.println();
      // Serial.print("Bottomed out. Encoder reading: ");
      // Serial.print(encPosActual[enco]);
      // Serial.println(". Proceeding to next iteration.");
      steps[iter] = motorArray[mot]->currentPosition();
      temp = moveMotorAndCheckEncoders(mot, 50 * neg);
    }
    int zeroPos = (steps[0] + steps[1] + steps[2]) / 3;
    motorArray[mot]->moveTo(zeroPos + neg * 100); // Add some offset to stop the legs from bottoming out.
    motorArray[mot]->runToPosition();

    // Zero the motor and encoder
    motorArray[mot]->setCurrentPosition(0);
    encoders[motEncMap[mot]].setZeroReg();

    Serial.print("Finished initialising motor ");
    Serial.println(mot);
    Serial.println();

    motorArray[mot]->setMaxSpeed(10000);
    delay(200);
  }
}

byte moveMotorAndCheckEncoders(byte motor, int steps){
  // Returns a two digit byte where the first digit (tens) is the flag and the second digit (ones) is the encoder that changed
  byte flag = 0;
  byte encoder = 0; // Encoder num that changed
  float encoderThreshold = 0.1; // Ensures noise in encoders doesn't cause wrong encoder to get flagged {rad}
  float encPosActualPrev[6];
  int changed[6];

  // Record what the encoder readings pre-movement are
  readEncoders();
  for (byte enc = 0; enc < 6; enc++){
    // Serial.println();
    // Serial.print("Encoder ");
    // Serial.print(enc);
    // Serial.print(" pre-movement = ");
    // Serial.println(encPosActual[enc]);
    encPosActualPrev[enc] = encPosActual[enc];
  }

  // Serial.println();
  // Move the motor
  motorArray[motor]->move(steps);
  while (motorArray[motor]->distanceToGo() != 0){
    // Move the motor the prescribed number of steps
    motorArray[motor]->run();
    readEncoders();
  }
  
  delay(100); // Give the physical system time to respond (possibly not needed?)

  // After motor has moved, check the encoders for any increase
  readEncoders();
  for (byte enc = 0; enc < 6; enc++){
    // Serial.println();
    // Serial.print("Encoder ");
    // Serial.print(enc);
    // Serial.print(" post-movement = ");
    // Serial.println(encPosActual[enc]);
    if (encPosActual[enc] - encPosActualPrev[enc] > encoderThreshold){
      changed[enc] = 1;
      encoder = enc;

    } else {
      changed[enc] = 0;
    }
  }

  int checkSum = 0;
  for (byte i = 0; i < 6; i++){
    checkSum += changed[i];
    // Serial.print(changed[i]);
  }
  // Serial.println();

  if (checkSum == 1){
    // Everything went well
    flag = 1;
  } else if (checkSum == 0){
    // No encoders registered an increase
    flag = 3;
  } else if (checkSum > 1){
    // More than one encoder registered an increase
    flag = 2;
  }

  // Serial.println();
  // Serial.print("Return of flag + encoder: ");
  // Serial.println(flag * 10 + encoder);
  // Serial.println();

  return flag * 10 + encoder;
}

void BuildStewartPlatform() {
  // Build the SP using the given dimensions
  // Origin is at platform origin in lowest position

  float degToRad = PI / 180;

  // Define the angles to the nodes
  float gamma0 = 12;               // Offset from horizontal
  float gamma2 = baseSmallAngle;  // Angle between close base nodes {deg}
  float gamma1 = 120 - gamma2;    // Angle between far base nodes {deg}

  float lambda1 = platSmallAngle;                   // Angle between close platform nodes {deg}
  float lambda2 = 120 - lambda1;                    // Angle between far platform nodes {deg}
  float lambda0 = (gamma1 - lambda1) / 2 + gamma0;  // Offset from x axis for platform nodes {deg}

  float baseNodeAngles[6];  // Angles to each of the 6 base nodes {deg}
  float platNodeAngles[6];  // Angles to each of the 6 platform nodes {deg}

  // Define two indices that are used to orient the nodes properly
  float firstAngleIndex;
  float secondAngleIndex;

  for (int node = 0; node < 6; node++) {
    firstAngleIndex = floor((float(node) + 2) / 2);
    secondAngleIndex = ceil((float(node)) / 2);

    baseNodeAngles[node] = gamma0 + gamma1 * firstAngleIndex + gamma2 * secondAngleIndex;
    platNodeAngles[node] = lambda0 + lambda1 * firstAngleIndex + lambda2 * secondAngleIndex;

    baseNodes[node][0] = baseRad * cos(baseNodeAngles[node] * degToRad);
    baseNodes[node][1] = baseRad * sin(baseNodeAngles[node] * degToRad);
    // baseNodes[node][2] = -initialHeight;
    baseNodes[node][2] = 0;

    platNodes[node][0] = platRad * cos(platNodeAngles[node] * degToRad);
    platNodes[node][1] = platRad * sin(platNodeAngles[node] * degToRad);
    platNodes[node][2] = 0;

    // Serial.print("Base node: ");
    // Serial.println(node);
    // for (byte dim = 0; dim < 3; dim++){
    //   Serial.print(" dim:");
    //   Serial.print(dim);
    //   Serial.print(" value: ");
    //   Serial.println(baseNodes[node][dim]);
    // }
    // Serial.println();
  }
}

void CalcInitialLegLengths() {
  // Calculate the lengths of the legs {mm} (and the string between the base and the platform string guide) in
  // the origin position. This length will be effectively set as "home" (ie. step 0)

  float temp[3];  // To store the vector for the leg under consideration until the length can be calculated

  for (int leg = 0; leg < 6; leg++) {
    for (int dim = 0; dim < 3; dim++) {
      temp[dim] = startPos[dim] + platNodes[leg][dim] - baseNodes[leg][dim];
    }
    initLegLens[leg] = sqrt(pow(temp[0], 2) + pow(temp[1], 2) + pow(temp[2], 2));
    // Serial.println(initLegLens[leg]);
  }
}

// ------------------- Sub-functions -------------------- //
// Functions that are "two layers deep"

float cvtDegToRad(float angle){
  // Converts the provided angle {deg} to radians
  return angle * PI / 180;
}

int negFinder(byte motor){
  // Reverse the direction of travel for odd (or even) motors
  if (motor % 2 == 1){return 1;}
  else if (motor % 2 == 0){return -1;}
  else{return 0;}
}

void PopulateRotationMatrix(float phi, float theta, float alpha){
  // Populates the 3 x 3 rotation matrix needed to calculate leg lengths of SP

  // Convert angles to radians
  phi = cvtDegToRad(phi);
  theta = cvtDegToRad(theta);
  alpha = cvtDegToRad(alpha);

  // First column
  rot[0][0] =   cos(alpha) * cos(theta);
  rot[1][0] =   sin(alpha) * cos(theta);
  rot[2][0] = - sin(theta);

  // Second column
  rot[0][1] = - sin(alpha) * cos(phi) + cos(alpha) * sin(theta) * sin(phi);
  rot[1][1] =   cos(alpha) * cos(phi) + sin(alpha) * sin(theta) * sin(phi);
  rot[2][1] =   cos(theta) * sin(phi);

  // Third column
  rot[0][2] =   sin(alpha) * sin(phi) + cos(alpha) * sin(theta) * cos(phi);
  rot[1][2] = - cos(alpha) * sin(phi) + sin(alpha) * sin(theta) * cos(phi);
  rot[2][2] =   cos(theta) * cos(phi);
}

void SetAccelerations(int accel){
  // Set the acceleration of all motors to the nominated value
  for (byte mot = 0; mot < 6; mot++){
    motorArray[mot]->setAcceleration(accel);
  }
}

void ScaleSpeeds(float maxSpeed){
  // Scales the leg speeds so that they all arrive at their destination at the same time
  float furthestDist = 0; // Furthest distance that any leg needs to move by {steps}
  
  // Find the furthest distance travelled by any leg
  for (byte leg = 0; leg < 6; leg++){
    furthestDist = max(furthestDist, abs(motorArray[leg]->distanceToGo()));    
  }

  // Scale the speeds of the legs according to how far they need to move compared to furthestDist
  for (byte leg = 0; leg < 6; leg++){
    if (abs(motorArray[leg]->distanceToGo()) == furthestDist){
      motorArray[leg]->setMaxSpeed(maxSpeed);

    } else if (abs(motorArray[leg]->distanceToGo()) < furthestDist){
      float smallDist = motorArray[leg]->distanceToGo(); // Distance that this (sub-maximal) motor will move by
      motorArray[leg]->setMaxSpeed((smallDist / furthestDist) * maxSpeed);

    } else if (abs(motorArray[leg]->distanceToGo()) > furthestDist){
      Serial.println("ERROR in ScaleSpeeds!");
      Serial.print("Motor ");
      Serial.print(leg);
      Serial.println("Will move the furthest, but wasn't classified correctly.");
    }
  }
}

void moveToPosWithMaxSpeed(float x, float y, float z, float phi, float theta, float alpha, float maxSpeed){
  // Moves the platform to the nominated position (x, y, z, phi, theta, alpha) with the fastest motor moving at maxSpeed steps/sec
  nextPlatPos[0] = x;
  nextPlatPos[1] = y;
  nextPlatPos[2] = z;
  nextPlatPos[3] = phi;
  nextPlatPos[4] = theta;
  nextPlatPos[5] = alpha;

  CalcStepsToMoveTo();
  primeMotors();
  ScaleSpeeds(maxSpeed);
  moveMotorsUntilAllFinish();
}

void moveMotorsUntilAllFinish(){
  // Moves all six motors until they're all at their destination
  // Assumes they all have the same duration/steps in their path. BIG ASSUMPTION!!
  // Otherwise motion would be janky
  
  int counter = 1;
  while (counter != 0){
    counter = 0;
    for (byte mot = 0; mot < 6; mot++){
      motorArray[mot]->run();
      counter += abs(motorArray[mot]->distanceToGo());
      // Serial.print(counter);
      // Serial.print(" ");
    }
    // delay(5);
    // Serial.println();
  }

  // while (motorArray[0]->distanceToGo() != 0){
  //   for (byte mot = 0; mot < 6; mot++){
  //     motorArray[mot]->run();
  //   }
  // }
}

void parseInputPosData() {      // split the data into its parts
  strcpy(tempChars, receivedChars); // Necessary because strtok replaces commas with \0
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(tempChars, ","); // get the first set of data - leg 1 extension
  nextPlatPos[0] = constrain(atoi(strtokIndx), -maxHorizontalPos, maxHorizontalPos);     // x position
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  nextPlatPos[1] = constrain(atoi(strtokIndx), -maxHorizontalPos, maxHorizontalPos);     // y position
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  nextPlatPos[2] = constrain(atoi(strtokIndx), -legStroke, legStroke);            // z position
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  nextPlatPos[3] = constrain(atoi(strtokIndx), -maxPhiTheta, maxPhiTheta); // phi
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  nextPlatPos[4] = constrain(atoi(strtokIndx), -maxPhiTheta, maxPhiTheta); // theta
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  nextPlatPos[5] = constrain(atoi(strtokIndx), -maxAlpha, maxAlpha);       // alpha

  Serial.print(" Moving to: x = ");
  Serial.print(nextPlatPos[0]);
  Serial.print(" y = ");
  Serial.print(nextPlatPos[1]);
  Serial.print(" z = ");
  Serial.print(nextPlatPos[2]);
  Serial.print(" phi = ");
  Serial.print(nextPlatPos[3]);
  Serial.print(" theta = ");
  Serial.print(nextPlatPos[4]);
  Serial.print(" alpha = ");
  Serial.print(nextPlatPos[5]);
  Serial.println(" ");
}

// ---------------------- Ongoing ----------------------- //

void CalcStepsToMoveTo(){
  // Calculates how long each of the legs need to be {steps} at the next position
  float T[3] = {0, 0, 0}; // Translation from {O} to {P}
  float temp[3] = {0, 0, 0}; // To store the coordinates of the post-movement leg
  float posToMoveLegTo = 0; // Length that the leg should move to {mm}

  // Serial.println("T: ");
  for (byte dim = 0; dim < 3; dim++){
    T[dim] = nextPlatPos[dim] + startPos[dim]; // Grab new translational position
    // Serial.print(T[dim]);
    // Serial.print(", ");
  } 

  PopulateRotationMatrix(nextPlatPos[3], nextPlatPos[4], nextPlatPos[5]); // Populate the rotation matrix

  // Serial.println();
  // Serial.println("Step num to move motors to: ");
  for (byte leg = 0; leg < 6; leg++){
    for (byte dim = 0; dim < 3; dim++){
      temp[dim] = T[dim] + rot[dim][0] * platNodes[leg][0] + 
                           rot[dim][1] * platNodes[leg][1] +
                           rot[dim][2] * platNodes[leg][2] -
                           baseNodes[leg][dim];
      // Serial.print("Dim: ");
      // Serial.print(dim);
      // Serial.print(" temp[dim]: ");
      // Serial.println(temp[dim]);
    }
    // Serial.println();
    // Serial.print("Length for leg ");
    // Serial.print(leg);
    // Serial.print(": ");
    // Serial.println(sqrt(pow(temp[0], 2) + pow(temp[1], 2) + pow(temp[2], 2)));
    // Serial.println("######");
    posToMoveLegTo = sqrt(pow(temp[0], 2) + pow(temp[1], 2) + pow(temp[2], 2)) - initLegLens[leg];
    // Constrain the movement of the legs. This could be done with constrain(), but that wouldn't allow for error messages.
    if (posToMoveLegTo > legStroke){
      Serial.println();
      Serial.print("WARNING! - Leg ");
      Serial.print(leg);
      Serial.println("'s motion had to be truncated, as it was commanded to move to be too long");
      Serial.print("The attempted command was: ");
      Serial.println(posToMoveLegTo, 5);
      posToMoveLegTo = legStroke;
    } else if (posToMoveLegTo < 0){
      Serial.println();
      Serial.print("WARNING! - Leg ");
      Serial.print(leg);
      Serial.println("'s motion had to be truncated, as it was commanded to move to be too short");
      Serial.print("The attempted command was: ");
      Serial.println(posToMoveLegTo, 5);
      posToMoveLegTo = 0;
    }
    Serial.println();
    Serial.print("Leg ");
    Serial.print(leg);
    Serial.print(" pre-map: ");
    Serial.print(posToMoveLegTo);
    Serial.print(", ");
    stepNumToMoveMotorsTo[leg] = map(posToMoveLegTo, 0, legStroke, 0, maxSteps); // Map the {mm} value to {steps}
    Serial.print(" Post-map: ");
    Serial.println(stepNumToMoveMotorsTo[leg]);
    Serial.println();
  }
}

void readEncoders() {
  // Read the encoder values
  // Serial.println();
  // Serial.println("From readEncoders: ");
  for (byte enc = 0; enc < 6; enc++){
    encoders[enc].updateMovingAvgExp();
    encPos[enc] = encoders[enc].angleR(4, false); // 4 is radians. 3 is deg if preferred
    // encPos[enc] = encoders[enc].getMovingAvgExp(4); 

    if (encPos[enc] + PI < encPosPrev[enc]){
      revCounter[enc]++;

    } else if (encPos[enc] - PI > encPosPrev[enc]){
      revCounter[enc]--;
    }

    encPosPrev[enc] = encPos[enc];
    encPosActual[enc] = encPos[enc] + revCounter[enc] * 2 * PI;

    // Serial.print(encPosActual[enc], DEC);
    // Serial.print(", ");
  }
  // Serial.println();
}

void primeMotors() {
  // Get motors ready to move by setting their destinations {steps}
  // Serial.println("Motors moving to: ");
  for (byte i = 0; i < 6; i++) {
    int neg = negFinder(i);
    motorArray[i]->moveTo(neg * (stepNumToMoveMotorsTo[i]));
    // Serial.print("Motor ");
    // Serial.print(i);
    // Serial.print(" step to move to: ");
    // Serial.print(stepNumToMoveMotorsTo[i]);
  }
  // Serial.println();
}

void runMotors(){
  // Self-explanatory. Call as often as possible.
  for (byte mot = 0; mot < 6; mot++){
    motorArray[mot]->run();
  }
}

void manualBigMoves(){
  // Pattern controller!
  Serial.println("Pattern controller!!");
  Serial.println("Enter <P> to set pattern");
  Serial.println("Enter <S> to set max speed (<14,000)");
  Serial.println("Enter <A> to set max acceleration");
  Serial.println("Enter <R> to reset program");
  Serial.println("Enter <done> when finished");
  Serial.println("Enter <stop> to stop pattern repeating");

  int mode = 0; // What data is being sent? (0 is pattern, 1 is max speed, 2 is acceleration)
  int speed = 1000;
  int accel = 5000;
  int done = 0;
  int stop = 0; // Value corresponds to pattern that is currently repeating. 0 is none
  receivedChars[0] = {' '}; // To avoid the never-ending cycle of resetting

  while (done==0){
    readEncoders();
    recvWithStartEndMarkers();

    if (stop != 0){
      newData = true;
    }

    if (newData == true){
      if (String(receivedChars).compareTo("done") == 0){
        Serial.println("Done!");
        done = 1;
      } else if (String(receivedChars).compareTo("P") == 0){
        Serial.print("Enter the pattern you'd like to see (1 is straight up and down, 2 is straight up and down three times, ");
        Serial.print("3 is to set your own destination, 4 will return home, 5 will move in a circle, 6 will move side to side, ");
        Serial.println("7 will move in a screw pattern (up/down), 8 will move in a bouncy pattern, and 9 will throw the ball.");
        mode = 0;

      } else if (String(receivedChars).compareTo("S") == 0){
        Serial.println("Setting max speed (steps/sec)");
        mode = 1;

      } else if (String(receivedChars).compareTo("A") == 0){
        Serial.println("Setting max acceleration (steps/sec^2)");
        mode = 2;
      } else if (String(receivedChars).compareTo("R") == 0){
        Serial.println("Resetting...");
        startupMotors();
        manualBigMoves();
      } else if (String(receivedChars).compareTo("stop") == 0){
        stop = 0;
      }

      float data = 0;
      if (stop != 0){
        data = float(stop);
      } else{
        data = atoi(receivedChars);
      }

      if (data != 0){
        if (mode == 0){ // If setting pattern
          if (data == 1){
            Serial.print("Moving up and down at ");
            Serial.print(speed);
            Serial.print(" steps/sec and ");
            Serial.print(accel);
            Serial.println(" steps/sec^2");
            moveStraightUpAndDown(speed);
            stop = 1;

          } else if(data == 2){
            Serial.print("Moving up and down at ");
            Serial.print(speed);
            Serial.print(" steps/sec and ");
            Serial.print(accel);
            Serial.println(" steps/sec^2, three times");
            for (byte counter = 0; counter < 3; counter++){
              moveStraightUpAndDown(speed);
            }

          } else if(data == 3){
            moveToChosenPos(speed);

          } else if(data == 4){
            GoHome(speed);
            
          } else if(data == 5){
            moveCircles(speed);
            stop = 5;
            
          } else if(data == 6){
            moveSideToSide(speed);
            stop = 6;
            
          } else if(data == 7){
            moveScrew(speed);
            stop = 7;
            
          } else if(data == 8){
            moveBouncy(speed);
            stop = 8;
            
          } else if(data == 9){
            throwBall(speed);
            stop = 9;
            
          } else{
            Serial.print("No pattern assigned to: ");
            Serial.println(data);
          }

        } else if (mode == 1){ // If setting max speed {steps/sec}
          Serial.print("Setting max speed to: ");
          Serial.print(data);
          Serial.println(" steps/sec");
          speed = data;
          for (byte mot = 0; mot < 6; mot++){
            motorArray[mot]->setMaxSpeed(data);
          }
        } else if (mode == 2){ // If setting max acceleration {steps/sec^2}
          Serial.print("Setting max acceleration to: ");
          Serial.print(data);
          Serial.println(" steps/sec^2");
          accel = data;
          SetAccelerations(accel);
        }
      }
      newData = false;
      Serial.println();
      Serial.print("Current max speed: ");
      Serial.print(speed);
      Serial.print(" steps/sec. Current max accel: ");
      Serial.println(accel);
      Serial.println();
    }
  }
}

// ------------------- Move Patterns -------------------- //

void moveStraightUpAndDown(int speed){
  // Moves all six motors straight up and down at the given speed
  moveToPosWithMaxSpeed(0, 0, legStroke, 0, 0, 0, float(speed)); // Move to fully extended pos
  // delay(100);
  moveToPosWithMaxSpeed(0, 0, 0, 0, 0, 0, float(speed)); // Return home
}

void moveToChosenPos(int speed){
  // Asks for the desired position to move to, and then moves there
  byte received = 0;
  newData = false;
  Serial.print("Choosing the pose to move to. Enter your desired pose in the format: ");
  Serial.println("'<x, y, z, phi, theta, alpha>'");
  Serial.println("Enter <done> to return to main program");

  while (received == 0){
    recvWithStartEndMarkers();
    if (newData == true){
      if (String(receivedChars).compareTo("done") == 0){
        manualBigMoves();
      }
      parseInputPosData();
      moveToPosWithMaxSpeed(nextPlatPos[0], nextPlatPos[1], nextPlatPos[2], nextPlatPos[3], nextPlatPos[4], nextPlatPos[5], float(speed));
      newData = false;
    }
  }
}

void GoHome(int speed){
  // Moves motors to home
  moveToPosWithMaxSpeed(0, 0, 0, 0, 0, 0, float(speed)); // Return home
}

void moveCircles(int speed){
  // Moves the platform in a circle
  float radius = 100; // {mm}
  int numPoints = 50; // Number of points to discretize circle into
  float zDist = 100; // {mm}

  Serial.println("Moving the platform in a circle");

  for (byte pt = 0; pt < numPoints; pt++){
    float angle = 2 * PI * (float(pt) / numPoints);
    moveToPosWithMaxSpeed(radius * cos(angle), radius * sin(angle), zDist, 0, 0, 0, float(speed));
  }
  GoHome(speed);
}

void moveSideToSide(int speed){
  // Moves platform side to side along x axis
  moveToPosWithMaxSpeed(120, 0, 100, 0, 0, 0, float(speed));
  moveToPosWithMaxSpeed(-120, 0, 100, 0, 0, 0, float(speed));
}

void moveScrew(int speed){
  // Moves platform up and down in a screw motion
  moveToPosWithMaxSpeed(0, 0, 80, 0, 0, 30, float(speed));
  moveToPosWithMaxSpeed(0, 0, 155, 0, 0, 0, float(speed));
  moveToPosWithMaxSpeed(0, 0, 80, 0, 0, -30, float(speed));
  moveToPosWithMaxSpeed(0, 0, 0, 0, 0, 0, float(speed));
}

void moveBouncy(int speed){
  // Bounces the platform around
  moveToPosWithMaxSpeed(100, 100, 80, 0, 0, 0, float(speed));
  GoHome(speed);
  moveToPosWithMaxSpeed(100, -100, 80, 0, 0, 0, float(speed));
  GoHome(speed);
  moveToPosWithMaxSpeed(-100, -100, 80, 0, 0, 0, float(speed));
  GoHome(speed);
  moveToPosWithMaxSpeed(-100, 100, 80, 0, 0, 0, float(speed));
  GoHome(speed);
}

void throwBall(int speed){
  // Raises platform to peak slowly, waits 500 ms, retracts slowly, then throws!
  newData = false;

  moveToPosWithMaxSpeed(0, 0, 155, 0, 0, 0, 500);
  delay(500);
  moveToPosWithMaxSpeed(0, 0, 0, 0, 0, 0, 500);
  delay(200);

  while (1){
    recvWithStartEndMarkers();
    if (newData == true){
      if (String(receivedChars).compareTo("stop") == 0){
        break;
      }
      newData = false;
    }
    moveToPosWithMaxSpeed(0, 0, 155, 0, 0, 0, float(speed));
    delay(250); // Duration of ball in the air
    moveToPosWithMaxSpeed(0, 0, 0, 0, 0, 0, 500);
    delay(200);
  }
  
}

// ----------------------- Serial ------------------------ //

void recvWithStartEndMarkers() {
  static bool recvInProgress = false;
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
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

// ---------------------- Debugging ---------------------- //

void CalcLegLengthsAtPos(float x, float y, float z, float phi, float theta, float alpha){
  // Calcs the leg lengths when the platform is at the nominated position (x, y, z, phi, theta, alpha)
  nextPlatPos[0] = x;
  nextPlatPos[1] = y;
  nextPlatPos[2] = z;
  nextPlatPos[3] = phi;
  nextPlatPos[4] = theta;
  nextPlatPos[5] = alpha;
  CalcStepsToMoveTo();
  CalcPlatCOMPos();
}

void CalcPlatCOMPos(){
  // Takes in the most recent position command and calculates where the COM of the platform will be.
  // Used to debug issue of rotations also inducing translations in the platform
  float pos[3] = {0, 0, 0};

  Serial.println("Platform COM pos: ");
  for (byte dim = 0; dim < 3; dim++){
    pos[dim] = nextPlatPos[dim] + startPos[dim] + rot[dim][0] * 0 + 
                                                   rot[dim][1] * 0 +
                                                   rot[dim][2] * 0 -
                                                   0;
    Serial.print("Dim ");
    Serial.print(dim);
    Serial.print(": ");
    Serial.println(pos[dim]);
  }
}


// ----------------------- Backup ------------------------ //
// These functions were used in a previous version of the code and are just being kept here as a backup / for easier reference

void manualMove(){
  // int mode = 0; // What data is being sent? (0 is motor selection, 1 is position, 2 is speed)
  // int motor;
  // int done = 0;

  // while (done==0){
  //   readEncoders();
  //   recvWithStartEndMarkers();

  //   if (newData == true){
  //     if (String(receivedChars).compareTo("done") == 0){
  //       Serial.println("Done!");
  //       done = 1;
  //     } else if (String(receivedChars).compareTo("M") == 0){
  //       Serial.println("Enter the number of the motor you'd like to operate.");
  //       mode = 0;

  //     } else if (String(receivedChars).compareTo("P") == 0){
  //       Serial.println("Enter the position you'd like the chosen motor to move to");
  //       mode = 1;

  //     } else if (String(receivedChars).compareTo("V") == 0){
  //       Serial.println("Setting max speed");
  //       mode = 2;
  //     }

  //     float data = atoi(receivedChars);
  //     if (data != 0){
  //       if (mode == 0){
  //         motor = data;
  //         if (motor == 7){
  //           Serial.println("All motors will move together");
  //         } else{
  //           Serial.print("Motor ");
  //           Serial.print(motor);
  //           Serial.println(" chosen");
  //         }
  //       } else if (mode == 1){
  //         if (motor == 7){
  //           for (byte mot = 0; mot < 6; mot++){
  //             int neg = 1;
  //             if (mot % 2 == 1){neg = 1;}
  //             else if (mot % 2 == 0){neg = -1;}
  //             motorArray[mot]->move(neg * data);
  //           }
  //         } else{
  //           motorArray[motor]->move(data);
  //         }
  //         Serial.print("Moving motor ");
  //         Serial.print(motor);
  //         Serial.print(" by ");
  //         Serial.print(data);
  //         Serial.println(" steps");
  //       } else if (mode == 2){
  //         if (motor == 7){
  //           for (byte mot = 0; mot < 6; mot++){
  //             motorArray[mot]->setMaxSpeed(data);
  //           }
  //         } else{
  //           motorArray[motor]->setMaxSpeed(data);
  //         }
  //         Serial.print("Setting speed of motor ");
  //         Serial.print(motor);
  //         Serial.print(" to ");
  //         Serial.print(data);
  //         Serial.println(" steps/sec");
  //       }
  //     }
  //     newData = false;
  //   }
  //   for (byte mot = 0; mot < 6; mot++){
  //     motorArray[mot]->run();
  //   }
  // }
}

void moveAllMotorsToGivenPos(int pos){
  // Will move all of the motors to the same position. Particularly useful just after startup to move to middle position.
  int neg = 0;
  for (byte mot = 0; mot < 6; mot++){
    if (mot % 2 == 1){neg = 1;}
    else if (mot % 2 == 0){neg = -1;}
    motorArray[mot]->moveTo(neg * pos);
  }

  while (motorArray[1]->distanceToGo() != 0){
    for (byte mot = 0; mot < 6; mot++){
      motorArray[mot]->run();
    }
  }
}

// ###################################################### //
//                         Setup                          //
// ###################################################### //

void setup() {
  Serial.begin(9600);
  Serial.println("###########################################################################");
  Serial.println("                                 STARTING                                  ");
  Serial.println("###########################################################################");

  BuildStewartPlatform();
  CalcInitialLegLengths();
  // CalcLegLengthsAtPos(0, 0, 100, 20, 0, 0);

  Wire.begin();

  SetupMotorsAndEncoders();

  startupMotors();
  manualBigMoves();

}

// ###################################################### //
//                       Main Loop                        //
// ###################################################### //

void loop() {
  // recvWithStartEndMarkers();
  // if (newData == true){
  //   parseData();
  //   // primeMotors();
  //   newData = false;
  // }

  // runMotors();
  // readEncoders();

}
