/* This code is for the Teensy 4.0 on Jugglebot's 'platform' (attached near the hand motor ODrive)
This code currently fulfils these functions:
 - Listens to the CAN bus and reports how many messages were received over the past XXX ms (where XXX is defineable in the code)
 - Reporting the inclination of the arm from an SCL3300 inclinometer

It also allows for debugging specific CAN messages (based on arbitration ID) 

NOTE on CAN ids:

Odrives need a 'block' of 32 arbitration IDs, starting at the node ID. 
For 7 ODrives (with CAN id 0—7), this means we need to stay away from arbitration IDs between 0x00 and 0xE0 (7 * 32 = 224 = E0)

Additionally, there is a 'broadcast id' that will transmit to all ODrives on the bus (id = 0x3f<<5 = 0x7E0), so we need to keep out of its block,
which spans to the end of the range (7FF)

Therefore, 'safe' IDs are in the range:
0xE0 --> 0x7E0

*/
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <SCL3300.h>
#include <Arduino.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
SCL3300 inclinometer;

// For CanSniffer Functionality
uint32_t receivedCount = 0;
uint32_t lastReportTime = 0;
const uint32_t reportInterval = 500;  // Report every XXX milliseconds
const uint32_t REPORT_ID = 0x7DF; // Should be safe from conflict with any ODrive messages.

// For debugging a specific CAN message
const uint32_t node_id = 0;
const uint32_t cmd_id = 0x018;
const uint32_t debugID = (node_id << 5) | cmd_id;

// For Timing Analysis of Specific CAN Messages
uint32_t timingID = (node_id << 5) | 0x04 ; // Listen to RxSdo
uint16_t targetContentID = 385; // Specifically look our for 383 - "pos_setpoint"
unsigned long lastMessageTime = 0;
const unsigned long analysisPeriod = 500; // Analysis period in milliseconds 
unsigned long totalIntervals = 0;
int messageCount = 0;

// For inclinometer readings
uint32_t tiltID = 0x7DE; // Should be safe from conflict with ODrive messages

// Structure for relaying the tilt of the platform
struct platformTilt {
  float tiltX;  // {radians}
  float tiltY;  // {radians}
};

// For state updating
uint32_t stateUpdateID = 0x6E0; // For updating the state on the Teensy or reporting the last known state to the Jetson
// Structure to hold the robot state
struct RobotState {
  bool is_homed;
  bool levelling_complete;
  float pose_offset_tiltX;
  float pose_offset_tiltY;
};

// Initialize the state
RobotState state = {false, false, 0.0, 0.0};

/*********** canSniff ***********/
void canSniff(const CAN_message_t &msg) {
  receivedCount++;

  // Check for the specific message ID to trigger inclinometer reading and response
  if (msg.id == tiltID) {
    platformTilt tilt = getInclination();
    sendTiltData(tilt);
  }

  // Check for state update messages
  if (msg.id == stateUpdateID) {
    if (msg.len == 1 && msg.buf[0] == 0x01) {
      // If the received message is a request for the Teensy's last known state
      // Respond with the current state
      CAN_message_t response_msg;
      createStateCANMessage(state, response_msg);
      response_msg.id = stateUpdateID;  // Use the same CAN ID for response
      can1.write(response_msg);

      Serial.println("State sent to host PC.");

    } else if (msg.len == 8) {
      // Decode the state update message
      decodeStateCANMessage(msg, state);
      Serial.println("State updated from host PC:");

      // Output for debugging
      Serial.print("Is Homed: "); Serial.println(state.is_homed);
      Serial.print("Levelling Complete: "); Serial.println(state.levelling_complete);
      Serial.print("tiltX: "); Serial.println(state.pose_offset_tiltX);
      Serial.print("tiltY: "); Serial.println(state.pose_offset_tiltY);
      Serial.println();
    }
  }

  // debugPrintMessage(msg);

  analyzeMessageTiming(msg);

  // For debugging purposes, print the message content
  // Serial.printf("CAN Message Received: ID = 0x%X, DLC = %d, Data = ", msg.id, msg.len);
  // for (int i = 0; i < msg.len; ++i) {
  //     Serial.printf("0x%02X ", msg.buf[i]);
  // }
  // Serial.println();
}

/*********** Debugging Function to Print Message Contents with a Given Arbitration ID ***********/

void debugPrintMessage(const CAN_message_t &msg) {
    if (msg.id == debugID) {
        Serial.printf("Debug: CAN Message with ID 0x%X, DLC = %d, Data = ", msg.id, msg.len);
        for (int i = 0; i < msg.len; ++i) {
            Serial.printf("0x%02X ", msg.buf[i]);
        }
        Serial.println();

        if (msg.len >= 8) {
            // Unpack data into two floats using little-endian notation
            float value1, value2;
            memcpy(&value1, &msg.buf[0], sizeof(float));
            memcpy(&value2, &msg.buf[4], sizeof(float));

            Serial.printf("Unpacked floats: value1 = %f, value2 = %f\n", value1, value2);
        } else {
            Serial.println("Error: Insufficient data length to unpack two floats.");
        }
    }
}

void analyzeMessageTiming(const CAN_message_t &msg) {
  // For counting number of messages in "analysisPeriod":
  if (msg.id == timingID) {
      unsigned long currentTime = millis();
      if (lastMessageTime > 0) { // Avoid the first message
          unsigned long interval = currentTime - lastMessageTime;
          totalIntervals += interval;
          messageCount++;
      }
      lastMessageTime = currentTime;

      // Check if the analysis period has elapsed
      if (currentTime - lastReportTime >= analysisPeriod) {
          if (messageCount > 0) {
              unsigned long averageInterval = totalIntervals / messageCount;
              Serial.print("Average interval for CAN ID 0x");
              Serial.print(timingID, HEX);
              Serial.print(": ");
              Serial.print(averageInterval);
              Serial.println(" ms");

              // Reset the counters after reporting
              totalIntervals = 0;
              messageCount = 0;
              lastReportTime = currentTime;
          }
      }
  }

  // For simply printing the interval between every subsequent message (useful for monitoring message regularity):
  // if (msg.id == timingID) {
  //   // Check if the message's 2nd and 3rd bytes match the target ID (little-endian)
  //   uint16_t contentID = (uint16_t)(msg.buf[2] << 8 | msg.buf[1]);
  //   if (contentID == targetContentID) {
  //     unsigned long currentTime = millis();
  //     if (lastMessageTime > 0) { // Ensure this is not the first message
  //         unsigned long interval = currentTime - lastMessageTime;
  //         Serial.print("Interval between messages for CAN ID 0x");
  //         Serial.print(timingID, HEX);
  //         Serial.print(" with content ID ");
  //         Serial.print(targetContentID);
  //         Serial.print(": ");
  //         Serial.print(interval);
  //         Serial.println(" ms");
  //     }
  //     lastMessageTime = currentTime; // Update last message time
  //   }
  // }
}

void reportStatus() {
  uint32_t currentTime = millis();
  if (currentTime - lastReportTime > reportInterval) {
    // Publishing to the CAN bus
    CAN_message_t msg;
    msg.id = REPORT_ID;  
    msg.len = 4;
    msg.buf[0] = receivedCount & 0xFF;
    msg.buf[1] = (receivedCount >> 8) & 0xFF;
    msg.buf[2] = reportInterval & 0xFF;
    msg.buf[3] = (reportInterval >> 8) & 0xFF;

    // Send the message
    can1.write(msg);

    receivedCount = 0;
    lastReportTime = currentTime;

    // Debugging output
    // Serial.print("CAN traffic report: ");
    // Serial.print(msg.buf[0] + (msg.buf[1] << 8));
    // Serial.print(" messages in ");
    // Serial.print(msg.buf[2] + (msg.buf[3] << 8));
    // Serial.println(" ms interval.");

  }
}

/*********** FUNCTIONS FOR INCLINOMETER ***********/
// Function to convert angle from 0-360 to -180 to +180
float convertTo180Range(float angle) {
  if (angle > 180.0) {
    angle -= 360.0;
  }
  return angle;
}

// Function to get the inclination of the platform
platformTilt getInclination() {
  platformTilt tilt;  // Declare the platformTilt structure
  const int maxAttempts = 3;  // Maximum number of attempts to read the inclinometer
  int attempt = 0;
  bool success = false;

  while (attempt < maxAttempts && !success) {
    // Read angle data from the inclinometer, if available
    if (inclinometer.available()) {
      // Retrieve the angle data
      float angleY = convertTo180Range(inclinometer.getCalculatedAngleY());
      float angleZ = convertTo180Range(inclinometer.getCalculatedAngleZ());

      // Map angles to the robot's x and y axes. Found experimentally
      float tiltX = -angleZ;
      float tiltY = -angleY;
      
      // Print angle data in a format suitable for the Serial Plotter
      Serial.print("X:"); Serial.print(tiltX); Serial.print(" ");
      Serial.print("Y:"); Serial.println(tiltY);
      Serial.println();

      // Convert degrees to radians
      tiltX = tiltX * PI / 180;
      tiltY = tiltY * PI / 180;

      tilt.tiltX = tiltX;
      tilt.tiltY = tiltY;

      success = true;  // Reading was successful

    } else {
      inclinometer.reset();  // Reset the inclinometer
      delay(100);  // Wait for the inclinometer to reset
    }

    attempt++;
  }

  if (!success) {
    // If we fail to get a reading after three attempts, set default values
    tilt.tiltX = 3.14;
    tilt.tiltY = 3.14;
  }

  return tilt;
}

// Function to send tilt data over the CAN bus
void sendTiltData(platformTilt tilt) {
  CAN_message_t msg;
  msg.id = tiltID; // Same ID as the request
  msg.len = 8;

  // Pack the tilt data into the CAN message
  uint8_t* tiltXBytes = (uint8_t*)&tilt.tiltX;
  uint8_t* tiltYBytes = (uint8_t*)&tilt.tiltY;

  for (int i = 0; i < 4; i++) {
    msg.buf[i] = tiltXBytes[i];
    msg.buf[i + 4] = tiltYBytes[i];
  }

  // Send the message
  can1.write(msg);

  // Debugging output
  Serial.print("Sent tilt data: ");
  Serial.print("X = ");
  Serial.print(tilt.tiltX);
  Serial.print(", Y = ");
  Serial.println(tilt.tiltY);
}

/*********** State Update Functions ***********/

// Function to send homing status over the CAN bus
void createStateCANMessage(const RobotState &state, CAN_message_t &msg) {
  // Byte 0: Boolean flags
  uint8_t flags = 0;
  flags |= (state.is_homed) << 0;
  flags |= (state.levelling_complete) << 1;

  // Bytes 1-2: Pose offset (tiltX), scaled by 1000
  int16_t tiltX_scaled = (int16_t)(state.pose_offset_tiltX * 1000);

  // Bytes 3-4: Pose offset (tiltY), scaled by 1000
  int16_t tiltY_scaled = (int16_t)(state.pose_offset_tiltY * 1000);

  // Construct CAN message
  msg.len = 8;
  msg.buf[0] = flags;
  msg.buf[1] = tiltX_scaled & 0xFF;
  msg.buf[2] = (tiltX_scaled >> 8) & 0xFF;
  msg.buf[3] = tiltY_scaled & 0xFF;
  msg.buf[4] = (tiltY_scaled >> 8) & 0xFF;
  msg.buf[5] = 0;  // Reserved for future use
  msg.buf[6] = 0;  // Reserved for future use
  msg.buf[7] = 0;  // Reserved for future use
}

void decodeStateCANMessage(const CAN_message_t &msg, RobotState &state) {
    if (msg.len == 8) {
        // Byte 0: Extract flags
        uint8_t flags = msg.buf[0];
        state.is_homed = flags & 0x01;
        state.levelling_complete = (flags >> 1) & 0x01;

        // Bytes 1-2: Extract tiltX (scaled by 1000)
        int16_t tiltX_scaled = (msg.buf[2] << 8) | msg.buf[1];
        state.pose_offset_tiltX = tiltX_scaled / 1000.0;

        // Bytes 3-4: Extract tiltY (scaled by 1000)
        int16_t tiltY_scaled = (msg.buf[4] << 8) | msg.buf[3];
        state.pose_offset_tiltY = tiltY_scaled / 1000.0;
    }
}


void setup() {
  Serial.begin(115200);

  can1.begin();
  can1.setBaudRate(1000000);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);
  can1.mailboxStatus();

  Serial.println("CAN Bus Monitor Setup Complete");

  // Initialize the SCL3300
  while (inclinometer.begin() == false) {
    Serial.println("Murata SCL3300 inclinometer not connected. Resetting...");
    
    inclinometer.reset();
    delay(500);
  }

  Serial.println("SCL3300 Initialized!");
}

void loop() {
  can1.events();  // Needed to process incoming messages and call canSniff
  reportStatus();
}
