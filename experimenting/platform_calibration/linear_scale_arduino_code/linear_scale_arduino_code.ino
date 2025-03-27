#include "QuadEncoder.h"
#include <FlexCAN_T4.h>

// --- Configuration ---
// Using encoder channel 1 with Phase A on pin 0 and Phase B on pin 1.
QuadEncoder encoder(1, 0, 1, 0);

// Conversion factor
const float mm_per_count = 0.001;

// Configure the CAN broadcast rate (Hz)
const float can_broadcast_rate = 1000; // Hz
// Calculate wait time (in ms) between messages.
const unsigned long can_wait_time_ms = (unsigned long)(1000.0 / can_broadcast_rate);

// Configure whether we're using a Serial interface for debugging.
const bool using_serial = false;

// Instantiate the CAN bus on CAN1 (with CRX1 on pin 23, CTX1 on pin 22).
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

void setup() {
  // Initialize the encoder.
  encoder.setInitConfig();
  encoder.init();

  // Initialize the CAN bus.
  Can1.begin();
  Can1.setBaudRate(1000000); // 1 Mbps
  
  // Initialize Serial if needed for debugging.
  if (using_serial) {
    Serial.begin(115200);
    while (!Serial); // Wait for Serial monitor
    Serial.println("Encoder CAN Test Started");
  }
}

void loop() {
  static unsigned long lastSendTime = millis();
  unsigned long now = millis();

  // Check if it's time to send a new CAN message.
  if (now - lastSendTime >= can_wait_time_ms) {
    lastSendTime = now;

    // Read the current encoder count.
    long count = encoder.read();

    // Convert count to physical position (mm).
    float position_mm = count * mm_per_count;

    // Prepare a CAN message (4-byte payload for the float value).
    CAN_message_t msg;
    msg.id  = 0x7E0;  // Node ID: 0x7E0
    msg.len = 4;      // 4-byte payload

    // Pack the float into the message buffer using a union.
    union {
      float f;
      uint8_t bytes[4];
    } data;
    data.f = position_mm;
    for (int i = 0; i < 4; i++) {
      msg.buf[i] = data.bytes[i];
    }

    // Send the CAN message.
    Can1.write(msg);

    // Optional debugging output.
    if (using_serial) {
      Serial.print("Count: ");
      Serial.print(count);
      Serial.print(" | Position (mm): ");
      Serial.println(position_mm, 6);
    }
  }
}
