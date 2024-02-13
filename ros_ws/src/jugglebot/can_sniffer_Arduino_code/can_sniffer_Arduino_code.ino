#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
uint32_t receivedCount = 0;
uint32_t lastReportTime = 0;
const uint32_t reportInterval = 500;  // Report every XXX milliseconds

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  can1.begin();
  can1.setBaudRate(1000000);
  can1.setMaxMB(16);
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(canSniff);
  can1.mailboxStatus();

  Serial.println("CAN Bus Monitor Setup Complete");
}

void canSniff(const CAN_message_t &msg) {
  receivedCount++;
  // Process the message or store stats as needed
}

void reportStatus() {
  uint32_t currentTime = millis();
  if (currentTime - lastReportTime > reportInterval) {
    // Publishing to the CAN bus
    CAN_message_t msg;
    msg.id = 0x20;  // Per the ODrive docs, 0x20 should be safe from conflict with any ODrive messages
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
    Serial.print("CAN traffic report: ");
    Serial.print(msg.buf[0] + (msg.buf[1] << 8));
    Serial.print(" messages in ");
    Serial.print(msg.buf[2] + (msg.buf[3] << 8));
    Serial.println(" ms interval.");
  }
}


void loop() {
  can1.events();  // Needed to process incoming messages and call canSniff
  reportStatus();
}
