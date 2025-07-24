/*  Teensy 4.0 dual-CAN loop-back test
 *
 *  CAN1  TX → pin 23   (CTX1)   -> transceiver TXD
 *        RX ← pin 22   (CRX1)   <- transceiver RXD
 *
 *  CAN2  TX → pin 1    (CTX2)   -> transceiver TXD
 *        RX ← pin 0    (CRX2)   <- transceiver RXD
 *
 *  Baud-rate: 1 Mbit/s (change if your setup needs something else)
 *
 *  Library:  FlexCAN_T4 by @tonton81
 *            https://github.com/tonton81/FlexCAN_T4
 */

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;

uint32_t counter = 0;

// helper to print one frame nicely
void printFrame(const char *tag, const CAN_message_t &msg) {
  Serial.printf("%s  ID:0x%03X  DLC:%u  Data:", tag, msg.id, msg.len);
  for (uint8_t i = 0; i < msg.len; ++i) {
    Serial.printf(" %02X", msg.buf[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}   // wait for PC terminal

  // ---------- CAN1 ----------
  can1.begin();
  can1.setBaudRate(1000000);              // 1 Mbps
  can1.setMaxMB(16);                      // plenty for this demo
  can1.enableFIFO();                      // easier than individual mailboxes

  // ---------- CAN2 ----------
  can2.begin();
  can2.setBaudRate(1000000);
  can2.setMaxMB(16);
  can2.enableFIFO();

  Serial.println("CAN-loopback test running…");
}

void loop() {
  // ---- transmit on CAN1 ----
  CAN_message_t tx;
  tx.id  = 0x123;
  tx.len = 8;
  tx.buf[0] = (counter >> 24) & 0xFF;
  tx.buf[1] = (counter >> 16) & 0xFF;
  tx.buf[2] = (counter >>  8) & 0xFF;
  tx.buf[3] = (counter      ) & 0xFF;
  tx.buf[4] = 0xDE; tx.buf[5] = 0xAD; tx.buf[6] = 0xBE; tx.buf[7] = 0xEF;

  can1.write(tx);
  printFrame("TX", tx);

  // ---- look for it on CAN2 (non-blocking poll) ----
  CAN_message_t rx;
  uint32_t tStart = millis();
  bool gotIt = false;
  while (millis() - tStart < 50) {        // up to 50 ms wait
    if (can2.read(rx)) {
      printFrame("RX", rx);
      gotIt = true;
      break;
    }
  }
  if (!gotIt) Serial.println("RX timeout");

  counter++;
  delay(500);
}
