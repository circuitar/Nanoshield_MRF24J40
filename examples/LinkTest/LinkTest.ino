/**
 * Tests the communication link between two MRF24J40 modules (base and node).
 *
 * You must program one module with the constant BASE = true and the other with BASE = false.
 * Make sure to set the PA_LNA constant to false if you are using a MRF24J40A module or
 * to true if you are using, MRF24J40B, MRF24J40C and MRF24J40D.
 *
 * Copyright (c) 2014 Circuitar
 * All rights reserved.
 *
 * This software is released under a BSD license. See the attached LICENSE file for details.
 */
#include <SPI.h>
#include <Nanoshield_MRF24J40.h>

// Defines if the module is the base or the node
#define BASE false

// Defines if PA/LNA control is activated
#define PA_LNA false

// Network addresses
word panId = 0xcafe;
word localAddr;
word destAddr;

// Transmission delay, higher causes less collisions
int txDelay = 20;

// Transmission period tracking
unsigned int interval = 2000;
unsigned long periodStart;

// Packet data
struct packet {
  int sequenceNumber;
  int packetsSent;
  int packetsReceived;
  int rssi;
  int lqi;
} rxPacket, txPacket;

// Packet counters
int packetsSent = 0;
int packetsReceived = 0;

// RSSI and LQI sums for the current period
unsigned long rssiSum = 0;
unsigned long lqiSum = 0;

Nanoshield_MRF24J40 mrf;

void setup() {
  word addr;
  
  Serial.begin(9600);
  
  delay(1000);

  // Initialize module
  mrf.init();
  
  // Delay to let RF stabilize
  delay(1);

  // Configure module as base or node
  if (BASE) {
    // Set to coordinator
    mrf.write_short(0x00, 0x0C);
    localAddr = 0x6001;
    destAddr = 0x6002;
  } else {
    localAddr = 0x6002;
    destAddr = 0x6001;
  }

  // Configure PA_LNA control
  mrf.set_palna(PA_LNA);
  
  // Try to write and then read back the PAN ID
  mrf.set_pan(panId);
  panId = mrf.get_pan();
  Serial.print("\nPAN ID: 0x");
  Serial.print(panId, HEX);
  Serial.print("\n");

  // Try to write and then read back the module address
  mrf.address16_write(localAddr);
  addr = mrf.address16_read();
  Serial.print("Address: 0x");
  Serial.print(addr, HEX);
  Serial.print("\n");

  // Print some register values for debugging
  byte reg;
  
  reg = mrf.read_short(0x00);
  Serial.print("RXMCR: 0x");
  Serial.print(reg, HEX);
  Serial.print("\n");

  reg = mrf.read_short(0x11);
  Serial.print("TXMCR: 0x");
  Serial.print(reg, HEX);
  Serial.print("\n");

  reg = mrf.read_short(0x10);
  Serial.print("ORDER: 0x");
  Serial.print(reg, HEX);
  Serial.print("\n");

  reg = mrf.read_short(0x0D);
  Serial.print("RXFLUSH: 0x");
  Serial.print(reg, HEX);
  Serial.print("\n");

  reg = mrf.read_long(0x200);
  Serial.print("RXCON0: 0x");
  Serial.print(reg, HEX);
  Serial.print("\n");

  reg = mrf.read_short(0x32);
  Serial.print("INTCON: 0x");
  Serial.print(reg, HEX);
  Serial.print("\n");
  
  // Initialize transmission period start time
  periodStart = millis();
}

void loop() {
  int lqi = 0;
  int rssi = 0;

  // Check if messages have been received
  if (receivePacket(&rxPacket, &lqi, &rssi)) {
    packetsReceived++;
    rssiSum += rssi;
    lqiSum += lqi;
  }

  // Send packet to destination
  sendPacket(destAddr, &txPacket);
  packetsSent++;

  // Check for end of transmission period
  if (millis() - periodStart >= interval) {
    // Update data to be transmitted
    txPacket.packetsSent = packetsSent;
    txPacket.packetsReceived = packetsReceived;
    if (packetsReceived > 0) {
      txPacket.rssi = rssiSum / packetsReceived;
      txPacket.lqi = lqiSum / packetsReceived;
    } else {
      txPacket.rssi = 0;
      txPacket.lqi = 0;
    }
    
    // Reset received packet if no packets received
    if (packetsReceived == 0) {
      rxPacket.packetsSent = 0;
      rxPacket.packetsReceived = 0;
      rxPacket.rssi = 0;
      rxPacket.lqi = 0;
    }
    
    // Print data
    Serial.println();
    Serial.println("Local:");
    printPacket(txPacket);
    Serial.println("Remote:");
    printPacket(rxPacket);

    // Restart reception period
    packetsSent = 0;
    packetsReceived = 0;
    rssiSum = 0;
    lqiSum = 0;
    periodStart = millis();
  }
  
  // Delay until next transmission
  if (txDelay > 0) {
    delay(txDelay);
  }
}

void printPacket(struct packet p) {
  Serial.print("  Packets sent: ");
  Serial.println(p.packetsSent);
  Serial.print("  Packets received: ");
  Serial.println(p.packetsReceived);
  Serial.print("  Packet loss: ");
  if (p.packetsSent > 0 && p.packetsReceived > 0) {
    Serial.print(100 * (p.packetsSent - p.packetsReceived) / p.packetsSent);
    Serial.println("%");
  } else {
    Serial.println("-");
  }
  Serial.print("  RSSI: ");
  Serial.println(p.rssi);
  Serial.print("  LQI: ");
  Serial.println(p.lqi);
}

void sendPacket(word addr, struct packet* p) {
  char buf[128];
  p->sequenceNumber++;
  sprintf(buf, "%d,%d,%d,%d,%d", p->sequenceNumber, p->packetsSent, p->packetsReceived, p->rssi, p->lqi);
  mrf.send16(addr, buf);
}

boolean receivePacket(struct packet* p, int* lqi, int* rssi) {
  char buf[128];
  int frameSize;
  int i;

  // Check RXIF in INTSTAT
  if (mrf.read_short(0x31) & 0x08) {
    // Disable interrupts and receiver
    noInterrupts();
    mrf.write_short(0x39, 0x04);
    
    // Packet received, get the number of bytes
    frameSize = mrf.read_long(0x300);
    
    // Copy the message bytes into the user buffer
    for (i = 0; i < (frameSize - 11); i++) {
      buf[i] = mrf.read_long(0x301 + 9 + i);
    }
    buf[i] = '\0';

    // Get link quality indicator    
    if (lqi) {
      *lqi = mrf.read_long(0x301 + 9 + i + 2);
    }

    // Get signal strength
    if (rssi) {
      *rssi = mrf.read_long(0x301 + 9 + i + 3);
    }
  
    // Flush the reception buffer, re-enable interrupts and receiver
    mrf.write_short(0x0D, 0x01);
    mrf.write_short(0x39, 0x00);
    interrupts();
    
    // Wait until RXIF is cleared (takes a while)
    while(mrf.read_short(0x31) & 0x08);
    
    p->sequenceNumber = atoi(strtok(buf, ","));
    p->packetsSent = atoi(strtok(NULL, ","));
    p->packetsReceived = atoi(strtok(NULL, ","));
    p->rssi = atoi(strtok(NULL, ","));
    p->lqi = atoi(strtok(NULL, ","));
  
    return true;
  }
  
  return false;
}

