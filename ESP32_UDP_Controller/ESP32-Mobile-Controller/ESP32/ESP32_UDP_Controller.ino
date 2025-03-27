#include <WiFi.h>
#include <WiFiUdp.h>
#include "config.h"

// Create a UDP instance
WiFiUDP udp;

// Buffer for incoming UDP packets
char incomingPacket[255];

// Local UDP port
const unsigned int localPort = 12345;  // Choose a port number

void setup() {
  Serial.begin(115200);
  
  // Set up WiFi in Access Point mode
  WiFi.softAP(ssid, password);
  
  Serial.println("Access Point started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start UDP
  udp.begin(localPort);
  Serial.printf("UDP listening on port %d\n", localPort);
}

void loop() {
  // Check for incoming UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read the packet into the buffer
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;  // Null-terminate the string
    }
    
    Serial.printf("Received packet: %s\n", incomingPacket);
    
    // Handle the incoming packet (you can add your own logic here)
    handleUdpMessage(incomingPacket);
  }
  
  delay(10);  // Small delay to avoid overwhelming the loop
}

void handleUdpMessage(const char* message) {
  // Process the incoming message
  // Add your logic here to control motors or respond to the mobile app
  Serial.printf("Processing message: %s\n", message);
}