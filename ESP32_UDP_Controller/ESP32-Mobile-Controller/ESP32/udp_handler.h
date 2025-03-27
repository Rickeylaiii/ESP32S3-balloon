#ifndef UDP_HANDLER_H
#define UDP_HANDLER_H

#include <WiFi.h>
#include <WiFiUdp.h>

// Define the UDP port
#define UDP_PORT 12345

// Structure to hold the received UDP message
struct UdpMessage {
    char message[256];
};

// Function to initialize UDP communication
void initUdp();

// Function to handle incoming UDP messages
void handleUdpMessages();

// Function to send a UDP message
void sendUdpMessage(const char* message, const char* ipAddress);

#endif // UDP_HANDLER_H