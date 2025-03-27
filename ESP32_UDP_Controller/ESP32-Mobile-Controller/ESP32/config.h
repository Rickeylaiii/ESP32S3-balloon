#ifndef CONFIG_H
#define CONFIG_H

// WiFi credentials
const char* ssid = "Your_SSID";          // Replace with your WiFi SSID
const char* password = "Your_PASSWORD";   // Replace with your WiFi Password

// UDP configuration
const unsigned int localPort = 8888;      // Local port to listen on
const char* udpBroadcastAddress = "255.255.255.255"; // Broadcast address for UDP

// Other constants
const int maxMessageSize = 512;           // Maximum size of UDP message

#endif // CONFIG_H