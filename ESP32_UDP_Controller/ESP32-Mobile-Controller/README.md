# ESP32 Mobile Controller

This project enables control of an ESP32 device via a mobile application using UDP communication. The ESP32 operates in WiFi router mode, allowing mobile devices to connect directly to it.

## Project Structure

```
ESP32-Mobile-Controller
├── ESP32
│   ├── ESP32_UDP_Controller.ino       # Main program for ESP32, handles WiFi and UDP communication
│   ├── config.h                        # Contains WiFi configuration and constants
│   └── udp_handler.h                   # Defines functions and structures for handling UDP messages
├── MobileApp
│   ├── android
│   │   ├── app
│   │   │   ├── src
│   │   │   │   ├── main
│   │   │   │   │   ├── java
│   │   │   │   │   │   └── com
│   │   │   │   │   │       └── example
│   │   │   │   │   │           └── esp32controller
│   │   │   │   │   │               ├── MainActivity.java      # Main activity for Android app
│   │   │   │   │   │               ├── UdpManager.java       # Manages UDP communication
│   │   │   │   │   │               └── ControllerActivity.java # Activity for controlling the ESP32
│   │   │   │   │   └── res
│   │   │   │   │       ├── layout
│   │   │   │   │       │   ├── activity_main.xml            # Layout for main activity
│   │   │   │   │       │   └── activity_controller.xml      # Layout for controller activity
│   │   │   │   │       └── values
│   │   │   │   │           ├── strings.xml                   # String resources for the app
│   │   │   │   │           └── colors.xml                    # Color resources for the app
│   │   │   │   └── AndroidManifest.xml                       # Android app manifest
│   │   │   └── build.gradle                                    # Build configuration for Android module
│   │   └── build.gradle                                        # Build configuration for the entire Android project
│   └── ios
│       ├── ESP32Controller
│       │   ├── AppDelegate.swift                              # iOS app delegate for lifecycle management
│       │   ├── ViewController.swift                           # Main view controller for iOS app
│       │   ├── UdpManager.swift                               # Manages UDP communication for iOS app
│       │   └── Info.plist                                     # Configuration information for iOS app
│       └── ESP32Controller.xcodeproj                         # Xcode project file for iOS app
├── Protocol
│   └── communication_protocol.md                               # Describes the communication protocol between ESP32 and mobile app
└── README.md                                                  # Project documentation and instructions
```

## Features

- **ESP32 Control**: Control motors or other devices connected to the ESP32.
- **UDP Communication**: Fast and efficient communication between the mobile app and the ESP32.
- **WiFi Router Mode**: ESP32 acts as a WiFi access point for mobile devices.

## Getting Started

1. **ESP32 Setup**:
   - Upload the `ESP32_UDP_Controller.ino` sketch to your ESP32.
   - Modify the `config.h` file to set your desired WiFi credentials.

2. **Mobile Application**:
   - Build and run the Android or iOS application.
   - Ensure your mobile device is connected to the ESP32's WiFi network.

3. **Control**:
   - Use the mobile app to send commands to the ESP32 and control connected devices.

## Communication Protocol

Refer to `Protocol/communication_protocol.md` for details on the UDP communication protocol used between the ESP32 and the mobile application.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.