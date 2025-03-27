# Communication Protocol between ESP32 and Mobile Application

## Overview
This document outlines the communication protocol used for UDP communication between the ESP32 microcontroller and the mobile application. The protocol defines the structure of messages exchanged, including commands and responses.

## Communication Method
- **Protocol Type**: UDP (User Datagram Protocol)
- **Transport Layer**: The ESP32 acts as a WiFi access point, allowing mobile devices to connect directly to it.

## Message Structure
Each message sent between the ESP32 and the mobile application follows a specific format:

### Message Format
```
<Command>:<Parameter1>:<Parameter2>:<Checksum>
```

- **Command**: A string that specifies the action to be performed (e.g., "SET_SPEED", "GET_STATUS").
- **Parameter1**: The first parameter associated with the command (e.g., speed value).
- **Parameter2**: The second parameter, if applicable (e.g., motor number).
- **Checksum**: A simple checksum for error checking, calculated as the sum of ASCII values of the command and parameters.

### Example Messages
1. **Set Motor Speed**
   - **Message**: `SET_SPEED:100:1:123`
   - **Description**: Sets motor 1 to speed 100.
   
2. **Get Motor Status**
   - **Message**: `GET_STATUS:1:0`
   - **Description**: Requests the status of motor 1.

## Response Structure
Responses from the ESP32 to the mobile application will also follow a similar format:

### Response Format
```
<Status>:<Message>:<Checksum>
```

- **Status**: Indicates success or failure (e.g., "OK", "ERROR").
- **Message**: A descriptive message related to the status.
- **Checksum**: Similar to the request checksum.

### Example Responses
1. **Successful Speed Set**
   - **Response**: `OK:Motor speed set successfully:123`
   
2. **Error in Command**
   - **Response**: `ERROR:Invalid command received:456`

## Error Handling
In case of an error, the ESP32 will respond with an error message indicating the nature of the problem. The mobile application should handle these errors gracefully and provide feedback to the user.

## Conclusion
This communication protocol facilitates efficient and reliable interaction between the ESP32 and mobile application, enabling control and monitoring of connected devices.