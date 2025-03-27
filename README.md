


https://github.com/user-attachments/assets/f849bf14-467e-49ee-abb1-8e11647cb779


https://github.com/user-attachments/assets/3fcdc3fb-1d82-409b-9b4f-4b68670845e6

![ecbe92497b6dbc72a2f50104f1cd107](https://github.com/user-attachments/assets/3677b218-2745-499a-a7e9-7753df2ffe9f)
![cfed1c70533ec250ce0434732a0a4cd](https://github.com/user-attachments/assets/9a5839c5-e7bd-46ee-b46f-0fc174a1b950)


# MS5611
Arduino library for MS5611 temperature and pressure sensor.
## Description

The MS5611-01BA03 is a high resolution pressure (and temperature) sensor a.k.a GY-63.
The high resolution is made possible by oversampling many times.

The device address is 0x76 or 0x77 depending on the CSB/CSO pin.

This library only implements the I2C interface.

An experimental SPI version of the library can be found here 
- https://github.com/RobTillaart/MS5611_SPI

### Breakout GY-63

```cpp
//
//  BREAKOUT  MS5611  aka  GY63 - see datasheet
//
//  SPI    I2C
//              +--------+
//  VCC    VCC  | o      |
//  GND    GND  | o      |
//         SCL  | o      |
//  SDI    SDA  | o      |
//  CSO         | o      |
//  SDO         | o L    |   L = led
//          PS  | o    O |   O = opening  PS = protocol select
//              +--------+
//
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76
//  CS to GND  ==>  0x77
//
```

