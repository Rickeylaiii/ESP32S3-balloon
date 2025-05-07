#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <Wire.h>
#include <MS5611.h>  // Add MS5611 library
#include "pid.h"     // Add PID controller header file
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// PID altitude control related variables
PidController altitudeController(20.0, 0.1, 5.0, 50); // Kp, Ki, Kd, update interval

// WiFi credentials
const char* ssid = "Mi10S";     // CHANGE THIS
const char* password = "12345678";  // CHANGE THIS

// MS5611 sensor variables
int tempReadCounter = 0;
#define TEMP_READ_INTERVAL 100  // 每10次压力读取才读一次温度

MS5611 ms5611(0x77);  // Create MS5611 object, default address 0x77
float temperature = 0;  // Temperature in °C
float pressure = 0;     // Pressure in mBar
float altitude = 0;     // Altitude in meters
unsigned long lastSensorReadTime = 0;  // Last sensor read time
const unsigned long sensorReadInterval = 10;  // Read interval in milliseconds
bool readingTemp = true;  // For alternately reading temperature and pressure

// Add to the global variables section
// Altitude filter related variables
#define ALTITUDE_FILTER_SIZE 10  // Filter window size
float altitudeBuffer[ALTITUDE_FILTER_SIZE]; // Store recent altitude data
int altitudeBufferIndex = 0;    // Current buffer index
float filteredAltitude = 0;     // Filtered altitude
bool altitudeBufferFilled = false; // Whether buffer is filled

// Add to global variables section
// AP mode configuration
#define AP_SSID "ESP32-Motor-AP"
#define AP_PASSWORD "12345678"
#define CONFIG_PORTAL_TIMEOUT 180   // AP mode timeout in seconds
bool apMode = false;                // Whether in AP mode
unsigned long apStartTime = 0;      // AP mode start time
const byte DNS_PORT = 53;           // DNS server port
DNSServer dnsServer;                // DNS server object for capturing all DNS requests
Preferences preferences;            // For saving WiFi settings

// Add WiFi status tracking variables to global variables section
bool lastWifiConnected = false;
int reconnectAttempts = 0;
const int maxReconnectAttempts = 10;
unsigned long lastReconnectTime = 0;
const unsigned long reconnectInterval = 5000; // 5 seconds

// Motor pin definitions
const int motor1_in1 = 5;  // Motor 1 input 1
const int motor1_in2 = 6;  // Motor 1 input 2
const int motor2_in1 = 7;  // Motor 2 input 1
const int motor2_in2 = 15; // Motor 2 input 2
const int motor3_in1 = 16; // Motor 3 input 1
const int motor3_in2 = 17; // Motor 3 input 2

// LEDC PWM configuration
#define PWM_FREQUENCY 15000  // 15KHz
#define PWM_RESOLUTION 10    // 10-bit resolution (0-1023)

// 在全局变量区域添加
#define SERVO_FREQ 50  // 50Hz 舵机/电调标准频率 (20ms周期)
#define SERVO_RESOLUTION 12 // 使用16位分辨率以获得更高精度
#define SERVO_MIN_US 1000   // 1000微秒 - 最大正向速度
#define SERVO_MID_US 1500   // 1500微秒 - 停止
#define SERVO_MAX_US 2000   // 2000微秒 - 最大反向速度

// PWM channel assignments
#define MOTOR1_IN1_CHANNEL 0
#define MOTOR1_IN2_CHANNEL 1
#define MOTOR2_IN1_CHANNEL 2
#define MOTOR2_IN2_CHANNEL 3
// 修改电机3通道定义，只用一个通道
#define MOTOR3_PWM_CHANNEL 4  // 统一使用单通道

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// HTML webpage
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>ESP32 Motor Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 2.3rem;}
    p {font-size: 1.9rem;}
    body {max-width: 600px; margin:0px auto; padding-bottom: 25px;}
    .slider {width: 300px; height: 50px;}
    .button {background-color: #f44336; border: none; color: white; padding: 15px 32px; 
             text-align: center; font-size: 16px; margin: 4px 2px; cursor: pointer;}
    #joystick-container {
      position: relative;
      width: 300px;
      height: 300px;
      background: #f0f0f0;
      border: 2px solid #999;
      border-radius: 50%;
      margin: 0 auto;
      touch-action: none;
    }
    #joystick-thumb {
      position: absolute;
      width: 80px;
      height: 80px;
      background: #3498db;
      border-radius: 50%;
      top: 110px;
      left: 110px;
      cursor: pointer;
      user-select: none;
    }
    .motor-value {
      font-size: 1.5rem;
      margin: 10px;
    }
    .slider-container {
      display: flex;
      align-items: center;
      justify-content: center;
      margin: 10px auto;
    }
    .slider-value {
      font-size: 1.5rem;
      margin-left: 15px;
      min-width: 60px;
    }
    .control-container {
      display: flex;
      justify-content: center;
      align-items: center;
      margin: 20px auto;
      gap: 30px; /* spacing between joystick and slider */
    }
    
    .joystick-motor-container {
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    
    .vertical-slider-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      height: 300px;
      justify-content: center;
    }
    
    .vertical-slider {
      -webkit-appearance: slider-vertical;
      width: 50px;
      height: 250px;
      margin: 10px 0;
    }
    
    /* Firefox specific styles */
    @supports (-moz-appearance:none) {
      .vertical-slider {
        writing-mode: bt-lr; /* IE */
        -webkit-appearance: slider-vertical; /* WebKit */
        width: 50px;
        height: 250px;
      }
    }
  </style>
</head>
<body>
  <h2>ESP32 Motor Control</h2>
  
  <div class="control-container">
    <div class="joystick-motor-container">
      <p>Left Motor & Right Motor</p>
      <div id="joystick-container">
        <div id="joystick-thumb"></div>
      </div>
      <div class="motor-value">
        Left Motor: <span id="motor1Value">0</span>%
      </div>
      <div class="motor-value">
        Right Motor: <span id="motor2Value">0</span>%
      </div>
    </div>
    
    <!-- Motor status indicator in vertical slider container -->
    <div class="vertical-slider-container">
      <p>Motor 3</p>
      <div id="motor3-control-indicator" style="font-size: 0.8rem; margin-bottom: 5px; height: 20px;">Manual Control</div>
      <input type="range" min="-100" max="100" value="0" class="vertical-slider" id="motor3Slider" 
             orient="vertical" oninput="document.getElementById('motor3Value').innerHTML = this.value;" 
             onchange="updateMotor(3, this.value);">
      <div class="slider-value"><span id="motor3Value">0</span>%</div>
    </div>
  </div>
  
  <button class="button" onclick="stopAllMotors()">STOP ALL MOTORS</button>

  <!-- Add sensor data display area -->
  <div style="margin-top: 20px; border: 1px solid #ddd; border-radius: 10px; padding: 15px; background-color: #f9f9f9;">
    <h3 style="margin-top: 0;">Sensor Data</h3>
    <div style="display: flex; flex-wrap: wrap; justify-content: center;">
      <div style="margin: 10px 20px; text-align: center;">
        <div style="font-weight: bold; color: #2196F3;">Temperature</div>
        <div style="font-size: 1.4rem;"><span id="temperature">--</span>C</div>
      </div>
      <div style="margin: 10px 20px; text-align: center;">
        <div style="font-weight: bold; color: #4CAF50;">Pressure</div>
        <div style="font-size: 1.4rem;"><span id="pressure">--</span> hPa</div>
      </div>
      <div style="margin: 10px 20px; text-align: center;">
        <div style="font-weight: bold; color: #FF9800;">Altitude</div>
        <div style="font-size: 1.4rem;"><span id="altitude">--</span> m</div>
      </div>
      <div style="margin: 10px 20px; text-align: center;">
        <div style="font-weight: bold; color: #E91E63;">Vertical Speed</div>
        <div style="font-size: 1.4rem;"><span id="verticalSpeed">--</span> m/s</div>
      </div>
    </div>
  </div>

  <!-- Altitude control UI after sensor data section -->
  <div style="margin-top: 20px; border: 1px solid #ddd; border-radius: 10px; padding: 15px; background-color: #f9f9f9;">
    <h3 style="margin-top: 0;">Altitude Control</h3>
    
    <div style="display: flex; flex-wrap: wrap; justify-content: center; margin-bottom: 15px;">
      <div style="margin: 10px 20px; text-align: center;">
        <div style="font-weight: bold; color: #2196F3;">Initial Altitude</div>
        <div style="font-size: 1.2rem;"><span id="initialAltitude">--</span> m</div>
      </div>
      <div style="margin: 10px 20px; text-align: center;">
        <div style="font-weight: bold; color: #4CAF50;">Relative Altitude</div>
        <div style="font-size: 1.2rem;"><span id="relativeAltitude">--</span> m</div>
      </div>
      <div style="margin: 10px 20px; text-align: center;">
        <div style="font-weight: bold; color: #FF9800;">Target Altitude</div>
        <div style="font-size: 1.2rem;"><span id="targetAltitudeDisplay">--</span> m</div>
      </div>
    </div>
    
    <div style="margin: 15px 0;">
      <button onclick="calibrateAltitude()" style="background-color: #2196F3; color: white; border: none; padding: 10px 15px; border-radius: 5px; cursor: pointer; margin: 0 5px;">Calibrate Initial Altitude</button>
      <button onclick="toggleAutoAltitude()" id="autoControlButton" style="background-color: #FF9800; color: white; border: none; padding: 10px 15px; border-radius: 5px; cursor: pointer; margin: 0 5px;">Enable Auto Control</button>
    </div>
    
    <div style="margin: 15px 0; display: flex; justify-content: center; align-items: center;">
      <label style="margin-right: 10px;">Set Target Altitude:</label>
      <input type="number" id="targetAltitude" step="0.01" style="width: 80px; padding: 8px;" value="0">
      <span style="margin: 0 5px;">m</span>
      <button onclick="setTargetAltitude()" style="background-color: #4CAF50; color: white; border: none; padding: 8px 15px; border-radius: 5px; margin-left: 10px; cursor: pointer;">Set</button>
    </div>
  </div>

  <!-- PID parameter adjustment interface after altitude control UI -->
  <div style="margin-top: 20px; border: 1px solid #ddd; border-radius: 10px; padding: 15px; background-color: #f9f9f9;">
    <h3 style="margin-top: 0;">PID Parameter Adjustment</h3>
    
    <div style="display: flex; flex-wrap: wrap; justify-content: center; margin-bottom: 15px;">
      <div style="margin: 10px 20px; text-align: center;">
        <div style="font-weight: bold; color: #2196F3;">Kp</div>
        <input type="number" id="kp" step="0.01" min="0" style="width: 80px; padding: 8px;" value="20.0">
      </div>
      <div style="margin: 10px 20px; text-align: center;">
        <div style="font-weight: bold; color: #4CAF50;">Ki</div>
        <input type="number" id="ki" step="0.001" min="0" style="width: 80px; padding: 8px;" value="0.1">
      </div>
      <div style="margin: 10px 20px; text-align: center;">
        <div style="font-weight: bold; color: #FF9800;">Kd</div>
        <input type="number" id="kd" step="0.01" min="0" style="width: 80px; padding: 8px;" value="5.0">
      </div>
    </div>
    
    <div style="text-align: center; margin: 15px 0;">
      <button onclick="updatePidParams()" style="background-color: #4CAF50; color: white; border: none; padding: 10px 15px; border-radius: 5px; cursor: pointer;">Update PID Parameters</button>
    </div>
  </div>

  <script>
    // Joystick control
    const container = document.getElementById('joystick-container');
    const thumb = document.getElementById('joystick-thumb');
    let isDragging = false;
    let centerX, centerY, maxRadius;
    let currentX = 0;
    let currentY = 0;
    
    // Initialize joystick parameters
    function initJoystick() {
      const rect = container.getBoundingClientRect();
      centerX = rect.width / 2;
      centerY = rect.height / 2;
      maxRadius = Math.min(centerX, centerY) - 40; // Subtract joystick radius to prevent overflow
    }
    
    // Update joystick position and motor values
    function updateJoystickPosition(clientX, clientY) {
      const rect = container.getBoundingClientRect();
      let dx = clientX - rect.left - centerX;
      let dy = clientY - rect.top - centerY;
      
      // Calculate distance and angle
      let distance = Math.sqrt(dx * dx + dy * dy);
      
      // Limit within boundary
      if (distance > maxRadius) {
        dx = dx * maxRadius / distance;
        dy = dy * maxRadius / distance;
        distance = maxRadius;
      }
      
      // Update joystick position
      thumb.style.left = (centerX + dx - 40) + 'px';
      thumb.style.top = (centerY + dy - 40) + 'px';
      
      // Invert dy, making up positive and down negative
      dy = -dy;
      
      // Calculate differential steering left and right motor values
      // forwardSpeed controls forward/reverse, turnSpeed controls steering
      const forwardSpeed = dy / maxRadius * 100;
      const turnSpeed = dx / maxRadius * 100;
      
      // Calculate final speed for left and right motors
      // Forward(+y) + Right(+x) = Left motor faster
      // Forward(+y) + Left(-x) = Right motor faster
      let leftMotorSpeed = Math.round(forwardSpeed + turnSpeed);
      let rightMotorSpeed = Math.round(forwardSpeed - turnSpeed);
      
      // Limit within -100 to 100 range
      leftMotorSpeed = Math.max(-100, Math.min(100, leftMotorSpeed));
      rightMotorSpeed = Math.max(-100, Math.min(100, rightMotorSpeed));
      
      // Update display and send to backend
      document.getElementById("motor1Value").innerHTML = leftMotorSpeed;
      document.getElementById("motor2Value").innerHTML = rightMotorSpeed;
      
      // Only send when values change
      if (currentX !== leftMotorSpeed || currentY !== rightMotorSpeed) {
        currentX = leftMotorSpeed;
        currentY = rightMotorSpeed;
        updateMotor(1, leftMotorSpeed);
        updateMotor(2, rightMotorSpeed);
      }
    }
    
    // Mouse event handling
    container.addEventListener('mousedown', (e) => {
      isDragging = true;
      updateJoystickPosition(e.clientX, e.clientY);
    });
    
    document.addEventListener('mousemove', (e) => {
      if (isDragging) {
        updateJoystickPosition(e.clientX, e.clientY);
      }
    });
    
    document.addEventListener('mouseup', () => {
      if (isDragging) {
        isDragging = false;
        // Return to center position
        thumb.style.left = (centerX - 40) + 'px';
        thumb.style.top = (centerY - 40) + 'px';
        // Stop motors
        updateMotor(1, 0);
        updateMotor(2, 0);
        document.getElementById("motor1Value").innerHTML = 0;
        document.getElementById("motor2Value").innerHTML = 0;
        currentX = 0;
        currentY = 0;
      }
    });
    
    // Touch event handling
    container.addEventListener('touchstart', (e) => {
      e.preventDefault();
      isDragging = true;
      updateJoystickPosition(e.touches[0].clientX, e.touches[0].clientY);
    });
    
    document.addEventListener('touchmove', (e) => {
      if (isDragging) {
        e.preventDefault();
        updateJoystickPosition(e.touches[0].clientX, e.touches[0].clientY);
      }
    });
    
    document.addEventListener('touchend', () => {
      if (isDragging) {
        isDragging = false;
        // Return to center position
        thumb.style.left = (centerX - 40) + 'px';
        thumb.style.top = (centerY - 40) + 'px';
        // Stop motors
        updateMotor(1, 0);
        updateMotor(2, 0);
        document.getElementById("motor1Value").innerHTML = 0;
        document.getElementById("motor2Value").innerHTML = 0;
        currentX = 0;
        currentY = 0;
      }
    });
    
    // Initialize joystick
    window.addEventListener('load', initJoystick);
    window.addEventListener('resize', initJoystick);
    
    function updateMotor(motorNumber, value) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/motor?num="+motorNumber+"&val="+value, true);
      xhr.send();
    }
    
    function stopAllMotors() {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/stopAll", true);
      xhr.send();
      
      // Reset all controllers to center position
      document.getElementById("motor3Slider").value = 0;
      document.getElementById("motor3Value").innerHTML = 0;
      document.getElementById("motor1Value").innerHTML = 0;
      document.getElementById("motor2Value").innerHTML = 0;
      thumb.style.left = (centerX - 40) + 'px';
      thumb.style.top = (centerY - 40) + 'px';
      currentX = 0;
      currentY = 0;
      
      // Ensure status display updates
      setTimeout(updateAltitudeStatus, 200);
    }

    // Updated sensor data function
  function updateSensorData() {
    fetch('/sensorData')
      .then(response => {
        if (!response.ok) {
          throw new Error('Network response was not ok');
        }
        return response.json();
      })
      .then(data => {
        document.getElementById('temperature').textContent = data.temperature.toFixed(1);
        document.getElementById('pressure').textContent = data.pressure.toFixed(1);
        document.getElementById('altitude').textContent = data.filteredAltitude.toFixed(2);
        document.getElementById('verticalSpeed').textContent = data.verticalSpeed.toFixed(2);
      })
      .catch(error => {
        console.error('Error fetching sensor data:', error);
      });
  }

    // Start updating sensor data periodically after page loads
    window.addEventListener('load', function() {
      // 立即首次更新
      updateSensorData(); 
      updateAltitudeStatus();
      initJoystick();
      loadPidParams();
      
      // 增加更新频率以获取更及时的反馈
      setInterval(updateSensorData, 300);     // 更快的传感器数据更新
      setInterval(updateAltitudeStatus, 200); // 更快的状态更新
    });

    // Altitude control related functions
    function updateAltitudeStatus() {
      fetch('/altitudeStatus')
        .then(response => {
          if (!response.ok) {
            throw new Error('Network response was not ok');
          }
          return response.json();
        })
        .then(data => {
          document.getElementById('initialAltitude').textContent = data.initialAltitude.toFixed(2);
          document.getElementById('relativeAltitude').textContent = data.currentRelativeAltitude.toFixed(2);
          document.getElementById('targetAltitudeDisplay').textContent = data.targetAltitude.toFixed(2);
          
          // Only update value when input field doesn't have focus
          const targetInput = document.getElementById('targetAltitude');
          if (document.activeElement !== targetInput) {
            targetInput.value = data.targetAltitude.toFixed(1);
          }
          
          // Update auto control button status
          const autoButton = document.getElementById('autoControlButton');
          const motor3Slider = document.getElementById('motor3Slider');
          const sliderContainer = motor3Slider.parentElement;
          
          if (data.autoControl) {
            autoButton.textContent = "Disable Auto Control";
            autoButton.style.backgroundColor = "#f44336";
            
            // Disable vertical motor slider and show notification
            motor3Slider.disabled = true;
            
            // Add notification to slider container
            if (!document.getElementById('auto-control-notice')) {
              const notice = document.createElement('div');
              notice.id = 'auto-control-notice';
              notice.style.color = '#f44336';
              notice.style.fontWeight = 'bold';
              notice.style.fontSize = '0.9rem';
              notice.style.marginTop = '10px';
              notice.textContent = "Auto control active";
              sliderContainer.appendChild(notice);
            }
          } else {
            autoButton.textContent = "Enable Auto Control";
            autoButton.style.backgroundColor = "#FF9800";
            
            // Enable vertical motor slider and remove notification
            motor3Slider.disabled = false;
            
            // Correct way: first check if element exists
            const notice = document.getElementById('auto-control-notice');
            if (notice) {
              notice.remove();
            }
          }
          
          // If initial altitude not set, disable auto control button
          autoButton.disabled = !data.initialSet;

          // Update vertical slider control indicator
          const indicator = document.getElementById('motor3-control-indicator');
          if (data.autoControl) {
            indicator.textContent = "Automatic Control";
            indicator.style.color = "#f44336";
          } else {
            indicator.textContent = "Manual Control";
            indicator.style.color = "#4CAF50";
          }
        })
        .catch(error => console.error('Error getting altitude status:', error));
    }

    // Calibrate initial altitude
    function calibrateAltitude() {
      // 立即显示视觉反馈
      const calibrateButton = document.activeElement;
      const originalText = calibrateButton.textContent;
      const originalColor = calibrateButton.style.backgroundColor;
      
      calibrateButton.textContent = "Calibrating...";
      calibrateButton.style.backgroundColor = "#cccccc";
      calibrateButton.disabled = true;
      
      fetch('/calibrateAltitude')
        .then(response => response.text())
        .then(message => {
          // 操作成功的反馈
          calibrateButton.textContent = "Calibrated!";
          calibrateButton.style.backgroundColor = "#4CAF50";
          
          // 延迟恢复按钮状态
          setTimeout(() => {
            calibrateButton.textContent = originalText;
            calibrateButton.style.backgroundColor = originalColor;
            calibrateButton.disabled = false;
          }, 1500);
          
          // 更新状态显示
          updateAltitudeStatus();
        })
        .catch(error => {
          console.error('Error calibrating altitude:', error);
          // 操作失败的反馈
          calibrateButton.textContent = "Failed!";
          calibrateButton.style.backgroundColor = "#f44336";
          
          // 延迟恢复按钮状态
          setTimeout(() => {
            calibrateButton.textContent = originalText;
            calibrateButton.style.backgroundColor = originalColor;
            calibrateButton.disabled = false;
          }, 2000);
        });
    }

    // Set target altitude
    function setTargetAltitude() {
      const value = document.getElementById('targetAltitude').value;
      
      // Add button status indicator and disable to prevent multiple submissions
      const setButton = document.activeElement;
      const originalText = setButton.textContent;
      setButton.textContent = "Setting...";
      setButton.disabled = true;
      
      fetch(`/setTargetAltitude?value=${value}`)
        .then(response => {
          if (!response.ok) {
            throw new Error('Request failed: ' + response.status);
          }
          return response.text();
        })
        .then(message => {
          // Success notification
          console.log(message);
          setButton.style.backgroundColor = "#4CAF50";
          setButton.textContent = "Set";
          
          // Update status display
          updateAltitudeStatus();
          
          // Restore button state after 1 second
          setTimeout(() => {
            setButton.textContent = originalText;
            setButton.disabled = false;
            setButton.style.backgroundColor = "#4CAF50";
          }, 1000);
        })
        .catch(error => {
          console.error('Error setting target altitude:', error);
          setButton.textContent = "Failed";
          setButton.style.backgroundColor = "#f44336";
          
          // Restore button state after 3 seconds
          setTimeout(() => {
            setButton.textContent = originalText;
            setButton.disabled = false;
            setButton.style.backgroundColor = "#4CAF50";
          }, 3000);
        });
    }

    // Toggle automatic altitude control
    function toggleAutoAltitude() {
      // 获取当前按钮和状态显示元素
      const autoButton = document.getElementById('autoControlButton');
      const motor3Slider = document.getElementById('motor3Slider');
      const indicator = document.getElementById('motor3-control-indicator');
      const sliderContainer = motor3Slider.parentElement;
      
      // 立即更新UI以提供即时反馈
      const currentState = autoButton.textContent.includes("Disable");
      const newState = !currentState;
      
      if (newState) {
        // 启用自动控制 - 立即更新UI
        autoButton.textContent = "Disable Auto Control";
        autoButton.style.backgroundColor = "#f44336";
        motor3Slider.disabled = true;
        indicator.textContent = "Automatic Control";
        indicator.style.color = "#f44336";
        
        // 添加通知
        if (!document.getElementById('auto-control-notice')) {
          const notice = document.createElement('div');
          notice.id = 'auto-control-notice';
          notice.style.color = '#f44336';
          notice.style.fontWeight = 'bold';
          notice.style.fontSize = '0.9rem';
          notice.style.marginTop = '10px';
          notice.textContent = "Auto control active";
          sliderContainer.appendChild(notice);
        }
        
        // 重置滑块位置
        document.getElementById("motor3Slider").value = 0;
        document.getElementById("motor3Value").innerHTML = 0;
      } else {
        // 禁用自动控制 - 立即更新UI
        autoButton.textContent = "Enable Auto Control";
        autoButton.style.backgroundColor = "#FF9800";
        motor3Slider.disabled = false;
        indicator.textContent = "Manual Control";
        indicator.style.color = "#4CAF50";
        
        // 移除通知
        const notice = document.getElementById('auto-control-notice');
        if (notice) {
          notice.remove();
        }
      }
      
      // 添加该段缺失的代码
      fetch(`/autoAltitude?enable=${newState ? '1' : '0'}`)
        .then(response => response.text())
        .catch(error => {
          console.error('Error toggling automatic altitude control:', error);
          // 如果发生错误，恢复UI到之前状态
          toggleAutoAltitude(); // 简单的切换回去
        });
    }

    // Update PID parameters
    function updatePidParams() {
      const kp = document.getElementById('kp').value;
      const ki = document.getElementById('ki').value;
      const kd = document.getElementById('kd').value;
      
      fetch(`/setPidParams?kp=${kp}&ki=${ki}&kd=${kd}`)
        .then(response => response.json())
        .then(data => {
          console.log('PID parameters updated:', data);
          // Optional: Display confirmation message
          alert(`PID parameters updated: Kp=${data.kp}, Ki=${data.ki}, Kd=${data.kd}`);
        })
        .catch(error => console.error('Error updating PID parameters:', error));
    }

    // Modified loadPidParams function to display more precise values
    function loadPidParams() {
      fetch('/getPidParams')
        .then(response => response.json())
        .then(data => {
          document.getElementById('kp').value = data.kp.toFixed(2);
          document.getElementById('ki').value = data.ki.toFixed(3);
          document.getElementById('kd').value = data.kd.toFixed(2);
        })
        .catch(error => console.error('Error getting PID parameters:', error));
    }

    // 在index_html的脚本末尾添加辅助函数

    // 防止按钮重复点击的辅助函数
    function debounceButton(button, callback, delay = 500) {
      if (button.dataset.processing === 'true') return;
      
      button.dataset.processing = 'true';
      callback();
      
      setTimeout(() => {
        button.dataset.processing = 'false';
      }, delay);
    }

    // 修改按钮事件处理
    document.querySelectorAll('button').forEach(button => {
      const originalClick = button.onclick;
      if (originalClick) {
        button.onclick = function(event) {
          debounceButton(this, () => originalClick.call(this, event));
        };
      }
    });
  </script>
</body>
</html>
)rawliteral";

// 在index_html之后，setup()函数之前添加这段代码
// HTML for AP mode configuration page
const char config_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>ESP32 WiFi Configuration</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 1.8rem; color: #0066CC;}
    body {max-width: 600px; margin: 20px auto; padding: 20px; background-color: #f8f9fa;}
    .input-container {margin: 15px 0; text-align: left; padding: 0 20px;}
    label {display: block; margin-bottom: 5px; font-weight: bold;}
    input[type="text"], input[type="password"] {
      width: 100%; 
      padding: 10px; 
      box-sizing: border-box; 
      border: 1px solid #ddd; 
      border-radius: 4px; 
      margin-bottom: 10px;
    }
    button {
      background-color: #0066CC; 
      color: white; 
      border: none; 
      padding: 12px 20px; 
      border-radius: 4px;
      cursor: pointer;
    }
    button:hover {background-color: #0052a3;}
    .network-list {
      margin: 15px 0; 
      max-height: 200px; 
      overflow-y: auto; 
      border: 1px solid #ddd; 
      border-radius: 4px;
    }
    .network-item {
      padding: 10px; 
      border-bottom: 1px solid #eee; 
      cursor: pointer; 
      text-align: left;
    }
    .network-item:hover {background-color: #f0f0f0;}
    .network-name {font-weight: bold;}
    .network-strength {font-size: 0.8em; color: #666;}
    .loading {
      display: inline-block;
      width: 20px;
      height: 20px;
      border: 3px solid rgba(0,0,0,.3);
      border-radius: 50%;
      border-top-color: #0066CC;
      animation: spin 1s ease-in-out infinite;
    }
    @keyframes spin {
      to { transform: rotate(360deg); }
    }
  </style>
</head>
<body>
  <h2>ESP32 WiFi Configuration</h2>

  <div class="input-container">
    <button onclick="scanWifi()" id="scanButton">Scan WiFi Networks</button>
    <div id="scanning" style="display: none; margin-top: 10px;">
      <span class="loading"></span> Scanning networks...
    </div>
    <div id="networkList" class="network-list" style="display: none;"></div>
  </div>

  <form id="wifiForm" action="/save-wifi" method="post">
    <div class="input-container">
      <label for="ssid">WiFi Network:</label>
      <input type="text" id="ssid" name="ssid" placeholder="Enter SSID">
    </div>
    <div class="input-container">
      <label for="password">Password:</label>
      <input type="password" id="password" name="password" placeholder="Enter password">
    </div>
    <button type="submit">Save Configuration</button>
  </form>

  <script>
    function scanWifi() {
      document.getElementById('scanButton').disabled = true;
      document.getElementById('scanning').style.display = 'block';
      document.getElementById('networkList').style.display = 'none';
      
      fetch('/scan-wifi')
        .then(response => response.json())
        .then(networks => {
          if (networks.length === 0) {
            setTimeout(checkScanResults, 2000);
            return;
          }
          displayNetworks(networks);
        })
        .catch(error => {
          console.error('Error scanning networks:', error);
          document.getElementById('scanButton').disabled = false;
          document.getElementById('scanning').style.display = 'none';
        });
    }
    
    function checkScanResults() {
      fetch('/scan-wifi')
        .then(response => response.json())
        .then(networks => {
          if (networks.length === 0) {
            setTimeout(checkScanResults, 1000);
            return;
          }
          displayNetworks(networks);
        })
        .catch(error => {
          console.error('Error checking scan results:', error);
          document.getElementById('scanButton').disabled = false;
          document.getElementById('scanning').style.display = 'none';
        });
    }
    
    function displayNetworks(networks) {
      const networkList = document.getElementById('networkList');
      networkList.innerHTML = '';
      
      // Sort networks by signal strength
      networks.sort((a, b) => b.rssi - a.rssi);
      
      networks.forEach(network => {
        const strengthPercent = mapRSSIToPercentage(network.rssi);
        const item = document.createElement('div');
        item.className = 'network-item';
        item.innerHTML = `
          <div class="network-name">${network.ssid}</div>
          <div class="network-strength">Signal Strength: ${strengthPercent}%</div>
        `;
        item.onclick = function() {
          document.getElementById('ssid').value = network.ssid;
          document.getElementById('password').focus();
        };
        networkList.appendChild(item);
      });
      
      document.getElementById('scanButton').disabled = false;
      document.getElementById('scanning').style.display = 'none';
      networkList.style.display = 'block';
    }
    
    function mapRSSIToPercentage(rssi) {
      // RSSI typically ranges from -100 (weak) to -30 (strong)
      if (rssi >= -30) return 100;
      if (rssi <= -100) return 0;
      return Math.round(((rssi + 100) / 70) * 100);
    }
    
    document.getElementById('wifiForm').addEventListener('submit', function(event) {
      const ssid = document.getElementById('ssid').value.trim();
      const password = document.getElementById('password').value;
      
      if (!ssid) {
        alert('Please enter a WiFi network name (SSID)');
        event.preventDefault();
        return false;
      }
      
      return true;
    });
    
    // Scan WiFi networks when the page loads
    window.onload = function() {
      setTimeout(scanWifi, 500);
    };
  </script>
</body>
</html>
)rawliteral";

const char* PID_NAMESPACE = "pid_params";

// ==== 任务句柄 ====
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t pidTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t wifiTaskHandle = NULL;

// ==== 队列句柄 ====
QueueHandle_t altitudeQueue = NULL;      // 高度数据队列
QueueHandle_t motorCommandQueue = NULL;  // 电机控制命令队列
QueueHandle_t webCommandQueue = NULL;    // Web控制命令队列

// ==== 信号量句柄 ====
SemaphoreHandle_t sensorDataMutex = NULL;
SemaphoreHandle_t motorStateMutex = NULL;
SemaphoreHandle_t wifiStateMutex = NULL;

// 传感器数据结构
typedef struct {
    float temperature;          // 温度 (°C)
    float pressure;             // 压力 (mBar)
    float rawAltitude;          // 原始高度 (m)
    float filteredAltitude;     // 滤波后高度 (m)
    float verticalSpeed;        // 垂直速度 (m/s)
    unsigned long lastReadTime; // 最后读取时间 (ms)
} SensorData_t;

// 电机控制命令结构
typedef struct {
    int motorNum;   // 电机编号 (1-3)
    int value;      // 控制值 (-100到+100)
    bool fromAuto;  // 是否来自自动控制
} MotorCommand_t;

// 电机状态结构 
typedef struct {
    int motor1Speed;          // 左电机速度(-100到+100)
    int motor2Speed;          // 右电机速度(-100到+100)
    int motor3Speed;          // 升降电机速度(-100到+100)
    bool autoControlActive;   // 自动高度控制是否启用
} MotorState_t;

// 全局共享数据实例
SensorData_t g_sensorData;
MotorState_t g_motorState;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nESP32 FreeRTOS 气球控制系统启动");
    
    // 初始化共享数据结构
    memset(&g_sensorData, 0, sizeof(g_sensorData));
    memset(&g_motorState, 0, sizeof(g_motorState));
    
    // 创建互斥锁
    sensorDataMutex = xSemaphoreCreateMutex();
    motorStateMutex = xSemaphoreCreateMutex();
    wifiStateMutex = xSemaphoreCreateMutex();
    
    // 创建队列
    altitudeQueue = xQueueCreate(10, sizeof(float)); // 储存10个高度值
    motorCommandQueue = xQueueCreate(20, sizeof(MotorCommand_t)); // 储存20个电机命令
    webCommandQueue = xQueueCreate(5, sizeof(uint8_t)); // 储存5个web命令
    
    // 初始化硬件
    initHardware();
    
    // 创建任务
    static StaticTask_t sensorTaskBuffer;
    static StackType_t sensorTaskStack[4096];
    sensorTaskHandle = xTaskCreateStaticPinnedToCore(
        sensorTask, "SensorTask", 4096, NULL, 3,
        sensorTaskStack, &sensorTaskBuffer, 1
    );
    
    xTaskCreatePinnedToCore(
        pidTask,                // 任务函数
        "PidTask",              // 任务名称
        4096,                   // 堆栈大小
        NULL,                   // 任务参数
        2,                      // 任务优先级
        &pidTaskHandle,         // 任务句柄
        1                       // 核心
    );
    
    xTaskCreatePinnedToCore(
        motorTask,              // 任务函数
        "MotorTask",            // 任务名称
        2048,                   // 堆栈大小
        NULL,                   // 任务参数
        4,                      // 任务优先级(高优先级以确保电机命令快速执行)
        &motorTaskHandle,       // 任务句柄
        1                       // 核心
    );
    
    xTaskCreatePinnedToCore(
        wifiTask,               // 任务函数
        "WiFiTask",             // 任务名称
        4096,                   // 堆栈大小
        NULL,                   // 任务参数
        1,                      // 任务优先级(低优先级)
        &wifiTaskHandle,        // 任务句柄
        0                       // 核心
    );
    
    // 不需要创建Web服务器任务，因为AsyncWebServer是事件驱动的
    setupWebServer();
    
    // // 创建系统状态任务
    // xTaskCreatePinnedToCore(
    //     statsTask,              // 任务函数
    //     "StatsTask",            // 任务名称
    //     2048,                   // 堆栈大小
    //     NULL,                   // 任务参数
    //     1,                      // 任务优先级
    //     NULL,                   // 任务句柄
    //     0                       // 核心
    // );
    
    Serial.println("所有任务已启动");
}

// 硬件初始化函数
void initHardware() {
    // I2C初始化
    Wire.begin();
    delay(100);
    
    // MS5611传感器初始化
    if (ms5611.begin()) {
        Serial.println("MS5611传感器初始化成功");
        ms5611.setOversampling(OSR_ULTRA_HIGH);
    } else {
        Serial.println("MS5611传感器初始化失败！检查接线");
    }
    
    // 扫描I2C设备
    scanI2C();
    
    // 电机PWM初始化
    initMotors();
    
    // 加载PID参数
    loadPidParameters();
    
    // WiFi初始化
    initWiFi();
}

void loop() {
    // 主循环留空，所有工作在任务中完成
    // vTaskDelay(pdMS_TO_TICKS(1000));
}

// Add servo pulse width mapping function
uint32_t mapMotorValueToPulseWidth(int value) {
  // Map values from -100 to +100 to 1000us to 2000us
  // Note: -100 corresponds to 1000us (max forward), 0 corresponds to 1500us (stop), +100 corresponds to 2000us (max reverse)
  const int deadband = 7;
  if (abs(value) <= deadband) {
      // Deadband: 0 to 7 maps to 1500us (stop)
    value = 0;
  }

  uint32_t pulseWidth;
  
  if (value < 0) {
      // -100 to 0 maps to 1000us to 1500us (forward speed)
      pulseWidth = map(value, -100, 0, SERVO_MIN_US, SERVO_MID_US);
  } else {
      // 0 to +100 maps to 1500us to 2000us (reverse speed)
      pulseWidth = map(value, 0, 100, SERVO_MID_US, SERVO_MAX_US);
  }
  
  // Convert microseconds to count value (based on PWM resolution and frequency)
  // For 50Hz PWM, the period is 20ms (20000us)
  // For 16-bit resolution, the maximum count value is 65535
  uint32_t dutyCycle = (pulseWidth * 4096) / 20000;
  // // 对于10位分辨率(0-1023)
  // uint32_t dutyCycle = (pulseWidth * 1024) / 20000;

  
  Serial.printf("Motor value: %d, Pulse width: %d us, Duty cycle count: %d\n", value, pulseWidth, dutyCycle);
  return dutyCycle;
}
/**
 * Control motor direction and speed
 * @param motorNum - Motor number (1-3)
 * @param forward - true for forward, false for backward
 * @param speed - PWM value (0-1023)
 */
void setMotor(int motorNum, bool forward, uint16_t speed) {
  // Motors 1 and 2 use H-bridge driver method
  if (motorNum == 1 || motorNum == 2) {
      int in1_channel, in2_channel;
      
      // Select the correct PWM channels
      if (motorNum == 1) {
          in1_channel = MOTOR1_IN1_CHANNEL;
          in2_channel = MOTOR1_IN2_CHANNEL;
      } else { // motorNum == 2
          in1_channel = MOTOR2_IN1_CHANNEL;
          in2_channel = MOTOR2_IN2_CHANNEL;
      }
      
      if (forward) {
          // Forward: IN1=PWM, IN2=0
          ledcWrite(in1_channel, speed);
          ledcWrite(in2_channel, 0);
      } else {
          // Reverse: IN1=0, IN2=PWM
          ledcWrite(in1_channel, 0);
          ledcWrite(in2_channel, speed);
      }
      
      Serial.printf("Motor %d set to %s, speed %d\n", 
              motorNum, forward ? "forward" : "reverse", speed);
  }
  // Motor 3 uses BEC driver
  else if (motorNum == 3) {
      // Note: This function doesn't directly use forward and speed parameters
      // Motor 3 control logic has been moved to dedicated code in motorTask
      Serial.printf("Warning: Motor 3 uses BEC driver, should be handled through motorTask\n");
  }
}

/**
 * Stop motor (coast to stop)
 * @param motorNum - Motor number (1-3)
 */
void stopMotor(int motorNum) {
  if (motorNum == 1 || motorNum == 2) {
      int in1_channel, in2_channel;
      
      switch(motorNum) {
          case 1:
              in1_channel = MOTOR1_IN1_CHANNEL;
              in2_channel = MOTOR1_IN2_CHANNEL;
              break;
          case 2:
              in1_channel = MOTOR2_IN1_CHANNEL;
              in2_channel = MOTOR2_IN2_CHANNEL;
              break;
          default:
              return; // Don't process motor 3
      }
      
      // Both inputs LOW = coast to stop
      ledcWrite(in1_channel, 0);
      ledcWrite(in2_channel, 0);
  }
  else if (motorNum == 3) {
      // Motor 3 stop = 1500us pulse width
      uint32_t stopPulse = (SERVO_MID_US * 4096) / 20000;
      ledcWrite(MOTOR3_PWM_CHANNEL, stopPulse);
  }
  
  Serial.printf("Motor %d stopped\n", motorNum);
}

// Start AP mode
void startAPMode() {
  apMode = true;
  apStartTime = millis();
  
  // Stop STA mode connection attempts
  WiFi.disconnect();
  
  // Configure AP
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  Serial.println("\n========== Entering AP Configuration Mode ==========");
  Serial.print("AP Name: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Please connect to this WiFi using your phone or computer and visit http://192.168.4.1 to configure");
  Serial.println("=====================================");

  // Configure DNS server (capture all domain requests)
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  
  // Configure regular web service (original code)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  // Add routes for configuration mode
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", config_html);
  });

  // Handle WiFi scan requests
  server.on("/scan-wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "[";
    int n = WiFi.scanComplete();
    
    if(n == -2){
      // Trigger a WiFi scan
      WiFi.scanNetworks(true);
      request->send(200, "application/json", "[]");
      return;
    }
    
    // Scan complete, build JSON response
    for (int i = 0; i < n; ++i) {
      if (i > 0) json += ",";
      json += "{\"ssid\":\"";
      json += WiFi.SSID(i);
      json += "\",\"rssi\":";
      json += WiFi.RSSI(i);
      json += "}";
    }
    json += "]";
    
    // Delete scan results and start new scan
    WiFi.scanDelete();
    if(WiFi.scanComplete() == -2){
      WiFi.scanNetworks(true);
    }
    
    request->send(200, "application/json", json);
  });

  // Handle saving WiFi settings request
  server.on("/save-wifi", HTTP_POST, [](AsyncWebServerRequest *request) {
    String ssid, password;
    if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
      ssid = request->getParam("ssid", true)->value();
      password = request->getParam("password", true)->value();
      
      // Save WiFi settings
      preferences.begin("wifi", false);
      preferences.putString("ssid", ssid);
      preferences.putString("password", password);
      preferences.end();
      
      request->send(200, "text/plain", "WiFi configuration successful! Device will restart in 10 seconds...");
      
      // Delay before restarting ESP32
      delay(500);  // Ensure response is sent
      
      // Use a timer to restart after 10 seconds so the user can see the success message
      static bool restartScheduled = false;
      if(!restartScheduled) {
        restartScheduled = true;
        Serial.println("WiFi settings saved, restarting in 10 seconds...");
        Serial.print("New SSID: ");
        Serial.println(ssid);
        delay(10000); // Wait for 10 seconds
        ESP.restart();
      }
    } else {
      request->send(400, "text/plain", "Please provide both SSID and password");
    }
  });
}

// Handle DNS requests in AP mode
void handleDNS() {
  dnsServer.processNextRequest();
}

// Check for configuration mode timeout
bool checkAPTimeout() {
  if(apMode && millis() - apStartTime > CONFIG_PORTAL_TIMEOUT * 1000) {
    Serial.println("Configuration mode timeout, restarting device...");
    delay(500);
    ESP.restart();
    return true;
  }
  return false;
}

// Try to restore WiFi settings from storage
bool loadSavedWiFiSettings() {
  preferences.begin("wifi", true);
  String saved_ssid = preferences.getString("ssid", "");
  String saved_password = preferences.getString("password", "");
  preferences.end();
  
  if(saved_ssid.length() > 0) {
    Serial.println("Found saved WiFi settings");
    Serial.print("SSID: ");
    Serial.println(saved_ssid);
    Serial.println("Attempting to connect...");
    
    WiFi.begin(saved_ssid.c_str(), saved_password.c_str());
    return true;
  }
  
  return false;
}

void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    
    bool hasStoredCredentials = loadSavedWiFiSettings();
    
    if (!hasStoredCredentials) {
        Serial.println("使用默认WiFi设置");
        WiFi.begin(ssid, password);
    }
    
    // 不在此处等待WiFi连接，由WiFi任务处理
    Serial.println("WiFi连接过程已启动，等待连接结果...");
}

// Read MS5611 sensor data
void readSensorData() {
  tempReadCounter++;
  // Then read pressure and calculate altitude
  if (tempReadCounter < TEMP_READ_INTERVAL) {
    if (ms5611.read() == MS5611_READ_OK) {
      pressure = ms5611.getPressure();
      // Calculate raw altitude
      float rawAltitude = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));
      
      // Pass raw altitude to filter
      updateAltitudeFilter(rawAltitude);
      
      // Update global altitude variable with filtered altitude
      altitude = filteredAltitude;
      
      // Serial.print("Pressure: ");
      // Serial.print(pressure);
      // Serial.println(" mBar");
      Serial.print("Raw Altitude: ");
      Serial.print(rawAltitude);
      Serial.println(" m");
      Serial.print("Filtered Altitude: ");
      Serial.print(altitude);
      Serial.println(" m");
    } else {
      Serial.println("Failed to read pressure");
    }
  }else {
    // 偶尔读取温度
    tempReadCounter = 0;
    if (ms5611.read() == MS5611_READ_OK) {
      temperature = ms5611.getTemperature();
    }
  }
  
}

// Scan for devices on I2C bus
void scanI2C() {
  byte error, address;
  int deviceCount = 0;
  
  Serial.println("Scanning I2C bus...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found! Check wiring.");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" device(s)");
  }
}

// Update altitude filter
void updateAltitudeFilter(float newAltitude) {
  const float alpha = 0.2; // 平滑因子，值越小滤波效果越强 (0.1~0.3)
  static float lastFilteredAlt = 0;
  
  // 初始化处理
  if (lastFilteredAlt == 0) {
    lastFilteredAlt = newAltitude;
    filteredAltitude = newAltitude;
  } else {
    // 应用指数加权移动平均公式
    filteredAltitude = alpha * newAltitude + (1 - alpha) * lastFilteredAlt;
    lastFilteredAlt = filteredAltitude;
  }
  
  // 更新缓冲区状态
  altitudeBuffer[altitudeBufferIndex] = newAltitude;
  altitudeBufferIndex = (altitudeBufferIndex + 1) % ALTITUDE_FILTER_SIZE;
  if (altitudeBufferIndex == 0) {
    altitudeBufferFilled = true;
  }
}

// PID控制任务
void pidTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz控制率
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 本地变量
    float currentAltitude = 0;
    int pidOutput = 0;
    int lastPidOutput = 0;
    bool motorStopped = true;
    unsigned long stableZeroTime = 0;
    MotorCommand_t motorCmd;
    bool autoControlActive = false;
    
    // 任务循环
    while (1) {
        // 精确的周期控制
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // Serial.println("PID控制任务");
        // 检查自动控制是否激活
        if (xSemaphoreTake(motorStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            autoControlActive = g_motorState.autoControlActive;
            xSemaphoreGive(motorStateMutex);
        }
        
        // 如果自动控制激活，从队列获取高度数据并计算PID输出
        if (autoControlActive && altitudeController.isActive()) {
            // 非阻塞方式接收最新高度
            if (xQueueReceive(altitudeQueue, &currentAltitude, 0) == pdTRUE) {
                // 计算PID输出
                pidOutput = altitudeController.compute(currentAltitude);
                // // ========== 在这里添加速度限制 ==========
                // const float maxRateOfChange = 5.0; // 每个控制周期最大变化率(5%)
                // // 限制输出变化速率
                // pidOutput = constrain(pidOutput, 
                //                       lastPidOutput - maxRateOfChange, 
                //                       lastPidOutput + maxRateOfChange);
                // 死区控制逻辑
                const int MIN_OUTPUT_THRESHOLD = 7;
                
                if (abs(pidOutput) <= MIN_OUTPUT_THRESHOLD) {
                    if (!motorStopped) {
                        if (stableZeroTime == 0) {
                            stableZeroTime = millis();
                        } else if (millis() - stableZeroTime > 100) {
                            pidOutput = 0;
                            motorStopped = true;
                            stableZeroTime = 0;
                        } else {
                            // 平滑过渡
                            pidOutput = (lastPidOutput > 0) ? MIN_OUTPUT_THRESHOLD : 
                                        (lastPidOutput < 0) ? -MIN_OUTPUT_THRESHOLD : 0;
                        }
                    } else {
                        pidOutput = 0;
                    }
                } else {
                    motorStopped = false;
                    stableZeroTime = 0;
                }
                
                lastPidOutput = pidOutput;
                
                // 发送电机控制命令
                motorCmd.motorNum = 3; // 垂直控制电机
                motorCmd.value = pidOutput;
                motorCmd.fromAuto = true;
                xQueueSend(motorCommandQueue, &motorCmd, 0);
                
                // 调试输出
                static uint32_t lastDebugTime = 0;
                if (millis() - lastDebugTime > 1000) {
                    Serial.printf("PID控制: 当前高度=%.2f m, 目标=%.2f m, 输出=%d%%\n",
                        currentAltitude, altitudeController.getTarget(), pidOutput);
                    lastDebugTime = millis();
                }
            }
        } else {
            // 如果自动控制没有激活，清空高度队列
            float dummy;
            while (xQueueReceive(altitudeQueue, &dummy, 0) == pdTRUE);
        }
    }
}

// Motor control task
void motorTask(void *pvParameters) {
  MotorCommand_t cmd;
  
  // Task loop
  while (1) {
      // Wait for motor control commands
      if (xQueueReceive(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
          int motorNum = cmd.motorNum;
          int value = cmd.value;
          
          // If command is from auto control but auto control is not active, ignore the command
          bool shouldExecute = true;
          if (cmd.fromAuto) {
              if (xSemaphoreTake(motorStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                  shouldExecute = g_motorState.autoControlActive;
                  xSemaphoreGive(motorStateMutex);
              }
          }
          
          if (shouldExecute) {
              // Update motor state
              if (xSemaphoreTake(motorStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                  switch (motorNum) {
                      case 1: g_motorState.motor1Speed = value; break;
                      case 2: g_motorState.motor2Speed = value; break;
                      case 3: g_motorState.motor3Speed = value; break;
                  }
                  xSemaphoreGive(motorStateMutex);
              }
              
              // Control motors
              if (motorNum == 1 || motorNum == 2) {
                  // Motors 1 and 2 maintain the original H-bridge control method
                  if (value > 0) {
                      // Forward
                      uint16_t speed = map(value, 0, 100, 0, 1023);
                      setMotor(motorNum, true, speed);
                  } else if (value < 0) {
                      // Reverse
                      uint16_t speed = map(abs(value), 0, 100, 0, 1023);
                      setMotor(motorNum, false, speed);
                  } else {
                      // Stop
                      stopMotor(motorNum);
                  }
              }
              else if (motorNum == 3) {
                  // Motor 3 uses BEC control method
                  // Map values from -100 to +100 to PWM signals from 1000us to 2000us
                  uint32_t dutyCycle = mapMotorValueToPulseWidth(value);
                  ledcWrite(MOTOR3_PWM_CHANNEL, dutyCycle);
                  
                  // Output debug info
                  if (value == 0) {
                      Serial.println("Motor 3 stopped (1500us)");
                  } else {
                      Serial.printf("Motor 3 set to %d%% (%s direction)\n", 
                                    abs(value), value < 0 ? "forward" : "reverse");
                  }
              }
          }
      }
      
      // Small delay to avoid excessive CPU consumption
      vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// 修改WiFi任务中的重连逻辑
void wifiTask(void *pvParameters) {
    bool lastWifiConnected = false;
    int reconnectAttempts = 0;
    unsigned long lastReconnectTime = 0;
    const int maxReconnectAttempts = 20;    // 增加最大重连次数
    const unsigned long reconnectInterval = 10000; // 增加重连间隔到10秒
    
    // 任务循环
    while (1) {
        // 检查当前WiFi状态
        bool currentlyConnected = (WiFi.status() == WL_CONNECTED);
        
        // 如果WiFi状态发生变化
        if (lastWifiConnected != currentlyConnected) {
            if (lastWifiConnected && !currentlyConnected) {
                // WiFi断开连接，停止所有电机
                Serial.println("WiFi连接丢失 - 安全停止所有电机");
                
                MotorCommand_t stopCmd;
                for (int i = 1; i <= 3; i++) {
                    stopCmd.motorNum = i;
                    stopCmd.value = 0;
                    stopCmd.fromAuto = false;
                    xQueueSend(motorCommandQueue, &stopCmd, 0);
                }
                
                // 重置重连计数器
                reconnectAttempts = 0;
                lastReconnectTime = millis();
            } else if (!lastWifiConnected && currentlyConnected) {
                // WiFi重新连接
                Serial.println("WiFi重新连接成功");
                Serial.print("新IP地址: ");
                Serial.println(WiFi.localIP());
                reconnectAttempts = 0;
            }
            
            // 更新上一个状态
            lastWifiConnected = currentlyConnected;
        }
        
        // 尝试重连WiFi
        if (!currentlyConnected && reconnectAttempts < maxReconnectAttempts) {
            if (millis() - lastReconnectTime > reconnectInterval) {
                reconnectAttempts++;
                Serial.printf("WiFi断开。尝试重新连接... (尝试 %d/%d)\n", 
                          reconnectAttempts, maxReconnectAttempts);
                WiFi.disconnect();
                vTaskDelay(pdMS_TO_TICKS(100));
                WiFi.begin();  // 使用保存的凭据自动重连
                lastReconnectTime = millis();
            }
        }
        // 达到最大重连次数后切换到AP模式
        else if (!currentlyConnected && reconnectAttempts >= maxReconnectAttempts) {
            Serial.println("达到最大重连尝试次数。切换到AP模式...");
            // 启动AP模式
            startAPMode();
            vTaskDelete(NULL); // 终止自身任务
        }
        
        // 处理AP模式下的DNS请求
        if (apMode) {
            handleDNS();
            
            // 检查AP模式超时
            if (checkAPTimeout()) {
                vTaskDelete(NULL); // 超时后终止任务
            }
        }
        
        // 任务延时
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒检查一次
    }
}

// 初始化电机PWM控制
void initMotors() {
    // 设置电机引脚为输出模式
    pinMode(motor1_in1, OUTPUT);
    pinMode(motor1_in2, OUTPUT);
    pinMode(motor2_in1, OUTPUT);
    pinMode(motor2_in2, OUTPUT);
    pinMode(motor3_in1, OUTPUT);
    
    // 配置LEDC PWM通道
    ledcSetup(MOTOR1_IN1_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(MOTOR1_IN2_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(MOTOR2_IN1_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(MOTOR2_IN2_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    // 为电机3配置舵机PWM信号（50Hz，16位分辨率）
    ledcSetup(MOTOR3_PWM_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
    
    
    // 将GPIO引脚连接到PWM通道
    ledcAttachPin(motor1_in1, MOTOR1_IN1_CHANNEL);
    ledcAttachPin(motor1_in2, MOTOR1_IN2_CHANNEL);
    ledcAttachPin(motor2_in1, MOTOR2_IN1_CHANNEL);
    ledcAttachPin(motor2_in2, MOTOR2_IN2_CHANNEL);
    ledcAttachPin(motor3_in1, MOTOR3_PWM_CHANNEL);
  
    
    // 初始化所有电机为停止状态
    stopMotor(1);
    stopMotor(2);
    stopMotor(3);
    
    Serial.println("电机PWM初始化完成");
}

// 设置Web服务器
void setupWebServer() {
    // 基本路由 - 主页
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", index_html);
    });
    
    // 电机控制端点
    server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *request) {
        String motorNum;
        String motorVal;
        
        if (request->hasParam("num") && request->hasParam("val")) {
            motorNum = request->getParam("num")->value();
            motorVal = request->getParam("val")->value();
            
            int num = motorNum.toInt();
            int val = motorVal.toInt();
            
            // 检查电机3是否处于自动控制模式
            bool autoActive = false;
            if (xSemaphoreTake(motorStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                autoActive = g_motorState.autoControlActive;
                xSemaphoreGive(motorStateMutex);
            }
            
            if (num == 3 && autoActive) {
                request->send(200, "text/plain", "电机3处于自动控制中");
                return;
            }
            
            // 发送电机控制命令
            MotorCommand_t cmd;
            cmd.motorNum = num;
            cmd.value = val;
            cmd.fromAuto = false;
            
            if (xQueueSend(motorCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
                request->send(503, "text/plain", "命令队列已满");
                return;
            }
            
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "缺少参数");
        }
    });
    
    // 停止所有电机端点
    server.on("/stopAll", HTTP_GET, [](AsyncWebServerRequest *request) {
        // 发送停止命令给所有电机
        MotorCommand_t stopCmd;
        for (int i = 1; i <= 3; i++) {
            stopCmd.motorNum = i;
            stopCmd.value = 0;
            stopCmd.fromAuto = false;
            xQueueSend(motorCommandQueue, &stopCmd, 0);
        }
        
        // 如果自动控制处于激活状态，禁用它
        if (xSemaphoreTake(motorStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_motorState.autoControlActive = false;
            xSemaphoreGive(motorStateMutex);
        }
        
        request->send(200, "text/plain", "所有电机已停止");
    });
    
    // 传感器数据端点
    server.on("/sensorData", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          String json = "{\"temperature\":";
          json += g_sensorData.temperature;
          json += ",\"pressure\":";
          json += g_sensorData.pressure;
          json += ",\"rawAltitude\":";
          json += g_sensorData.rawAltitude;
          json += ",\"filteredAltitude\":";
          json += g_sensorData.filteredAltitude;
          json += ",\"verticalSpeed\":";
          json += g_sensorData.verticalSpeed;
          json += "}";
          xSemaphoreGive(sensorDataMutex);
          request->send(200, "application/json", json);
      } else {
          request->send(503, "text/plain", "传感器数据暂时不可用");
      }
  });
    
    // PID控制相关端点
    // 获取高度控制状态，完整实现
    server.on("/altitudeStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{\"initialAltitude\":";
        json += altitudeController.getInitialValue();
        json += ",\"initialSet\":";
        json += (altitudeController.getInitialValue() != 0 || altitudeController.isActive()) ? "true" : "false";
        json += ",\"targetAltitude\":";
        json += altitudeController.getTarget();
        json += ",\"currentRelativeAltitude\":";
        
        // 获取当前高度并计算相对高度
        float currentAltitude = 0;
        if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            currentAltitude = g_sensorData.filteredAltitude;
            xSemaphoreGive(sensorDataMutex);
        }
        
        json += (currentAltitude - altitudeController.getInitialValue());
        json += ",\"autoControl\":";
        
        // 获取自动控制状态
        bool autoControl = false;
        if (xSemaphoreTake(motorStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            autoControl = g_motorState.autoControlActive;
            xSemaphoreGive(motorStateMutex);
        }
        
        json += autoControl ? "true" : "false";
        json += "}";
        request->send(200, "application/json", json);
    });
    
    // 修改Web服务器的设置目标高度处理函数
    server.on("/setTargetAltitude", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            float value = request->getParam("value")->value().toFloat();
            
            // 增加调试输出
            Serial.printf("收到目标高度设置请求: %.2f m\n", value);
            
            // 如果PID控制器尚未激活，先设置初始高度
            if (!altitudeController.isActive()) {
                float currentAltitude = 0;
                if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    currentAltitude = g_sensorData.filteredAltitude;
                    xSemaphoreGive(sensorDataMutex);
                }
                
                altitudeController.setInitialValue(currentAltitude);
                altitudeController.activate(true);
                Serial.printf("自动激活PID控制器，基准高度: %.2f m\n", currentAltitude);
            }
            
            // 设置目标高度
            altitudeController.setTargetValue(value);
            
            // 发送更详细的响应
            String response = "目标高度已设置为 " + String(value) + " m";
            request->send(200, "text/plain", response);
            
            // 输出确认
            Serial.println(response);
        } else {
            request->send(400, "text/plain", "缺少参数");
        }
    });
    
    // 切换自动高度控制
    server.on("/autoAltitude", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("enable")) {
            String value = request->getParam("enable")->value();
            bool enable = (value == "1");
            
            if (xSemaphoreTake(motorStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                g_motorState.autoControlActive = enable;
                xSemaphoreGive(motorStateMutex);
            }
            
            // 如果禁用自动控制，停止电机3
            if (!enable) {
                MotorCommand_t stopCmd;
                stopCmd.motorNum = 3;
                stopCmd.value = 0;
                stopCmd.fromAuto = false;
                xQueueSend(motorCommandQueue, &stopCmd, 0);
            }
            
            request->send(200, "text/plain", enable ? "自动控制已启用" : "自动控制已禁用");
        } else {
            request->send(400, "text/plain", "缺少参数");
        }
    });
    
    // 添加到setupWebServer()函数中，在/autoAltitude处理程序之后

    // 添加高度校准端点
    server.on("/calibrateAltitude", HTTP_GET, [](AsyncWebServerRequest *request) {
        // 获取当前高度作为基准高度
        float currentAltitude = 0;
        if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            currentAltitude = g_sensorData.filteredAltitude;
            xSemaphoreGive(sensorDataMutex);
        }
        
        // 设置初始高度并激活PID控制器
        altitudeController.setInitialValue(currentAltitude);
        altitudeController.setTargetValue(0); // 相对高度目标初始为0
        altitudeController.activate(true);    // 激活控制器
        
        Serial.printf("高度已校准，基准高度: %.2f m\n", currentAltitude);
        
        request->send(200, "text/plain", "高度已校准，当前高度设为基准");
    });

    // PID参数端点
    server.on("/getPidParams", HTTP_GET, [](AsyncWebServerRequest *request) {
        String json = "{\"kp\":";
        json += altitudeController.getKp();
        json += ",\"ki\":";
        json += altitudeController.getKi();
        json += ",\"kd\":";
        json += altitudeController.getKd();
        json += "}";
        request->send(200, "application/json", json);
    });
    
    server.on("/setPidParams", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("kp") && request->hasParam("ki") && request->hasParam("kd")) {
            float kp = request->getParam("kp")->value().toFloat();
            float ki = request->getParam("ki")->value().toFloat();
            float kd = request->getParam("kd")->value().toFloat();
            
            altitudeController.setKp(kp);
            altitudeController.setKi(ki);
            altitudeController.setKd(kd);
            
            // 保存到Flash
            preferences.begin(PID_NAMESPACE, false);
            preferences.putFloat("kp", kp);
            preferences.putFloat("ki", ki);
            preferences.putFloat("kd", kd);
            preferences.end();
            
            String json = "{\"kp\":";
            json += kp;
            json += ",\"ki\":";
            json += ki;
            json += ",\"kd\":";
            json += kd;
            json += "}";
            request->send(200, "application/json", json);
        } else {
            request->send(400, "text/plain", "缺少参数");
        }
    });
    
    // 启动Web服务器
    DefaultHeaders::Instance().addHeader("Cache-Control", "max-age=10"); // 10秒缓存
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    
    // 在setupWebServer函数末尾修改服务器设置

    // 优化Web服务器设置
    DefaultHeaders::Instance().addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    DefaultHeaders::Instance().addHeader("Pragma", "no-cache");
    DefaultHeaders::Instance().addHeader("Expires", "0");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

    // 增加服务器最大事件队列大小以提高响应速度
    // server.setMaxRequestContentLength(1024); // 减小最大请求体积
    server.onNotFound([](AsyncWebServerRequest *request){
      request->send(404, "text/plain", "Not found");
    });

    server.begin();
    Serial.println("Web服务器已启动，响应优化已配置");
}

// // 系统状态任务
// void statsTask(void *pvParameters) {
//     const TickType_t xFrequency = pdMS_TO_TICKS(5000); // 5秒打印一次
//     TickType_t xLastWakeTime = xTaskGetTickCount();
    
//     while (1) {
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
//         // 打印系统状态
//         Serial.println("\n---------- 系统状态 ----------");
//         Serial.printf("空闲堆内存: %u 字节\n", xPortGetFreeHeapSize());
        
//         // 传感器数据
//         if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
//             Serial.printf("温度: %.1f°C, 压力: %.1f hPa, 高度: %.2f m\n",
//                 g_sensorData.temperature, g_sensorData.pressure, g_sensorData.filteredAltitude);
//             xSemaphoreGive(sensorDataMutex);
//         }
        
//         // 电机状态
//         if (xSemaphoreTake(motorStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
//             Serial.printf("电机状态: M1=%d%%, M2=%d%%, M3=%d%%, 自动控制=%s\n",
//                 g_motorState.motor1Speed, g_motorState.motor2Speed, g_motorState.motor3Speed,
//                 g_motorState.autoControlActive ? "启用" : "禁用");
//             xSemaphoreGive(motorStateMutex);
//         }
        
//         // WiFi状态
//         Serial.printf("WiFi状态: %s\n", 
//             WiFi.status() == WL_CONNECTED ? "已连接" : "未连接");
//         if (WiFi.status() == WL_CONNECTED) {
//             Serial.printf("IP地址: %s\n", WiFi.localIP().toString().c_str());
//         }
        
//         Serial.println("-------------------------------");
//     }
// }

// 传感器任务 - 负责读取传感器数据并处理
void sensorTask(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 100Hz读取率
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 本地变量
    int tempReadCounter = 0;
    float rawAltitude = 0;
    // 垂直速度计算相关变量
    static float lastAltitude = 0;
    static unsigned long lastVelocityTime = 0;
    static float filteredSpeed = 0;
    // 任务循环
    while (1) {
        // 精确的周期控制
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        tempReadCounter++;
        
        // 大多数时间读取压力并计算高度
        if (tempReadCounter < TEMP_READ_INTERVAL) {
            if (ms5611.read() == MS5611_READ_OK) {
                float pressure = ms5611.getPressure();
                
                // 计算原始高度
                rawAltitude = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));
                
                // 使用简化版指数加权移动平均滤波
                const float alpha = 0.2; // 平滑因子，值越小滤波效果越强
                static float lastFilteredAlt = 0;
                static float filteredAlt = 0;

                // 初始化处理
                if (lastFilteredAlt == 0) {
                    lastFilteredAlt = rawAltitude;
                    filteredAlt = rawAltitude;
                } else {
                    // 应用指数加权移动平均公式
                    filteredAlt = alpha * rawAltitude + (1 - alpha) * lastFilteredAlt;
                    lastFilteredAlt = filteredAlt;
                }
                // ====== 垂直速度计算开始 ======
                unsigned long currentTime = millis();
                if (lastVelocityTime > 0) {  // 不是第一次读取
                    float deltaTime = (currentTime - lastVelocityTime) / 1000.0f;  // 转换为秒
                    if (deltaTime > 0) {  // 防止除零错误
                        float deltaAlt = filteredAlt - lastAltitude;  // 高度变化
                        
                        // 计算原始速度
                        float rawSpeed = deltaAlt / deltaTime;  // m/s
                        
                        // 速度滤波 (使用比高度滤波更强的滤波效果)
                        const float speedAlpha = 0.15;  // 速度滤波因子
                        filteredSpeed = speedAlpha * rawSpeed + (1 - speedAlpha) * filteredSpeed;
                    }
                }
                // 保存当前值作为下一次计算的基础
                lastAltitude = filteredAlt;
                lastVelocityTime = currentTime;
                // ====== 垂直速度计算结束 ======

                // 更新缓冲区状态以保持兼容性
                altitudeBuffer[altitudeBufferIndex] = rawAltitude;
                altitudeBufferIndex = (altitudeBufferIndex + 1) % ALTITUDE_FILTER_SIZE;
                if (altitudeBufferIndex == 0) {
                    altitudeBufferFilled = true;
                }
                
                // 更新全局过滤后高度变量
                filteredAltitude = filteredAlt;
                
                // 获取互斥锁并更新全局传感器数据
                if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    g_sensorData.pressure = pressure;
                    g_sensorData.rawAltitude = rawAltitude;
                    g_sensorData.filteredAltitude = filteredAlt;
                    g_sensorData.verticalSpeed = filteredSpeed;
                    g_sensorData.lastReadTime = millis();
                    xSemaphoreGive(sensorDataMutex);
                    
                    // 通过队列发送滤波后的高度到PID任务
                    xQueueSend(altitudeQueue, &filteredAlt, 0);
                    
                    // 调试输出(降低频率)
                    static uint32_t lastPrintTime = 0;
                    if (millis() - lastPrintTime > 500) {
                      Serial.printf("高度: 原始=%.2f m, 滤波=%.2f m, 速度=%.2f m/s, 压力=%.2f hPa\n",
                          rawAltitude, filteredAlt, filteredSpeed, pressure);
                      lastPrintTime = millis();
                  }
            }
        }
      } else {
            // 偶尔读取温度
            tempReadCounter = 0;
            if (ms5611.read() == MS5611_READ_OK) {
                float temp = ms5611.getTemperature();
                
                // 获取互斥锁更新温度数据
                if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    g_sensorData.temperature = temp;
                    xSemaphoreGive(sensorDataMutex);
                }
            }
          }
    }
}

// 从Flash存储加载PID参数
void loadPidParameters() {
    preferences.begin(PID_NAMESPACE, true); // 只读模式
    
    // 读取PID参数，如果不存在则使用默认值
    float kp = preferences.getFloat("kp", 20.0);
    float ki = preferences.getFloat("ki", 0.1);
    float kd = preferences.getFloat("kd", 5.0);
    
    // 设置PID参数到控制器
    altitudeController.setKp(kp);
    altitudeController.setKi(ki);
    altitudeController.setKd(kd);
    
    preferences.end();
    
    Serial.println("已加载PID参数:");
    Serial.printf("Kp: %.2f, Ki: %.3f, Kd: %.2f\n", kp, ki, kd);
}
