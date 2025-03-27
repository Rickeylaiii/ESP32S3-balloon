#ifndef PID_H
#define PID_H

class PidController {
private:
    // PID control parameters
    float kp;  // Proportional gain
    float ki;  // Integral gain
    float kd;  // Derivative gain
    
    // PID control state variables
    float errorSum;       // Error accumulation (for integral term)
    float lastError;      // Last error (for derivative term)
    unsigned long lastTime; // Last PID calculation time
    
    // Height control related variables
    float targetValue;      // Target value (relative to initial value)
    float initialValue;     // Initial reference value
    bool isInitialized;     // Whether initial value has been set
    bool isEnabled;         // Whether controller is enabled
    
    // Configuration
    int updateInterval;     // Update interval (milliseconds)
    float errorSumLimit;    // Integral term limit

public:
    // Constructor
    PidController(float p = 2.0, float i = 0.1, float d = 1.0, int interval = 100);
    
    // Configuration methods
    void setTunings(float p, float i, float d);
    void setInterval(int intervalMs);
    void setErrorSumLimits(float limit);
    
    // Control methods
    void calibrateInitialValue(float currentValue);
    void setTargetValue(float newTarget);
    void enable();
    void disable();
    bool isActive() const;
    void reset();
    
    // 添加方法别名，保持与balloon.ino调用兼容
    void setInitialValue(float value) { calibrateInitialValue(value); }
    void activate(bool state) { if(state) enable(); else disable(); }
    
    // Calculate PID output
    int compute(float currentValue);
    
    // Get status
    float getTarget() const;
    float getInitialValue() const;
    bool isInitialValueSet() const;
    float getCurrentRelative(float currentValue) const;
    
    // Debug output
    void printDebug(float currentValue, int output);

    // 在PidController类中添加这些方法
    void setKp(float newKp) { kp = newKp; }
    void setKi(float newKi) { ki = newKi; }
    void setKd(float newKd) { kd = newKd; }
    float getKp() { return kp; }
    float getKi() { return ki; }
    float getKd() { return kd; }
};

#endif // PID_H