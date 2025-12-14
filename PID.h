#ifndef PID_H
#define PID_H

class PIDController {
  public:
    // Constructor to set tuning parameters
    PIDController(float p, float i, float d);

    // Function to calculate the output
    // setpoint: Desired speed (from controller)
    // measured: Actual speed (gx, gy, or gz)
    float compute(float setpoint, float measured);
    
    // Method to update gains if needed during runtime
    void setGains(float p, float i, float d);
    
    void reset();

  private:
    float Kp, Ki, Kd;
    float previousError;
    float integral;
    unsigned long lastTime;
};



#endif