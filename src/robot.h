
#ifndef ROBOT_H
  #define ROBOT_H
#endif

#include <Arduino.h>
#include <math.h>
//#include "PID.h"
#include "IRLine.h"
#include "state_machines.h"

#ifndef NUM_WHEELS
#define NUM_WHEELS 2

typedef enum { 
  cm_pwm,
  cm_pid
} control_mode_t;

class robot_t {
  public:

  //encoder stuff
  int enc1, enc2;
  float w1e, w2e;
  float v1e, v2e;
  float ve, we;
  float ds, dtheta;
  float rel_s, rel_theta;
  float xe, ye, thetae;


  //turning flags
  bool END_TURN = false;
  unsigned long turn_start_time = 0;
  bool is_turning = false;

  float dt;
  float v, w;
  float v_req, w_req;
  float dv_max, dw_max;
  
  float wheel_radius, wheel_dist;
  
  float v1ref, v2ref;
  float w1ref, w2ref;
  float u1, u2;
  int PWM_1, PWM_2;
  int PWM_1_req, PWM_2_req;
  control_mode_t control_mode;

  double  IRkp = 0.1, IRki = 0, IRkd = 0.30;
  //d =0.35 and 120 good
  
  double lastIRkp = IRkp, lastIRki = IRki, lastIRkd = IRkd;

  double error;         // Error
  double prevError = 0; // Previous Error for each sensor
  double integral = 0; // Integral for each sensor
  double derivative = 0; // Derivative for each sensor


float right_v = 0.0, left_v = 0.0, right_w = 0.0, left_w = 0.0;


  float follow_v, follow_k;
  
  //PID_t PID[NUM_WHEELS];
  float battery_voltage;
  int button_state;
  IRLine_t IRLine;  
  
  robot_t();

  void odometry(void);

  void setRobotVW(float Vnom, float Wnom);
  void accelerationLimit(void);
  void VWToMotorsVoltage(void);

  void setMotorPWM(int new_PWM, int pin_a, int pin_b);
 

  void followLineRight(float Vnom, float K);
  void followLineLeft(float Vnom, float K);
  void followLine();

  //node functions

  void stop();
  void left_turn();
  void right_turn();
  void u_turn();
  void reverse();
  void forward();


  int IR_sum();
};


extern robot_t robot;


#endif // ROBOT_H
