/* Copyright (c) 2021  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "robot.h"
#include "config.h"

robot_t robot;

int nominal_speed = NOMINAL_SPEED;

robot_t::robot_t()
{
  wheel_dist = 0.125;
  wheel_radius = 0.0689 / 2;
  dv_max = 5;
  dw_max = 10;
  dt = 0.04;

  follow_k = -0.15;
  follow_v = 0.20;
}

void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  w1e = enc1 * TWO_PI / (2.0 * 1920.0 * dt);
  w2e = enc2 * TWO_PI / (2.0 * 1920.0 * dt);

  v1e = w1e * wheel_radius;
  v2e = w2e * wheel_radius;

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;
  
  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = thetae + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
}





void robot_t::accelerationLimit(void)
{
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}


void robot_t::VWToMotorsVoltage(void)
{
  v1ref = v + w * wheel_dist / 2;
  v2ref = v - w * wheel_dist / 2; 
  
  w1ref = v1ref * wheel_radius;
  w2ref = v2ref * wheel_radius;

  if (control_mode == cm_pwm) {

    PWM_1 = PWM_1_req;  
    PWM_2 = PWM_2_req;  
  }

  //  else if (control_mode == cm_pid) {
  //   u1 = 0;
  //   u2 = 0;      

  //   if (v1ref != 0) u1 = PID[0].calc(v1ref, v1e);
  //   else PID[0].Se = 0;

  //   if (v2ref != 0) u2 = PID[1].calc(v2ref, v2e);
  //   else PID[1].Se = 0;

  //   PWM_1 = u1 / battery_voltage * 255;
  //   PWM_2 = u2 / battery_voltage * 255;
 // }
}


/*void robot_t::setMotorPWM(int new_PWM, int pin_a, int pin_b)
{
  int PWM_max = 200;
  if (new_PWM >  PWM_max) new_PWM =  PWM_max;
  if (new_PWM < -PWM_max) new_PWM = -PWM_max;
  
  if (new_PWM == 0) 
  {  // Both outputs 0 -> A = H, B = H
    analogWrite(pin_a, 255);
    analogWrite(pin_b, 255);

  } else if (new_PWM > 0) 
  {
    analogWrite(pin_a, 255 - new_PWM);
    analogWrite(pin_b, 255);

  } 
  else 
  {
    analogWrite(pin_a, 255);
    analogWrite(pin_b, 255 + new_PWM);
  }
}
*/

void robot_t::setMotorSpeed(Adafruit_DCMotor *motor, int speed) {

  if (motor == nullptr) {
    Serial.println("Motor not initialized!");
    return;
  }
  printf("cpt a\n");
  int speed_max = 200;
  //printf("cpt b\n");
  if (speed > speed_max) speed = speed_max;
  if (speed < -speed_max) speed = -speed_max;
  //printf("cpt c\n");
  if (speed > 0) {
    motor->run(FORWARD);
    motor->setSpeed(speed);
    printf("speed = %d\n", speed);
  } else if (speed < 0) {
    motor->run(BACKWARD);
    motor->setSpeed(-speed);
    //printf("cpt e\n");
  } else {
    motor->run(RELEASE);
    //printf("cpt f\n");
  }
}

// void robot_t::followLineRight(float Vnom, float K)
// {
//   w_req = K * IRLine.pos_right;
//   //w_req = w_req * fabs(w_req);
//   //v_req = fmax(0, Vnom - 0.1 * fabs(w_req));
//   v_req = Vnom;
// }


// void robot_t::followLineLeft(float Vnom, float K)
// {
//   w_req = K * IRLine.pos_left;
//   //w_req = w_req * fabs(w_req);
//   //v_req = fmax(0, Vnom - 0.1 * fabs(w_req));
//   v_req = Vnom;
// }




void robot_t::stop()
{
  robot.PWM_1 = 0;
  robot.PWM_2 = 0; 
}



int robot_t::IR_sum(){
  return IRLine.IR_values[0] + IRLine.IR_values[1] + IRLine.IR_values[2] + IRLine.IR_values[3] + IRLine.IR_values[4];
}


// void robot_t::followLine()
// {
//   // robot.IRkp = 0.1;
//   // robot.IRki = 0.02;
//   // robot.IRkd = 0.1;

//   if(robot.IR_sum() < 2500){
//     error = -1.1 * IRLine.IR_values[0] - 1.0 * IRLine.IR_values[1] + 1.0 * IRLine.IR_values[3] + 1.1 * IRLine.IR_values[4];

//     integral += error;
//     derivative = error - prevError;
//     prevError = error;

//     output_w = (robot.IRkp * error + robot.IRki * integral + robot.IRkd * derivative)/10000;

//     //Serial.printf("Output_w = %f\n",output_w);
//     if (output_w > 3.5) output_w = 3.5;
//     else if (output_w < -3.5) output_w = -3.5;

//     // Serial.print(">Output_w:");
//     // Serial.println(robot.output_w);

//     // Aplicação das saídas
//     setRobotVW(robot.follow_v, output_w);
//   }
//   else {
//     integral = 0;
//     setRobotVW(robot.follow_v, 0);
//   }
// }



 void robot_t::left_turn()
 {
   static unsigned long start_time = 0;
    if (start_time == 0) {
        start_time = millis();
       PWM_1 = -100;
       PWM_2 = 100;
    }

    if (millis() - start_time > 435) { // 435ms to turn ~90º
        END_TURN = true;
        start_time = 0; // reset for next turn
    }

 }
void robot_t::right_turn()
{
  static unsigned long start_time = 0;
    if (start_time == 0) {
        start_time = millis();
       PWM_1 = 100;
       PWM_2 = -100;
    }

    if (millis() - start_time > 435) { // 435ms to turn ~90º
        END_TURN = true;
        start_time = 0; // reset for next turn
    }
}

  void robot_t::u_turn()
  {
      static unsigned long start_time = 0;
    if (start_time == 0) {
        start_time = millis();
       PWM_1 = 100;
       PWM_2 = -100;
    }

    if (millis() - start_time > 833) { // 833ms to turn ~180º
        END_TURN = true;
        start_time = 0; // reset for next turn
    }

  }
  void robot_t::reverse()
  {
    PWM_1 =  nominal_speed;
    PWM_2 = nominal_speed;
  }
  void robot_t::forward( )
  {
    PWM_1 =  nominal_speed;
    PWM_2 = nominal_speed;
  }


// int robot_t::IR_sum()
// {
//   return IRLine.IR_values[0] + IRLine.IR_values[1] + IRLine.IR_values[2] + IRLine.IR_values[3] + IRLine.IR_values[4];
// }
