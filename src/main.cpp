#include <Arduino.h>
#include "state_machines.h"
#include "robot.h"
#include "config.h"



#define ENC1_A 2
#define ENC1_B 3

#define ENC2_A 0
#define ENC2_B 1




#define MOTOR1A_PIN 16
#define MOTOR1B_PIN 17

#define MOTOR2A_PIN 14
#define MOTOR2B_PIN 15

#define SOLENOID_PIN_A 12
#define SOLENOID_PIN_B 13




void setMotorPWM(int new_PWM, int pin_a, int pin_b)
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


void init_PIO_dual_encoders(int enc1_pin_A, int enc2_pin_A);
int read_PIO_encoder(int sm);

// void read_PIO_encoders(void)
// {
//   robot.enc1 = read_PIO_encoder(0);
//   robot.enc2 = read_PIO_encoder(1);
// }








void setup()
{

  Serial.begin();

  init_ST();
  
  pinMode (START_BUTTON,INPUT);
  pinMode (RESET_BUTTON,INPUT);

  // Set the pins as input or output as needed
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

 

  // Motor driver pins
  pinMode(MOTOR1A_PIN, OUTPUT);
  pinMode(MOTOR1B_PIN, OUTPUT);
 
  pinMode(MOTOR2A_PIN, OUTPUT);
  pinMode(MOTOR2B_PIN, OUTPUT);

  pinMode(SOLENOID_PIN_A, OUTPUT);
  pinMode(SOLENOID_PIN_B, OUTPUT);
  
  //init_PIO_dual_encoders(ENC1_A, ENC2_A);


  analogReadResolution(10);
  
}


void loop() {
		

		#ifdef DEBUG
		printf ("\n*** Inicio do Ciclo ***\n");
		#endif
    

    
  // Read and process sensors
   // read_PIO_encoders();

    
    //readIRSensors();


    robot.setRobotVW(robot.v_req, robot.w_req);
    
    //robot.accelerationLimit();
    robot.v = robot.v_req;
    robot.w = robot.w_req;


    //robot.followLineLeft(robot.follow_v, robot.follow_k);
    //robot.followLineRight(robot.follow_v, robot.follow_k);
    

    setMotorPWM(robot.PWM_1, MOTOR1A_PIN, MOTOR1B_PIN);
    setMotorPWM(robot.PWM_2, MOTOR2A_PIN, MOTOR2B_PIN);

    setMotorPWM(robot.solenoid_PWM, SOLENOID_PIN_A, SOLENOID_PIN_B);


	  edge_detection();
		update_timers();

    // Main_FSM_Handler();
    // Map_FSM_Handler();
    // Solve_FSM_Handler();
    Test_FSM_Handler();

    Serial.printf("PWM1: %d\n",robot.PWM_1);
    Serial.printf("PWM2%d\n",robot.PWM_2);
   
} 





