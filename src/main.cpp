#include <Arduino.h>
#include "IRLine.h"
#include "state_machines.h"
#include "robot.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define DEBUG 1


/*
void init_PIO_dual_encoders(int enc1_pin_A, int enc2_pin_A);
int read_PIO_encoder(int sm);
*/


void setup()
{

  Serial.begin(115200);

  init_ST();
  
  pinMode (START_BUTTON,INPUT_PULLDOWN);
  pinMode (RESET_BUTTON,INPUT_PULLDOWN);

  // Set the pins as input or output as needed
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

  AFMS.begin();  // Default I2C address 0x60

  Adafruit_DCMotor *motor_1 = AFMS.getMotor(3); // M1 on shield
  Adafruit_DCMotor *motor_2 = AFMS.getMotor(4); // M2 on shield

  robot.motor_L = motor_1;
  robot.motor_R = motor_2;

  // Verify motor initialization
  if (!robot.motor_L || !robot.motor_R) {
    Serial.println("Motor initialization failed!");
    while(1); // Halt if motors didn't initialize
  }

  //init_PIO_dual_encoders(ENC1_A, ENC2_A);

  analogReadResolution(10);

  Serial.println("Setup complete. Starting main loop...");
  
}


void loop() {

		#ifdef DEBUG
		// printf ("\n*** Inicio do Ciclo ***\n");
		#endif
    

  // Read and process sensors
  // read_PIO_encoders();

  robot.IRLine.readIRSensors();
  //robot.IRLine.printIRLine();
  robot.PWM_1= 100;
  robot.PWM_2= 100;

  robot.setMotorSpeed(robot.motor_L, robot.PWM_1);
  //robot.setMotorSpeed(robot.motor_R, robot.PWM_2);
  printf("cpt 2\n");

  edge_detection();
  update_timers();
  // printf("cpt 3");
  //main_FSM_Handler();
  // Map_FSM_Handler();
  // Solve_FSM_Handler();
  //Test_FSM_Handler();

  // Serial.printf("PWM1: %d\n",robot.PWM_1);
  // Serial.printf("PWM2%d\n",robot.PWM_2);
  delay(250);
} 



