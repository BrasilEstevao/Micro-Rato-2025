#include <Arduino.h>
#include "IRLine.h"
#include "state_machines.h"
#include "robot.h"
#include "config.h"

//#define DEBUG 1



void init_PIO_dual_encoders(int enc1_pin_A, int enc2_pin_A);
int read_PIO_encoder(int sm);




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


   // ADC mux pins
  pinMode(MUXA_PIN, OUTPUT);
  pinMode(MUXB_PIN, OUTPUT);
  pinMode(MUXC_PIN, OUTPUT);
  
  //init_PIO_dual_encoders(ENC1_A, ENC2_A);

  analogReadResolution(10);
  
}


void loop() {
		

		#ifdef DEBUG
		printf ("\n*** Inicio do Ciclo ***\n");
		#endif
    

    
  // Read and process sensors
   // read_PIO_encoders();

  robot.IRLine.readIRSensors();

  robot.IRLine.printIRLine();


    



    robot.setMotorPWM(robot.PWM_1, MOTOR1A_PIN, MOTOR1B_PIN);
    robot.setMotorPWM(robot.PWM_2, MOTOR2A_PIN, MOTOR2B_PIN);

    robot.setMotorPWM(robot.solenoid_PWM, SOLENOID_PIN_A, SOLENOID_PIN_B);


	  edge_detection();
		update_timers();

    // Main_FSM_Handler();
    // Map_FSM_Handler();
    // Solve_FSM_Handler();
    //Test_FSM_Handler();

    // Serial.printf("PWM1: %d\n",robot.PWM_1);
    // Serial.printf("PWM2%d\n",robot.PWM_2);
   
} 





