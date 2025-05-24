// #include <Arduino.h>
// #include "IRLine.h"
// #include "state_machines.h"
// #include "robot.h"
// #include "config.h"
// #include <Wire.h>
// #include <Adafruit_MotorShield.h>
// #include "utility/Adafruit_MS_PWMServoDriver.h"

// #define DEBUG 1
// #include "path_handler.h"

// #define DEBUG 1

// Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 


// Adafruit_DCMotor *motor_1 = AFMS.get Motor(3); // M1 on shield
// Adafruit_DCMotor *motor_2 = AFMS.getMotor(4); // M2 on shield



// void setup()
// {

//   Serial.begin(115200);
//   while(!Serial); // Wait for serial port to connect
//   Serial.println("\n\nStarting..."); 
//   delay(1000); // Give time to open serial monitor

//   init_ST();
  
//   pinMode (RESET_BUTTON, INPUT_PULLUP);
//   pinMode (START_BUTTON, INPUT_PULLUP);
  

//   Set the pins as input or output as needed
//   pinMode(ENC1_A, INPUT_PULLUP);
//   pinMode(ENC1_B, INPUT_PULLUP);
//   pinMode(ENC2_A, INPUT_PULLUP);
//   pinMode(ENC2_B, INPUT_PULLUP);


//   delay(500);


//   Wire.begin(); // ESP32 default SDA=21, SCL=22
//   delay(100);

//   AFMS.begin();
  
  

//   robot.motor_L = motor_1;
//   robot.motor_R = motor_2;

//   Verify motor initialization
//   if (!robot.motor_L || !robot.motor_R) {
//     Serial.println("Motor initialization failed!");
//     while(1); // Halt if motors didn't initialize
//   }

//   init_PIO_dual_encoders(ENC1_A, ENC2_A);

//   analogReadResolution(10);

//   Serial.println("Setup complete. Starting main loop...");
//   Initialize the robot stopped
//   currentStateTest = STOP_TEST;
  
// }


// void loop() {

// 		#ifdef DEBUG
// 		Serial.println ("\n*** Inicio do Ciclo ***\n");
// 		#endif
    

//   Read and process sensors
//   read_PIO_encoders();

//   robot.IRLine.readIRSensors();
//   robot.IRLine.printIRLine();
//   robot.PWM_1= 100; //testando
//   robot.PWM_2= 100; //motores

//   robot.setMotorSpeed(robot.motor_L, robot.PWM_1);
//   robot.setMotorSpeed(robot.motor_R, robot.PWM_2);
//   Serial.println("cpt 2\n");

//     motor_1->setSpeed(150);
//     motor_1->run(FORWARD);
//     motor_2->setSpeed(150);
//     motor_2->run(FORWARD);
// 	  edge_detection();




//     Main_FSM_Handler();
//     Map_FSM_Handler();
//     Solve_FSM_Handler();
//     Test_FSM_Handler();

//     Serial.printf("PWM1: %d\n",robot.PWM_1);
//     Serial.printf("PWM2%d\n",robot.PWM_2);
   
//     delay(100);
// } 



#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Define I2C pins for ESP32 (default: 21-SDA, 22-SCL)
#define I2C_SDA 21
#define I2C_SCL 22

// Create motor shield object with default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Create motor objects - adjust these numbers (1-4) to match your motor connections
Adafruit_DCMotor *motor_L = AFMS.getMotor(1);  // Left motor on M1
Adafruit_DCMotor *motor_R = AFMS.getMotor(2);  // Right motor on M2

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection

  Serial.println("\n*** I2C Debugger ***");

  // Initialize I2C with pull-ups enabled (ESP32-specific)
  Wire.begin(I2C_SDA, I2C_SCL, 100000); // 100kHz speed
  Wire.setClock(100000);

  // Test I2C bus voltage levels
  pinMode(I2C_SDA, INPUT);
  pinMode(I2C_SCL, INPUT);
  Serial.print("SDA voltage: "); Serial.println(digitalRead(I2C_SDA) ? "HIGH (OK)" : "LOW (Problem!)");
  Serial.print("SCL voltage: "); Serial.println(digitalRead(I2C_SCL) ? "HIGH (OK)" : "LOW (Problem!)");

  // Scan I2C bus
  Serial.println("\nScanning I2C bus...");
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Found device at 0x"); Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) Serial.println("No I2C devices found!");
}

void loop() {
  // Simple motor test sequence
  Serial.println("\nRunning FORWARD");
  motor_L->setSpeed(150);
  motor_R->setSpeed(150);
  motor_L->run(FORWARD);
  motor_R->run(FORWARD);
  delay(2000);

  Serial.println("Running BACKWARD");
  motor_L->setSpeed(150);
  motor_R->setSpeed(150);
  motor_L->run(BACKWARD);
  motor_R->run(BACKWARD);
  delay(2000);

  Serial.println("STOPPING");
  motor_L->run(RELEASE);
  motor_R->run(RELEASE);
  delay(1000);

  // Ramp speed test
  Serial.println("Speed ramp test");
  for (int i=0; i<=255; i++) {
    motor_L->setSpeed(i);
    motor_R->setSpeed(i);
    motor_L->run(FORWARD);
    motor_R->run(FORWARD);
    delay(10);
  }
  motor_L->run(RELEASE);
  motor_R->run(RELEASE);
  delay(2000);
}

