#ifndef STATE_MACHINES_H
#define STATE_MACHINES_H

#include "config.h"

// Define timer block for timers
typedef struct {
	unsigned char on;
	unsigned int time;
} timerBlock;

// Main state machine state names
typedef enum {
  IDLE_MAIN,       
  MAP,        
  READY,     
  SOLVE,      
  SOLVED     
} StateNamesMain; 

// Mapping state machine state names
typedef enum {
  IDLE_MAP,      
  FOLLOW_LINE_MAP, 
  U_TURN,     
  LEFT_TURN_MAP,  
  RIGHT_TURN_MAP, 
  REVERSE,    
  SMALL_FORWARD, 
  FORWARD_MAP,
  END      
} StateNamesMap;

// Solving state machine state names
typedef enum {
  IDLE_SOLVE,      
  FOLLOW_LINE_SOLVE,   
  GET_INSTRUCTION, 
  LEFT_TURN_SOLVE,  
  RIGHT_TURN_SOLVE,
  FORWARD_SOLVE,
  FINISH  
} StateNamesSolve;

// Test state machine state names
typedef enum {
  FOLLOW_TEST,
  FORWARD_TEST,
  SMALL_FORWARD_TEST,
  RIGHT_TURN_TEST,
  LEFT_TURN_TEST, 
  BACKWARD_TEST,
  U_TURN_TEST,
  STOP_TEST,
  END_TEST,
  ALIGNING_TEST,
  IDLE_TEST
} StateNamesTest;

// Extern variables
// // extern StateNamesMain currentStateMain;
// // extern StateNamesMap currentStateMap;
// // extern StateNamesSolve currentStateSolve;
// extern StateNamesTest currentStateTest;



extern bool END_MAP;
extern bool END_SOLVE;

// Function declarations
void edge_detection();
void update_timers();
void start_timer(timerBlock* t);
void stop_timer(timerBlock* t);
void init_ST();


//state machines
void Main_FSM_Handler();
void Map_FSM_Handler();
void Solve_FSM_Handler();
//state machines
void Main_FSM_Handler();
void Map_FSM_Handler();
void Solve_FSM_Handler();
void Test_FSM_Handler();

#endif // STATE_MACHINES_H
