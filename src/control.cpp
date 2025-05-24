//#define RUN 
#define TEST 1
//#define DEBUG 1
//#define SUPERDEBUG 1

#include <Arduino.h>
#include "state_machines.h"
#include "robot.h"
#include "path_handler.h"


extern robot_t robot;

stack<char> path_taken, solved_path;

StateNamesMain currentStateMain = IDLE_MAIN;
StateNamesMap currentStateMap = IDLE_MAP;
StateNamesSolve currentStateSolve = IDLE_SOLVE;
StateNamesTest currentStateTest = STOP_TEST;

bool buttonStart, prevbuttonStart;
bool buttonRestart, prevbuttonRestart;

// Variáveis globais
bool END_MAP = false;
bool END_SOLVE = false;

bool cross=false;
int counter_align=0;
static unsigned long forwardStartTime = 0;


//edge detection variables
int p_START_BUTTON = 0;
int re_START_BUTTON = 0;

bool buttonStart_RE, buttonRestart_RE, buttonStart_FE, buttonRestart_FE; 




// Cycle time variables
unsigned int start_time = 0, end_time = 0, cycle_time = 0;

// Timers (exemplo)


// timerBlock timer_Exampler;


void edge_detection(){
  //   bool currentButtonState = digitalRead(START_BUTTON);

  //   //if ((millis() - lastDebounceTime) > debounceDelay) {
  //       if (!p_START_BUTTON && currentButtonState) {
  //           re_START_BUTTON = true;
  //       } else {
  //           re_START_BUTTON = false;
  //       }
  //       p_START_BUTTON = currentButtonState;
  
  //   //}
	// #ifdef DEBUG
	// Serial.printf("-- Edges re_BUTTON=%d p_BUTTON=%d\n", re_START_BUTTON, p_START_BUTTON);
	// #endif

  prevbuttonStart = buttonStart;
  buttonStart = !digitalRead(START_BUTTON);
  prevbuttonRestart = buttonRestart;
  buttonRestart = !digitalRead(START_BUTTON);

  buttonStart_RE = (buttonStart && !prevbuttonStart);
  buttonRestart_RE = (buttonRestart && !prevbuttonRestart);
  buttonStart_FE = (!buttonStart && prevbuttonStart);
  buttonRestart_FE = (!buttonRestart && prevbuttonRestart);

}

void update_timers()
{
	// end_time = get_time();

	// if (start_time == 0)
	//     cycle_time = 0;
	// else
	//     cycle_time = end_time - start_time;

	// start_time = end_time;

	// Exemplo:
	// if (timer_State.on == 1)
	//     timer_State.time += cycle_time;

	// #ifdef DEBUG
	// printf("-- Timers timer_State.on=%d timer_State.time=%d\n", timer_State.on, timer_State.time);
	// #endif
}



void start_timer(timerBlock* t)
{
	t->on = 1;
	t->time = 0;
}

void stop_timer(timerBlock* t)
{
	t->on = 0;
	t->time = 0;
}






#ifdef RUN
void main_FSM_Handler() 
{
 switch (currentStateMain) 
		{
			
			case IDLE_MAIN :
			
				#ifdef DEBUG
				printf("-- Current state main  = IDLE\n");
				#endif

				
				if (digitalRead(START_BUTTON)) 
				{
    					currentStateMain = MAP;
        }

				
				break;
			
			case MAP:
			
				#ifdef DEBUG
				printf("-- Current state main = MAP\n");
				#endif			
			
				
				if(END_MAP)
				{
					currentStateMain = READY;
				}

			break;
			
			case READY :
			
				#ifdef DEBUG
				printf("-- Current state main = READY\n");
				#endif


				
				if(digitalRead(START_BUTTON))
				{
					currentStateMain = SOLVE;
				}

        if(digitalRead(RESET_BUTTON))
        {
          currentStateMain = MAP;
        }

			break;
			
			case SOLVE:
			#ifdef DEBUG
			printf("-- Current state main = SOLVE\r\n");
			#endif
		
		
			if (END_SOLVE) 
			{
				currentStateMain = SOLVED; 
			}
		
			break;
		
		case SOLVED:
			#ifdef DEBUG
			printf("-- Current state main = SOLVED\n");
			#endif
		
			
			if (digitalRead(START_BUTTON)) 
			{
        currentStateMain = SOLVE;
			}

      if(digitalRead(RESET_BUTTON))
      {
        currentStateMain = IDLE_MAIN;
      }
			break;

		} 
        
        
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------     
// Update outputs
switch(currentStateMain)
    {
    case IDLE_MAIN:
        //stop motors
        // clear up instructions and temporary stacks 
        break;

    case MAP:
        //stop motors again( I guess)
        break;

    case READY:
        //copy instructions to temporary stack; (for running again if needed)
        break;

    default:
        break;
    }
}










void Map_FSM_Handler()
{
    switch(currentStateMap)
		{

			case IDLE_MAP:

				#ifdef DEBUG
				printf("-- Current state map = IDLE\n");
				#endif

				if(currentStateMain == MAP)
        {
          currentStateMap = FOLLOW_LINE_MAP;
        }

			break;

			case FOLLOW_LINE_MAP:

				#ifdef DEBUG
				printf("-- Current state map = FOLLOW_LINE\n");
				#endif

				if(detect_all_white()) //OOOOO
				{
					currentStateMap = U_TURN;
				}
				
        if(detect_right() || detect_full_line()) //OOXXX || XXXXX
        {
          currentStateMap = SMALL_FORWARD;
        }

        if(detetct_left()) //XXXOO
        {
          currentStateMap = LEFT_TURN_MAP;
        }

      break;


      case U_TURN:

        #ifdef DEBUG
        printf("-- Current state map = U_TURN\n");
        #endif

				if(robot.END_TURN)
				{
          currentStateMap = FOLLOW_LINE_MAP;
				}

			break;


			case LEFT_TURN_MAP:

        #ifdef DEBUG
        printf("-- Current state map = LEFT_TURN\n");
        #endif

        if(robot.END_TURN)
        {
          currentStateMap = FOLLOW_LINE_MAP;
        }

      break;

      case RIGHT_TURN_MAP:

        #ifdef DEBUG
        printf("-- Current state map = RIGHT_TURN\n");
        #endif

        if(robot.END_TURN)
        {
          currentStateMap = FOLLOW_LINE_MAP;
        }

      break;


      case SMALL_FORWARD:

        #ifdef DEBUG
        printf("-- Current state map = SMALL_FORWARD\n");
        #endif
				
        if(detect_forward()) //OOXOO
        {
          currentStateMap = FOLLOW_LINE_MAP;
        }

        if(detect_all_black()) //XXXXX
        {
          currentStateMap = END;
        }

			break;

      case FORWARD_MAP:

        #ifdef DEBUG
        printf("-- Current state map = FORWARD\n");
        #endif

        if(1)
        {
          currentStateMap = FOLLOW_LINE_MAP;
        }

      break;

      case END:

        #ifdef DEBUG
        printf("-- Current state map = END\n");
        #endif

        if(1)
        {
          currentStateMap = IDLE_MAP;
		    }

      break;
    }



    //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------     
 //update outputs

// Update outputs
switch(currentStateMap)
    {
    case IDLE_MAP:
        
      robot.stop();

        break;

    case FOLLOW_LINE_MAP:
        //follow_line funtion

        robot.followLine(); 
        //Only real use of the PID
        break;

    case U_TURN:
        //U-turn function 
        robot.u_turn();
        //Just angular speed until -180 degrees (use encoders)
        break;

    case LEFT_TURN_MAP:
        //left_turn function 
        robot.left_turn();
        //Just angular speed until -90 degrees (use encoders)
        break;

    case RIGHT_TURN_MAP:
        //right_turn function
        robot.right_turn();
        //Just angular speed until 90 degrees (use encoders)
        break;

    case REVERSE:
        //reverse function 
        robot.reverse();
        //activate motors in oposite until see line (in this case, since its always turning left, it will see either OOXXX or OOXOO)
        break;

    case SMALL_FORWARD:
        //small_forward function (maybe just forward function but with timer)
        robot.forward();
        break;

    case FORWARD_MAP:
        //forward function (go forward until OOXOO)
        robot.forward();
        break;

    case END:
       // solve_algorithm();
        robot.stop();
        break;

    default:
        break;
    }
}












void Solve_FSM_Handler()
{
  	switch (currentStateSolve)
		{



			case IDLE_SOLVE:
      
        #ifdef DEBUG
        printf("-- Current state solve = IDLE\n");
        #endif

        if(currentStateMain == SOLVE)
        {
          currentStateSolve = FOLLOW_LINE_SOLVE;
        }

			break;




			case FOLLOW_LINE_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = FOLLOW_LINE\n");
        #endif

        if(Detect_node)
        {
          currentStateSolve  = GET_INSTRUCTION;
        }
			break;




			case GET_INSTRUCTION:

        #ifdef DEBUG
        printf("-- Current state solve = GET_INSTRUCTION\n");
        #endif

				if(instruction == 'R')
				{
					currentStateSolve = RIGHT_TURN_SOLVE;
				}

        if(instruction == 'L')
        {
          currentStateSolve = LEFT_TURN_SOLVE;
        }

        if(instruction == 'F')
        {
          currentStateSolve = FORWARD_SOLVE;
        }
      
        if(instructions.empty())
        {
          currentStateSolve = FINISH;
        }

			break;




			case RIGHT_TURN_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = RIGHT_TURN_SOLVE\n");
        #endif

        if(back_on_line)
        {
          currentStateSolve = FOLLOW_LINE_SOLVE;
        }

			break;



    case LEFT_TURN_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = LEFT_TURN_SOLVE\n");
        #endif

        if(back_on_line)
        {
          currentStateSolve = FOLLOW_LINE_SOLVE;
        }

      break;

      case FORWARD_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = FORWARD_SOLVE\n");
        #endif

        if(OOXOO)
        {
          currentStateSolve = FOLLOW_LINE_SOLVE;
        }

      break;

      case FINISH:

        #ifdef DEBUG
        printf("-- Current state solve = FINISH\n");
        #endif

        if(1)
        {
          currentStateSolve = IDLE_SOLVE;
        }

      break;

      }




 //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------     
// Update outputs
switch(currentStateSolve)
    {
    case IDLE_SOLVE:
        robot.stop();
        break;

    case FOLLOW_LINE_SOLVE:
        robot.followLine();
        break;

    case GET_INSTRUCTION:
        //get_instruction function 
        //char currentInstruction = pathway.top(); 
        //pathway.pop();
        break;

    case RIGHT_TURN_SOLVE:
        robot.right_turn();
        break;

    case LEFT_TURN_SOLVE:
        robot.left_turn();
        break;

    case FORWARD_SOLVE:
        robot.forward();
        break;

    case FINISH:
        robot.stop();
        break;

    default:
        break;
    }

}

#endif


void Test_FSM_Handler()
{

    Serial.print("State: ");
    Serial.println(currentStateTest);

    // Declare 'type_of_node' once, outside the switch statement
    char type_of_node = robot.IRLine.detectNode();

    // Debugging output
    // Serial.printf("RE_START_BUTTON %d\n", re_START_BUTTON);
    // Serial.println();

    static bool pushedR = false;  // Flag for Right turn state
    static bool pushedL = false;  // Flag for Left turn state
    static bool pushedU = false;  // Flag for U-turn state
    static bool pushedF = false;  // Flag for Forward state

    switch (currentStateTest)
    {
      case FOLLOW_TEST:
          #ifdef DEBUG
          Serial.print("-- Current state test = FOLLOW \n");
          #endif

          // Reset push flags so each state can push only once during its activation
          pushedR = false;
          pushedL = false;
          pushedU = false;
          pushedF = false;

          if (re_START_BUTTON == 1)
          {
            #ifdef SUPERDEBUG
          Serial.printf("-- Current state test =  FOLLOW_TEST (re_START_BUTTON) \n");
          #endif
              currentStateTest = STOP_TEST;
          }


          if (type_of_node == 'B')
          {

              #ifdef SUPERDEBUG
              Serial.printf("-- Current state test = FOLLOW_TEST (R) \n");
              #endif

              forwardStartTime = 0;
              cross = true;
              currentStateTest = SMALL_FORWARD_TEST;
          } 
          else if (type_of_node == 'L') {

              #ifdef SUPERDEBUG
              Serial.printf("-- Current state test = FOLLOW_TEST (L) \n");
              #endif
              path_taken.push('L');
              currentStateTest = LEFT_TURN_TEST;
          } 
          else if (type_of_node == 'W') {

              #ifdef SUPERDEBUG
              Serial.printf("-- Current state test = FOLLOW_TEST (W) \n");
              #endif
              currentStateTest = U_TURN_TEST;
          } 
          else if(type_of_node == 'R')
          {
              #ifdef SUPERDEBUG
              Serial.printf("-- Current state test = FOLLOW_TEST (B) \n");
              #endif
               forwardStartTime = 0;
              currentStateTest =SMALL_FORWARD_TEST;
          }
          break;
          
      case FORWARD_TEST:
        #ifdef DEBUG
        Serial.print("-- Current state test = FORWARD \n");
        #endif

        if(buttonStart_RE == 1)
        {

          #ifdef SUPERDEBUG
          Serial.printf("-- Current state test = FORWARD_TEST \n");
          #endif
          currentStateTest = STOP_TEST;
        }
        else if(type_of_node == 'N')  // Black, End of path
        { 
          if(!pushedF)
          {
            path_taken.push('F');
            Serial.println("Pushed F");
            pushedF = true;
          }

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = FORWARD_TEST \n");
            #endif
            currentStateTest = FOLLOW_TEST;
        }
        break;

         case SMALL_FORWARD_TEST:
        #ifdef DEBUG
        Serial.print("-- Current state test = SMALL_FORWARD\n");
        #endif

        // Check if enough time has passed or if the robot has moved a small distance.
        static bool hasMovedEnough = false;

        if (forwardStartTime == 0) 
        {
            // First time entering SMALL_FORWARD_TEST, start the timer
            forwardStartTime = millis();
        }

        // Check if the robot has moved forward long enough (adjust time as necessary)
        if (millis() - forwardStartTime > 200) { // 1000 ms = 1 second, adjust this as needed
            hasMovedEnough = true;
        }

        if (hasMovedEnough) 
        {
            // Check the node type after moving a little further
          
              if (type_of_node == 'B')  // Black, End of path
            { 
                currentStateTest = END_TEST;
            }
            else if (type_of_node == 'N')  // Cross or Right T junction
            { 
              currentStateTest = FOLLOW_TEST;

               if(cross == true)
               {
                cross = false;
                currentStateTest = BACKWARD_TEST;
               }
                
            }
            else if (type_of_node == 'W')  // White, U-turn
            { 
                currentStateTest = BACKWARD_TEST;
            }

            forwardStartTime = 0;  // Reset the timer
            hasMovedEnough = false; // Reset the movement condition
        }

        break;

      case RIGHT_TURN_TEST:
        #ifdef DEBUG
        Serial.printf("-- Current state test = RIGHT_TURN \n");
        #endif
        
        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = RIGHT_TURN_TEST \n");
            #endif
            currentStateTest = STOP_TEST;
        }

       else  if (type_of_node == 'N' && robot.END_TURN)  // Turn finished
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = RIGHT_TURN_TEST \n");
            #endif
            currentStateTest = FOLLOW_TEST;
            robot.END_TURN = false;
        }
        break;

      case LEFT_TURN_TEST:
        #ifdef DEBUG
        Serial.printf("-- Current state test = LEFT_TURN \n");
        #endif
       
        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = LEFT_TURN_TEST \n");
            #endif
            currentStateTest = STOP_TEST;
        }

        else if (type_of_node == 'N' && robot.END_TURN)  // Turn finished
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = LEFT_TURN_TEST \n");
            #endif
            currentStateTest = FOLLOW_TEST;
            robot.END_TURN = false;
        }
        break;

      case U_TURN_TEST:

        #ifdef DEBUG
        Serial.print("-- Current state test = U_TURN \n");
        #endif
        
        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = U_TURN_TEST (start_button)\n");
            #endif
            currentStateTest = STOP_TEST;
        }

        else if (type_of_node == 'N' && robot.END_TURN) // U-turn finished
        {
            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = U_TURN_TEST (end_turn) \n");
            #endif
            currentStateTest = FOLLOW_TEST;
            robot.END_TURN = false;
        }
        break;

      case BACKWARD_TEST:
        #ifdef DEBUG
        Serial.print("-- Current state test = BACKWARD \n");
        #endif

        type_of_node = robot.IRLine.detectNode();
        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = BACKWARD_TEST \n");
            #endif
            currentStateTest = STOP_TEST;
        }

        else if (type_of_node == 'R')  // Reversing until find right turn
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = BACKWARD_TEST \n");
            #endif
            currentStateTest = RIGHT_TURN_TEST;
        }

        else if (type_of_node == 'B')  // Reversing until find T junction
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = BACKWARD_TEST \n");
            #endif
            currentStateTest = LEFT_TURN_TEST;
        }
    
        break;

      case STOP_TEST:
        #ifdef DEBUG
        Serial.printf("-- Current state test = STOP  \n");
        #endif

        robot.END_TURN = false;

        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = STOP_TEST \n");
            #endif
            currentStateTest = FOLLOW_TEST;
        }

        break;

      case END_TEST:
        #ifdef DEBUG
        Serial.printf("-- Current state test = END \n");
        #endif

        robot.END_TURN = false;

        if (re_START_BUTTON == 1)
        {

            #ifdef SUPERDEBUG
            Serial.printf("-- Current state test = END_TEST \n");
            #endif
            currentStateTest = FOLLOW_TEST;
        }

        break;
    }



    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=



    // Update outputs
    switch (currentStateTest)
    {
      case FOLLOW_TEST:
        robot.followLine();
        break;

      case FORWARD_TEST:
        robot.forward();
        break;

      case SMALL_FORWARD_TEST:
        robot.forward();
        break;

      case RIGHT_TURN_TEST:
        robot.right_turn();
        break;

      case LEFT_TURN_TEST:
        robot.left_turn();
        break;

      case U_TURN_TEST:
        robot.u_turn();
        break;

      case BACKWARD_TEST:
        robot.reverse();
        break;

      case STOP_TEST:
        robot.stop();
        break;
      case END_TEST:
        // #ifdef DEBUG
        // Serial.printf("-- Current state test = END \n");
        // #endif

        robot.stop();
        solved_path = get_path(path_taken);  // Process the path once the test is over
        Serial.println("Taken Path: ");
        while (!path_taken.empty()) {
            char c = path_taken.top();
            Serial.print(c);
            path_taken.pop();
        }
        break;
    }
}
