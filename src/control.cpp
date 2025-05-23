#define RUN 
#define TEST 1

#define DEBUG 1

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "state_machines.h"
#include "robot.h"


extern robot_t robot; 

StateNamesMain currentStateMain = IDLE_MAIN;
StateNamesMap currentStateMap = IDLE_MAP;
StateNamesSolve currentStateSolve = IDLE_SOLVE;
StateNamesTest currentStateTest = STOP_TEST;

// Variáveis globais
bool END_MAP = false;
bool END_SOLVE = false;


//edge detection variables
int p_START_BUTTON = 0;
int re_START_BUTTON = 0;




// Cycle time variables
unsigned int start_time = 0, end_time = 0, cycle_time = 0;

// Timers (exemplo)


// timerBlock timer_Exampler;

void init_ST()
{
	// Inicializa estados
	currentStateMain = IDLE_MAIN;
	currentStateMap = IDLE_MAP;
	currentStateSolve = IDLE_SOLVE;
	currentStateTest = FORWARD_TEST;

	// Variáveis internas
	END_MAP = false;
	END_SOLVE = false;

	// Outputs iniciais (exemplo: motores desligados)
}

void edge_detection()
{
    bool currentButtonState = digitalRead(START_BUTTON);

    //if ((millis() - lastDebounceTime) > debounceDelay) {
        if (!p_START_BUTTON && currentButtonState) {
            re_START_BUTTON = true;
        } else {
            re_START_BUTTON = false;
        }
        p_START_BUTTON = currentButtonState;
  
    //}
	#ifdef DEBUG
	Serial.printf("-- Edges re_BUTTON=%d p_BUTTON=%d\n", re_START_BUTTON, p_START_BUTTON);
	#endif
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
/*
				if(detect_all_white()) 
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
*/
      break;


      case U_TURN:

        #ifdef DEBUG
        printf("-- Current state map = U_TURN\n");
        #endif
/*
				if(END_U_TURN)
				{
          currentStateMap = FOLLOW_LINE_MAP;
				}
*/
			break;


			case LEFT_TURN_MAP:

        #ifdef DEBUG
        printf("-- Current state map = LEFT_TURN\n");
        #endif
/*
        if(END_LEFT_TURN)
        {
          currentStateMap = FOLLOW_LINE_MAP;
        }
*/
      break;

      case SMALL_FORWARD:

        #ifdef DEBUG
        printf("-- Current state map = SMALL_FORWARD\n");
        #endif
				/*
        if(detect_forward()) //OOXOO
        {
          currentStateMap = FORWARD_MAP;
        }

        if(detect_all_black()) //XXXXX
        {
          currentStateMap = END;
        }
*/
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

        robot.followLine(); //mudar;
        //Only real use of the PID
        break;

    case U_TURN:
        //U-turn function 
        robot.u_turn();// mudar;
        //Just angular speed until -180 degrees (use encoders)
        break;

    case LEFT_TURN_MAP:
        //left_turn function 
        robot.left_turn(); //mudar;
        //Just angular speed until -90 degrees (use encoders)
        break;

    case RIGHT_TURN_MAP:
        //right_turn function
        robot.right_turn();// mudar;
        //Just angular speed until 90 degrees (use encoders)
        break;

    case REVERSE:
        //reverse function 
        robot.reverse(); //mudar;
        //activate motors in oposite until see line (in this case, since its always turning left, it will see either OOXXX or OOXOO)
        break;

    case SMALL_FORWARD:
        //small_forward function (maybe just forward function but with timer)
        robot.forward(); //mudar;
        break;

    case FORWARD_MAP:
        //forward function (go forward until OOXOO)
        robot.forward(); //mudar;
        break;

    case END:
        //run solving algorithm on path taken
       // solve_algorithm();
        //stop robot
        robot.stop(); //mudar;
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
/*
        if(Detect_node)
        {
          currentStateSolve  = GET_INSTRUCTION;
        }*/
			break;




			case GET_INSTRUCTION:

        #ifdef DEBUG
        printf("-- Current state solve = GET_INSTRUCTION\n");
        #endif
/*
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
        }*/

			break;




			case RIGHT_TURN_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = RIGHT_TURN_SOLVE\n");
        #endif
/*
        if(back_on_line)
        {
          currentStateSolve = FOLLOW_LINE_SOLVE;
        }*/

			break;



    case LEFT_TURN_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = LEFT_TURN_SOLVE\n");
        #endif
/*
        if(back_on_line)
        {
          currentStateSolve = FOLLOW_LINE_SOLVE;
        }*/

      break;

      case FORWARD_SOLVE:

        #ifdef DEBUG
        printf("-- Current state solve = FORWARD_SOLVE\n");
        #endif
/*
        if(OOXOO)
        {
          currentStateSolve = FOLLOW_LINE_SOLVE;
        }
*/
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
        //stop robot
        break;

    case FOLLOW_LINE_SOLVE:
        //follow_line function
        break;

    case GET_INSTRUCTION:
        //get_instruction function 
        //char currentInstruction = pathway.top(); 
        //pathway.pop();
        break;

    case RIGHT_TURN_SOLVE:
        //right_turn function
        //same as the one used in mapping
        break;

    case LEFT_TURN_SOLVE:
        //left_turn function
        //same as the one used in mapping
        break;

    case FORWARD_SOLVE:
        //forward function
        break;

    case FINISH:
        //stop robot / or any finish task
        break;

    default:
        break;
    }

}

#endif

#ifdef TEST

void Test_FSM_Handler()
{


	static bool next_turn_left = true;


	// Serial.printf("RE_START_BUTTON %d\n", re_START_BUTTON);
	// Serial.println();

    switch(currentStateTest)
    {
    case FORWARD_TEST:

    #ifdef DEBUG
        Serial.print("-- Current state test = FORWARD\n");
        #endif

        if(re_START_BUTTON == 1)
        {
          currentStateTest = BACWARD_TEST;
        }
        break;


	case RIGHT_TURN_TEST:

	#ifdef DEBUG
		Serial.printf("-- Current state test = RIGHT_TURN  and next_turn_left = %d\n", next_turn_left);
		#endif

		

		if(robot.END_TURN == true) //signal from right turn function that means it has finished turning
		{
			currentStateTest = STOP_TEST;
			robot.END_TURN = false;
			next_turn_left = true;
		}
		break;

	case LEFT_TURN_TEST:

	#ifdef DEBUG
		Serial.printf("-- Current state test = LEFT_TURN and next_turn_left = %d\n", next_turn_left);
		#endif

		

		if(robot.END_TURN == true ) //signal from left turn function that means it has finished turning
		{
			currentStateTest = STOP_TEST;
			robot.END_TURN = false;
			next_turn_left = false;
		}
		break;

	case U_TURN_TEST:

	#ifdef DEBUG
		Serial.print("-- Current state test = U_TURN\n");
		#endif

		if(robot.END_TURN == true) //signal from u turn function that means it has finished turning
		{
			currentStateTest = STOP_TEST;
			robot.END_TURN = false;
		}
		break;

    case BACWARD_TEST:

    #ifdef DEBUG
        Serial.print("-- Current state test = BACKWARD\n");
        #endif


        if(re_START_BUTTON == 1)
        {
          currentStateTest = STOP_TEST;
        }
        break;

		case STOP_TEST:

			#ifdef DEBUG
			Serial.printf("-- Current state test = STOP and next turn left = %d\n", next_turn_left);
			#endif

			robot.END_TURN = false; // extra safety

			if(re_START_BUTTON == 1 && next_turn_left)
			{
				currentStateTest = LEFT_TURN_TEST;
			}
			else if(re_START_BUTTON == 1 && !next_turn_left)
			{
				currentStateTest = RIGHT_TURN_TEST;
			}

				break;
	}

    // Update outputs

    switch(currentStateTest)
    {
    case FORWARD_TEST:
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

    case BACWARD_TEST:
        robot.reverse();
        break;

	case STOP_TEST:
		robot.stop();
		break;
    }
  }

#endif
