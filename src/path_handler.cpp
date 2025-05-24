#include <stack>
#include <string>
#include <Arduino.h>

//#define FOLLOW LEFT
#define FOLLOW RIGHT
//#define DEBUG

using namespace std;

// Function to get the path from the original stack
// This function will process the stack and return a new stack with the optimized path
stack<char> get_path(stack <char> movements_made) 
{
  stack<char> tempStack;
  bool changed = true;
    
  while (changed) {
    changed = false;
    // Transfer original to tempStack while checking for patterns
    while (!movements_made.empty()) {
      char top = movements_made.top();
      movements_made.pop();
      tempStack.push(top);
            
      // Check if we have at least 3 elements to form a pattern
      if (tempStack.size() >= 3) {
        // Get the top three elements
        char third = tempStack.top(); tempStack.pop();
        char second = tempStack.top(); tempStack.pop();
        char first = tempStack.top(); tempStack.pop();
                
        string pattern = {first, second, third};
                
        #ifdef FOLLOW LEFT 
        // Check for left-hand patterns 
        if (pattern == "LUL") {
          tempStack.push('F');
          changed = true;
        } else if (pattern == "FUL") {
          tempStack.push('R');
          changed = true;
        } else if (pattern == "LUF") {
          tempStack.push('R');
          changed = true;
        } else if (pattern == "RUL") {
          tempStack.push('U');
          changed = true;
        } else if (pattern == "LUR") {
          tempStack.push('U');
          changed = true;
        } else {
          tempStack.push(first);
          tempStack.push(second);
          tempStack.push(third);
        }
        #endif

        #ifdef FOLLOW RIGHT
        // Check for right-hand patterns
        if (pattern == "RUR") {
          tempStack.push('F');
          changed = true;
        } else if (pattern == "FUR") {
          tempStack.push('L');
          changed = true;
        } else if (pattern == "RUF") {
          tempStack.push('L');
          changed = true;
        } else if (pattern == "LUR") {
          tempStack.push('U');
          changed = true;
        } else if (pattern == "RUL") {
          tempStack.push('U');
          changed = true;
        } else {
          tempStack.push(first);
          tempStack.push(second);
          tempStack.push(third);
        }
        #endif
      }
    }
        
    // Transfer back to original stack for next iteration
    while (!tempStack.empty()) {
      movements_made.push(tempStack.top());
      tempStack.pop();
    }
  }

  // Final reversal for correct output order
  stack<char> final_path;
  while (!movements_made.empty()) {
    final_path.push(movements_made.top());
    movements_made.pop();
  }

  #ifdef DEBUG
    Serial.print("Final Path: ");
    stack<char> debug_copy;
    // Print and save to debug_copy to restore later
    while (!final_path.empty()) {
      char c = final_path.top();
      Serial.print(c);
      debug_copy.push(c);
      final_path.pop();
    }
    Serial.println();
    // Restore final_path from debug_copy
    while (!debug_copy.empty()) {
      final_path.push(debug_copy.top());
      debug_copy.pop();
    }
  #endif

  // Return the final optimized path
  return final_path;
}

// Function to save the last move made by the robot
// This function will be called when the robot makes a move
void save_move(char move, stack<char> movements_made)
{
  // Save the move to the stack
  movements_made.push(move);
  
  #ifdef DEBUG
    Serial.print("Move saved: ");
    Serial.println(move);
  #endif
}

// Function to print the path
// void print_path()
// {
//   // Print the path from a copy of movements_made
//   stack<char> tempStack = movements_made;  // Make a temporary copy to print the contents
//   Serial.print("Path taken: ");
//   while (!tempStack.empty()) {
//     char c = tempStack.top();
//     Serial.print(c);
//     tempStack.pop();
//   }
//   Serial.println();
// }

