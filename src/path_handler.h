/**
 * @file get_path.h
 * @brief Header file for path optimization functions
 */

#ifndef GET_PATH_H
#define GET_PATH_H

#include <stack>
#include <Arduino.h>

using namespace std;

/**
 * @brief Optimizes the path by removing unnecessary movements.
 * 
 * This function processes the original stack of movements and condenses it into a new stack
 * with optimized movements based on specific patterns.
 * 
 * @param original The original stack of movements to be optimized.
 * @return A new stack containing the optimized path.
 */

stack<char> get_path(stack<char> movements_made);

/**
 * @brief Saves the move made by the robot to a stack.
 * 
 * This function pushes the given move character onto the stack of movements made.
 * 
 * @param move The move character to be saved (e.g., 'F', 'L', 'R', etc.).
 */
void save_move(char move, stack<char> movements_made);


/**
 * @brief Retrieves the next move from the stack of movements made.
 *  
 * This function removes and returns the top move from the stack of movements made.
 * 
 * @return The move character that was removed from the stack.
 */

char get_move();

void print_path();

#endif // GET_PATH_H    