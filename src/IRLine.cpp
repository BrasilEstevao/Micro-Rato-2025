/* Copyright (c) 2019  Paulo Costa
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

#include "IRLine.h"
#include "Arduino.h"
#include <string.h>
#include "config.h"
#include "robot.h"

#define DEBUG 1

extern robot_t robot;

IRLine_t::IRLine_t()
{
  IR_WaterLevel = 0;
  IR_tresh = 512;
  cross_tresh = 3;
  black_cross_level = 2.8;
}

void IRLine_t::calibrate(void)
{
  
}


void IRLine_t::calcIRLineEdgeLeft(void)
{
  byte c, found;
  int v, last_v;

  found = 0;
  IR_max = 0;
  pos_left = 2 * 16.0;
  total = 0;
  last_v = 0;
  for (c = 0; c < 5; c++) {
    v = IR_values[c] - IR_WaterLevel;
    if (v < 0) v = 0;
    if (v > IR_max) IR_max = v;
    total = total + v;

    if (!found && last_v < IR_tresh && v > IR_tresh) {
      pos_left = -12 + 16.0 * (c - 2) + 16.0 * (IR_tresh - last_v) / (v - last_v);
      found = 1;
    }
    last_v = v;
  }
}


void IRLine_t::calcIRLineEdgeRight(void)
{
  byte c, found;
  int v, last_v;

  found = 0;
  IR_max = 0;
  pos_right = -2 * 16.0;
  total = 0;
  last_v = 0;
  for (c = 0; c < 5; c++) {
    v = IR_values[4 - c] - IR_WaterLevel;
    if (v < 0) v = 0;
    if (v > IR_max) IR_max = v;
    total = total + v;

    if (!found && last_v < IR_tresh && v > IR_tresh) {
      pos_right = -(-12 + 16.0 * (c - 2) + 16.0 * (IR_tresh - last_v) / (v - last_v));
      found = 1;
    }
    last_v = v;
  }
  
}



static void adc_set_channel(int channel)
{
	gpio_put_masked(digitalPinToBitMask(MUXA_PIN) | digitalPinToBitMask(MUXB_PIN) | digitalPinToBitMask(MUXC_PIN), channel << MUXA_PIN);
  //digitalWrite(MUXA_PIN, channel & 1);
  //digitalWrite(MUXB_PIN, (channel >> 1) & 1);
  //digitalWrite(MUXC_PIN, (channel >> 2) & 1);
}

uint16_t read_adc(int channel)
{
	adc_set_channel(channel); // Switch external MUX to the desired channel
  delayMicroseconds(100);
	return analogRead(A2);    // The mux connects to analog input A2
}

void IRLine_t::readIRSensors(void)
{
  byte c;  // Read the five IR sensors using the AD converter
  for (c = 0; c < IRSENSORS_COUNT; c++) 
  {
    robot.IRLine.IR_values[(IRSENSORS_COUNT - 1) -c] = 1023 - read_adc(3 + c);
  }
  //Serial.println();
}


uint32_t IRLine_t::encodeIRSensors(void)
{
  byte c;                                           // Encode five IR sensors with 6 bits for each sensor
  uint32_t result = robot.IRLine.IR_values[0] >> 4; // From 10 bits to 6 bits
  for (c = 1; c < 5; c++)
  {
    result = (result << 6) | (robot.IRLine.IR_values[c] >> 4);
  }
  return result;
}







void IRLine_t::printIRLine(void)
{
  Serial.print(robot.IRLine.IR_values[0]);
  Serial.print("               ");
  Serial.print(robot.IRLine.IR_values[1]);
  Serial.print("               ");
  Serial.print(robot.IRLine.IR_values[2]);
  Serial.print("               ");
  Serial.print(robot.IRLine.IR_values[3]);
  Serial.print("               ");
  Serial.println(robot.IRLine.IR_values[4]);
}



//Node detection
char IRLine_t::detectNode(void)
{
  char node;

  //left -> XXXOO
  if(IR_values[0] > IR_tresh && IR_values[1] > IR_tresh && IR_values[2] > IR_tresh && IR_values[3] < IR_tresh && IR_values[4] < IR_tresh)
  {
    node = 'L';
  }
  
  //right -> OOXXX
  else if(IR_values[0] < IR_tresh && IR_values[1] < IR_tresh && IR_values[2] > IR_tresh && IR_values[3] > IR_tresh && IR_values[4] > IR_tresh)
  {
    node = 'R';
  }
  
  //T junction, Cross junction or End (All black -> XXXXX)
  else if(IR_values[0] > IR_tresh && IR_values[1] > IR_tresh && IR_values[2] > IR_tresh && IR_values[3] > IR_tresh && IR_values[4] > IR_tresh)
  {
    node = 'B';  //B for black
  }

  //After T or after U-Turn (All white -> OOOOO)
  else if (IR_values[0] < IR_tresh && IR_values[1] < IR_tresh && IR_values[2] < IR_tresh && IR_values[3] < IR_tresh && IR_values[4] < IR_tresh)
  {
    node = 'W';  //W for white
  }

  //Normal line -> XOOOO or OXOOO or OOXOO or OOOXO or OOOOX  
  else if ((IR_values[0] < IR_tresh && IR_values[1] > IR_tresh && IR_values[2] < IR_tresh && IR_values[3] < IR_tresh && IR_values[4] < IR_tresh) ||   // OXOOO
           (IR_values[0] < IR_tresh && IR_values[1] < IR_tresh && IR_values[2] < IR_tresh && IR_values[3] > IR_tresh && IR_values[4] < IR_tresh) ||   // OOOOX
           (IR_values[0] < IR_tresh && IR_values[1] < IR_tresh && IR_values[2] > IR_tresh && IR_values[3] < IR_tresh && IR_values[4] < IR_tresh) ||   // OOOXO
           (IR_values[0] > IR_tresh && IR_values[1] < IR_tresh && IR_values[2] < IR_tresh && IR_values[3] < IR_tresh && IR_values[4] < IR_tresh))     // XOOOO
  {
    node = 'N'; //N for normal
  }
   
  else 
  {
    node = 'E';
  }




  #ifdef DEBUG
  Serial.print("Node: ");
  Serial.println(node);
  #endif
  return node;
}