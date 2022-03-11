// Bump.c
// Runs on MSP432
// Provide low-level functions that interface bump switches the robot.
// Daniel Valvano and Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Negative logic bump sensors
// P4.7 Bump6, left side of robot
// P4.6 Bump5
// P4.5 Bump4
// P4.3 Bump3
// P4.2 Bump2
// P4.0 Bump1, right side of robot

#include <stdint.h>
#include "msp.h"
#include "..\inc\CortexM.h"
#include "..\inc\TExaS.h"
#include "..\inc\Clock.h"

//Initialize Edge trigger
void EdgeTrigger_Init(void){
  P4->SEL0 &= ~0xED;
  P4->SEL1 &= ~0xED;    //  P4.0,2,3,5,6,7 as GPIO 1110 1101
  P4->DIR &= ~0xED;     //  P4.0,2,3,5,6,7 as input
  P4->REN |= 0xED;      //  P4.0,2,3,5,6,7 as pull-up resistor
  P4->OUT |= 0xED;    //set output high for P4.0,2,3,5,6,7, to get falling edge trigger
  P4->IES |= 0xED;    //falling edge event
  P4->IFG &= ~0xED;   //clear flags 0,1,3,5,6,7
  P4->IE |= 0xED;     //arm interrupts on P4.0,2,3,5,6,7

  //Set to priority 1
  //NVIC->IP[9] = (NVIC->IP[9] & 0x00FFFFFF) | 0x10000000;
  NVIC->IP[9] = (NVIC->IP[9] & 0xFF00FFFF) | 0x00200000;
  //Enable interrupt 35 in NVIC
  //NVIC->ISER[1] = 0x00000008;
  NVIC->ISER[1] = 0x00000040;     //Enable Interrupt 38

  //EnableInterrupts();                // clear the I bit
}

//Bumper switch has been hit, interupt flagged,
//Call this function to reset
//uint8_t P4_Out;
//uint8_t status;
void PORT4_IRQHandler(void){
  //-- P4->IV Register values when SWX is pressed --\\
  //    SW6     SW5     SW4     SW3     SW2     SW1
  //  0x0010  0x000E  0x000C  0x0008  0x0006  0x0002
  P4->IFG &= ~0xED;   //clear flags 0,1,3,5,6,7
  Motor_Stop();       //stop motors
  Motor_Backward(2000, 2000);
  Clock_Delay1ms(1000);
  Motor_Stop();


  /*  status = P4->IV;
    if(status == 0x0002){
        printf("p4.0 flag hit");
        P4->IFG &= ~0xED;   //clear flags 0,1,3,5,6,7
    }
  */
}

// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){

    uint8_t result, p4_data;

    //P4->IN &= 0xED;

    p4_data = P4->IN;  //Read P4.0,2,3,5,6,7 --> 1110 1101 bit mask
    result = p4_data & 0xED;

    return result; // replace this line
}
