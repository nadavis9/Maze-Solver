// TA3InputCapture.c
// Runs on MSP432
// Use Timer A3 in capture mode to request interrupts on rising
// edges of P10.4 (TA3CCP0) and P8.2 (TA3CCP2) and call user
// functions.
// Use Timer A3 in capture mode to request interrupts on rising
// edges of P10.4 (TA3CCP0) and P10.5 (TA3CCP1) and call user
// functions.
// Daniel Valvano
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

// external signal connected to P10.5 (TA3CCP1) (trigger on rising edge)
// external signal connected to P10.4 (TA3CCP0) (trigger on rising edge)

#include <stdint.h>
#include "msp.h"

void ta3dummy(uint16_t t){};       // dummy function
void (*CaptureTask0)(uint16_t time) = ta3dummy;// user function
void (*CaptureTask1)(uint16_t time) = ta3dummy;// user function



//------------TimerA3Capture_Init01------------
// Initialize Timer A3 in edge time mode to request interrupts on
// the rising edges of P10.4 (TA3CCP0) and P10.5 (TA3CCP1).  The
// interrupt service routines acknowledge the interrupt and call
// a user function.
// Input: task0 is a pointer to a user function called when P10.4 (TA3CCP0) edge occurs
//              parameter is 16-bit up-counting timer value when P10.4 (TA3CCP0) edge occurred (units of 0.083 usec)
//        task1 is a pointer to a user function called when P10.5 (TA3CCP1) edge occurs
//              parameter is 16-bit up-counting timer value when P10.5 (TA3CCP1) edge occurred (units of 0.083 usec)
// Output: none
// Assumes: low-speed subsystem master clock is 12 MHz



void TimerA3Capture_Init01(void(*task0)(uint16_t time), void(*task1)(uint16_t time)){

    // Configure P10.4 and P10.5 as inputs
    P10->SEL0 |= 0x30;
    P10->SEL1 &= ~0x30;
    P10->DIR &= ~0x30;
    //
    //Configure P5.0 and P5.2 as inputs
    P5->SEL0 &= ~0x05;
    P5->SEL1 &= ~0x05;
    P5->DIR &= ~0x05;
    //
    TIMER_A3->CTL &= ~0x0030; // Halt Timer A3
    TIMER_A3->CTL = 0x0240; // Configure Timer A3
    // Configure capture compare registers for submodules TA3.0 and TA3.1
    TIMER_A3->CCTL[0] = 0x4910;
    TIMER_A3->CCTL[1] = 0x4910;
    //
    TIMER_A3->EX0 &= ~0x0007; // post scale clock divide by 1
    // Enable positive edge triggered interrupts on pins P10.4 and P10.5 each with priority 2
    NVIC->IP[3] = (NVIC->IP[3] & 0x0000FFFF) | 0x40400000 ;
    NVIC->ISER[0] = 0x0000C000;
    //
    TIMER_A3->CTL |= 0x0024; //Reset and start Timer A3
}



uint16_t First, Period_Right, Speed_Right;
int32_t Right_Direction;
uint16_t direction_dataR;

void TA3_0_IRQHandler(void){

    direction_dataR = P5->IN & 0x05; // Read motor direction data


    TIMER_A3->CCTL[0] &= ~0x0001; //Acknowledge interrupt flag
    Period_Right = TIMER_A3->CCR[0] - First; // Calculate period of encoder output

    First = TIMER_A3->CCR[0]; // Load time of first rising edge
    Speed_Right = 2000000 / Period_Right; //Convert encoder period to rpm

    // Determine direction of wheel
    if(direction_dataR == 0x05){
        Right_Direction = Right_Direction + 1;
    }
if (direction_dataR == 0x00) {
    Right_Direction = Right_Direction - 1;
}
}




uint16_t First1, Period_Left, Speed_Left;
uint16_t direction_dataL;
int32_t Left_Direction;

void TA3_N_IRQHandler(void){

    direction_dataL = P5->IN & 0x05; //Read motor direction data


    TIMER_A3->CCTL[1] &= ~0x0001; //Acknowledge interrupt flag
    Period_Left = TIMER_A3->CCR[1] - First1; // Calculate period of encoder output


    First1 = TIMER_A3->CCR[1]; // Load time of first rising edge
    Speed_Left = 2000000 / Period_Left; //Convert encoder period to rpm

    // Determine direction of wheel
    if(direction_dataL == 0x05){
       Left_Direction = Left_Direction + 1;
    }
    if (direction_dataL == 0x00) {
   Left_Direction = Left_Direction - 1;

    }
}



//------------TimerA3Capture_Init02------old robot version------
// Initialize Timer A3 in edge time mode to request interrupts on
// the rising edges of P10.4 (TA3CCP0) and P8.2 (TA3CCP2).  The
// interrupt service routines acknowledge the interrupt and call
// a user function.
// Input: task0 is a pointer to a user function called when P10.4 (TA3CCP0) edge occurs
//              parameter is 16-bit up-counting timer value when P10.4 (TA3CCP0) edge occurred (units of 0.083 usec)
//        task2 is a pointer to a user function called when P8.2 (TA3CCP2) edge occurs
//              parameter is 16-bit up-counting timer value when P8.2 (TA3CCP2) edge occurred (units of 0.083 usec)
// Output: none
// Assumes: low-speed subsystem master clock is 12 MHz
void TimerA3Capture_Init02(void(*task0)(uint16_t time), void(*task2)(uint16_t time)){


}


void TimerA2_Init(uint16_t Time){



    TIMER_A2->CTL = 0x0280;
    TIMER_A2->CCTL[0] = 0x0010;
    TIMER_A2->CCR[0] = (Time - 1);
    TIMER_A2->EX0 = 0x0005;
    NVIC->IP[3] = ( NVIC->IP[3] & 0xFFFFFF00)| 0x00000040;
    NVIC->ISER[0] = 0x00001000;
    TIMER_A2->CTL |= 0x0014;

}

int16_t  Error_Calc(void) {
    
    int16_t Speed_difference;
    
    Speed_difference = Speed_Left - Speed_Right;
    
    return Speed_difference;
    
    
    
    
}



