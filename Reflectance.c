// Reflectance.c
// Provide functions to take measurements using the kit's built-in
// QTRX reflectance sensor array.  Pololu part number 3672. This works by outputting to the
// sensor, waiting, then reading the digital value of each of the
// eight phototransistors.  The more reflective the target surface is,
// the faster the voltage decays.
// Daniel and Jonathan Valvano
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

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)

#include <stdint.h>
#include "msp432.h"
#include "..\inc\Clock.h"

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
void Reflectance_Init(void)
{
    P5->SEL0 &= ~0x08;          // Sets P5.3 as GPIO - CTRL EVEN
    P5->SEL1 &= ~0x08;          // Sets P5.3 as GPIO

    P5->DIR |= 0x08;            // Sets P5.3 to output
    P5->REN &= ~0x08;           // Disables pull up resistor for P5.3

    P5->OUT &= ~0x08;           // Sets output to low

    P9->SEL0 &= ~0x04;          // Sets P9.2 as GPIO - CTRL ODD
    P9->SEL1 &= ~0x04;          // Sets P9.2 as GPIO

    P9->DIR |= 0x04;            // Sets P9.2 to output
    P9->REN &= ~0x04;           // Disables pull up resistor for P9.2

    P9->OUT &= ~0x04;           // Sets output to low

    P7->SEL0 = 0x00;            // Sets P7 as GPIO - All 8 used for LINE SENSOR
    P7->SEL1 = 0x00;            // Sets P7 as GPIO

    P7->DIR = 0x00;             // Sets P7 as input
    P7->REN = 0x00;             // Sets P7 as input

}



// ------------Reflectance_Read------------
// Read the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Read(uint32_t time)
{

    // write this as part of Lab 6
    //set p5.3 and p9.2 high
    P5->OUT |= 0x08; //or P5 out with 0000 1000 to make it high
    P9->OUT |= 0x04; //0000 0100 making p9.2 high

    //Making p7.0  output and setting  high
      // P7->SEL0 &= 0xFE; //1111 1110
      // P7->SEL1 &= 0xFE; //1111 1110
      // P7->DIR |= 0x01; //OUTPUT
       //P7->REN &= 0xFE; //disable pull up resistor on p7.0
       //P7->OUT |= 0x01; //setting p7.0 high


    //Making p7.0 to p7.7 outputs and setting them to high
    //P7->SEL0 &= 0x00; //0000 0000
    //P7->SEL1 &= 0x00; //0000 0000
    P7->DIR |= 0xFF; //OUTPUT bits high
    P7->REN |= 0xFF; //enable pull up resistor on p7.0
    P7->OUT |= 0xFF; //setting p7.0 high

    //Wait 10us
    Clock_Delay1us(10);

    //Making p7.0 an input
       // P7->SEL0 &= 0xFE; //0000 0000 another way of making it a GPIO pin
        //P7->SEL1 &= 0xFE; //0000 000
        //P7->DIR &= 0xFE; //0 to be input 1111 1110
       // P7->REN |= 0x01; //enable pull up resistor on p7

    //Making p7.0 to p7.7 inputs
    //P7->SEL0 &= 0x00; //0000 0000 another way of making it a GPIO pin
    //P7->SEL1 &= 0x00; //0000 000
    P7->DIR &= 0x00; //0 to be input 1111 1110
    P7->REN &= 0x00; //disable pull up resistor on p7

    Clock_Delay1us(time);
    uint8_t value;
    value = P7->IN;
    //uint8_t value;
    //Loop
    /*for(int i = 0; i < 10000; i++){
        uint8_t value;
        value = P7->IN & 0x01; //bit mask to p7.0
        P4->OUT = value;

    }*/

  P5->OUT &= 0xF7; //setting 5.3 low
  P9->OUT &= 0xFB; //setting 9.2 low
  //Clock_Delay1ms(10);

  //uint8_t result = P4->OUT; // replace this line
  return value;

}

// ------------Reflectance_Center------------
// Read the two center sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// wait t us
// Read sensors
// Turn off the 8 IR LEDs
// Input: time to wait in usec
// Output: 0 (off road), 1 off to left, 2 off to right, 3 on road
// (Left,Right) Sensors
// 1,1          both sensors   on line
// 0,1          just right     off to left
// 1,0          left left      off to right
// 0,0          neither        lost
// Assumes: Reflectance_Init() has been called
uint8_t Reflectance_Center(uint32_t time){
    // write this as part of Lab 6
  return 0; // replace this line
}


// Perform sensor integration
// Input: data is 8-bit result from line sensor
// Output: position in 0.1mm relative to center of line
int32_t Reflectance_Position(uint8_t data)
{
    // write this as part of Lab 6
    int num;
    int den;
    int distance;
    uint8_t i, j;
    int a;
    int W[8] = {334, 238, 142, 48, -48, -142, -238, -334};
    uint8_t bitMask[8] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};

    num = 0;
    den = 0;

    for (i=0;i<8;i++)
    {

        a = data & bitMask[i];

        num = num + (a * W[i]);

    }

    for (j=0;j<8;j++)
    {
        den = den + (data & bitMask[j]);
    }

    distance = num / den;


 return distance; // replace this line
}


// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){

         P5->OUT |= 0x08;           // Set P5.3 HI, turns on CTRL EVEN
         P9->OUT |= 0x04;           // Set P9.2 HI, turns on CTRL ODD

         P7->DIR = 0xFF;            // Sets P7.all as output
         P7->REN = 0xFF;            // Enables pull up resistor for P7.all
         P7->OUT = 0xFF;            // Sets P7.all output high, charges capacitor

         Clock_Delay1us(10);        // Waiting for 10us

         P7->DIR = 0x00;            // Sets P7 as input
         P7->REN = 0x00;            // Sets P7 as input

}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    uint8_t result, lineResult, resultMSB, resultLSB, centerCheck;
    lineResult = 0;
   lineResult = P7->IN;        //& 0xFF; // Read in result from P7.all and store to "result"

   /*
   centerCheck = lineResult & 0x18;

           if(centerCheck == 0x18)
           {
               result = 0x03;      // Output is move straight; turn on both motors
           }
           //if(lineResult == 0)
           //{
           //    result = 0x00;
           //}
           else
           {
               resultMSB = lineResult & 0xF0;
               resultMSB = resultMSB>>4;       // Right shift MSB by 4, 1111XXXX -> XXXX1111
               resultLSB = lineResult & 0x0F;

               if(resultMSB>resultLSB)
               {
                   result = 0x01;  // Output is turn left; turn on right motor only
               }
               if(resultLSB>resultMSB)
               {
                   result = 0x02;  // Output is turn right; turn on left motor only
               }else{
                   result = 0x03;
               }
           }

*/


   P5->OUT &= ~0x08;       // Turns off IR LED P5.3
   P9->OUT &= ~0x04;       // Turns off IR LED P9.2
   //return result;
   return lineResult;
}
