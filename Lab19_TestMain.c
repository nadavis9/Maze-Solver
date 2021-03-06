//*****************************************************************************
// RSLK-MAX test main for Lab 19
// MSP432 with RSLK-MAX
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


#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "Motor.h"
#include "../inc/Bump.h"
//#include "../inc/ADC14.h"
//#include "../inc/LPF.h"
#include "../inc/TimerA1.h"
//#include "../inc/IRDistance.h"
//#include "../inc/Nokia5110.h"
#include "Reflectance.h"
//#include "Ultrasonic.h"
//#include "Ultrasonic.c"

// Motor interface, see Motor.c
// IR sensor interface, see IRdistance.c
// bump sensor interface, see Bump.c
// Nokia interface, see Nokia5110.c

void TestMotor(uint16_t duty){uint32_t in;
  do{
    in = LaunchPad_Input();
  }while((in&0x01)==0);  // start on switch 1 press
  Motor_Forward(duty,duty);
  do{
    in = LaunchPad_Input();
  }while((in&0x02)==0);  // stop on switch 2 press
}
// BLE variables
uint8_t JackiCommand=0;
uint8_t JackiBumpSensor=0;
uint8_t JackiUltraSensor=0;
uint16_t JackiSpeed=0;
uint16_t Switch1;       // 16-bit notify data from Button 1
uint32_t Switch2;       // 32-bit notify data from Button 2
uint32_t time=0;
int32_t Left,Center,Right; // IR distances
// ********OutValue**********
// Debugging dump of a data value to virtual serial port to PC
// data shown as 1 to 8 hexadecimal characters
// Inputs:  response (number returned by last AP call)
// Outputs: none
void OutValue(char *label,uint32_t value){
  UART0_OutString(label);
  UART0_OutUHex(value);
}
void ReadCommand(void){ // called on a SNP Characteristic Read Indication for characteristic JackiCommand
  OutValue("\n\rRead JackiCommand=",JackiCommand);
}
void ReadJackiBumpSensor(void){ // called on a SNP Characteristic Read Indication for characteristic JackiSensor
  JackiBumpSensor = Bump_Read();
  OutValue("\n\rRead JackiBumpSensor=",JackiBumpSensor);
}

void ReadJackiUltraSensor(void){ // called on a SNP Characteristic Read Indication for characteristic JackiSensor
  JackiUltraSensor = ultraCenter();
  OutValue("\n\rRead JackiUltraSensor=",JackiUltraSensor);
}

void RunJacki(void){
  if((JackiCommand==0)||(JackiCommand>4)||(Bump_Read())){
    JackiCommand = 0;
    UART0_OutString(" Stop");
    Motor_Forward(0,0);
  }
  if(JackiSpeed>14000){
    JackiSpeed = 1000;
  }
  if(JackiCommand==1){
    UART0_OutString(" Go");
    Motor_Forward(JackiSpeed,JackiSpeed);
  }
  if(JackiCommand==2){
    UART0_OutString(" Back");
    Motor_Backward(JackiSpeed/2,JackiSpeed/2);
    time=0;
  }
  if(JackiCommand==3){
    UART0_OutString(" Right");
    Motor_Right(JackiSpeed/2,JackiSpeed/2);
    time=0;
  }
  if(JackiCommand==4){
    UART0_OutString(" Left");
    Motor_Left(JackiSpeed/2,JackiSpeed/2);
    time=0;
  }
}
void WriteCommand(void){ // called on a SNP Characteristic Write Indication on characteristic JackiCommand
  OutValue("\n\rWrite JackiCommand=",JackiCommand);
  RunJacki();
}
void ReadJackiSpeed(void){ // called on a SNP Characteristic Read Indication for characteristic JackiSpeed
  OutValue("\n\rRead JackiSpeed=",JackiSpeed);
}
void WriteJackiSpeed(void){  // called on a SNP Characteristic Write Indication on characteristic JackiSpeed
  OutValue("\n\rJackiSpeed=",JackiSpeed);
  RunJacki();
}
void Button1(void){ // called on SNP CCCD Updated Indication
  OutValue("\n\rRight IR CCCD=",AP_GetNotifyCCCD(0));
}
void Button2(void){
  OutValue("\n\rLeft IR CCCD=",AP_GetNotifyCCCD(1));
}


void BLE_Init(void){volatile int r;
  UART0_Init();
  EnableInterrupts();
  UART0_OutString("\n\rJacki test project - MSP432-CC2650\n\r");
  r = AP_Init();
  AP_GetStatus();  // optional
  AP_GetVersion(); // optional
  AP_AddService(0xFFF0);
  //------------------------
  JackiCommand = 0;  // read/write parameter
  AP_AddCharacteristic(0xFFF1,1,&JackiCommand,0x03,0x0A,"JackiCommand",&ReadCommand,&WriteCommand);
  //------------------------
  JackiBumpSensor = Bump_Read(); // read only parameter (get from bump sensors)
  AP_AddCharacteristic(0xFFF2,1,&JackiBumpSensor,0x01,0x02,"JackiBumpSensor",&ReadJackiBumpSensor,0);
  //JackiLineSensor = Reflectance_Read(1000); // read only parameter (get from bump sensors)
  //AP_AddCharacteristic(0xFFF2,1,&JackiLineSensor,0x01,0x02,"JackiLineSensor",&ReadJackiLineSensor,0);
  //--------------------------------------
  JackiUltraSensor = ultraCenter(); // read only parameter (get from bump sensors)
  AP_AddCharacteristic(0xFFF3,1,&JackiUltraSensor,0x01,0x02,"JackiUltraSensor",&ReadJackiUltraSensor,0);
  //------------------------
  JackiSpeed = 100;   // read/write parameter
  AP_AddCharacteristic(0xFFF3,2,&JackiSpeed,0x03,0x0A,"JackiSpeed",&ReadJackiSpeed,&WriteJackiSpeed);
  //------------------------
  Switch1 = 0;
  AP_AddNotifyCharacteristic(0xFFF5,2,&Right,"Right IR",&Button1);
  //------------------------
  Switch2 = 0x00000000;
  AP_AddNotifyCharacteristic(0xFFF6,2,&Left,"Left IR",&Button2);
  //------------------------
  AP_RegisterService();
  AP_StartAdvertisementJacki();
  AP_GetStatus(); // optional

}

volatile uint32_t nr,nc,nl;
volatile uint32_t ADCflag;
/*void IRsampling(void){  // runs at 2000 Hz
  uint32_t raw17,raw14,raw16;
  ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
  nr = LPF_Calc(raw17);  // right is channel 17 P9.0
  nc = LPF_Calc2(raw14); // center is channel 14, P6.1
  nl = LPF_Calc3(raw16); // left is channel 16, P9.1
  Left = LeftConvert(nl);
  Center = CenterConvert(nc);
  Right = RightConvert(nr);
  ADCflag = 1;           // semaphore
}*/

void main(void){
  AP_Reset();
  DisableInterrupts();
  uint8_t in,last,bump, in2;
  uint32_t raw17,raw14,raw16;
  int i = 0;
  uint32_t time1=0,time2 = 0;                    // incremented with every pass through main loop
  Clock_Init48MHz();
  PWM_Init(15000,0,0);
  LaunchPad_Init(); // built-in switches and LEDs
  Bump_Init(); // bump switches
  Reflectance_Init();
  in = Bump_Read();
  in2 = Reflectance_Read(1000);
  Motor_Stop();
  Timer_Init();
  P6->SEL0 &= ~0x40;
  P6->SEL1 &= ~0x40;
  P6->DIR |= 0x40;
  P6->OUT &= ~0x40;

  P4->SEL0 &= ~0x30;
  P4->SEL1 &= ~0x30;
  P4->DIR |= 0x30;
  P4->OUT &= ~0x30;

  //Motor_Forward(8000, 8000);
  //Nokia5110_Init();
  //Nokia5110_Clear();
  //Nokia5110_OutString("Priming...  left      mmcenter    mmright     mm            num");
  // prime the arrays with some samples
  // IRDistance_Init();
  /*ADC0_InitSWTriggerCh17_14_16();   // initialize channels 17,14,16
  ADC_In17_14_16(&raw17,&raw14,&raw16);  // sample
  LPF_Init(raw17,512);     // P9.0/channel 17
  LPF_Init2(raw14,512);    // P6.1/channel 14
  LPF_Init3(raw16,512);    // P9.1/channel 16*/


  // switch to Average mode
  //Nokia5110_SetCursor(0, 0);             // zero leading spaces, first row
  //Nokia5110_OutString("Average     ");
  //Nokia5110_SetCursor(0, 5);             // zero leading spaces, sixth row
  //Nokia5110_OutString("Bump        ");
  BLE_Init();
  //TimerA1_Init(&IRsampling,250);    // 2000 Hz sampling
  EnableInterrupts();
  last = LaunchPad_Input();
	// write this as part of Lab 19
  //EnableInterrupts();
  
  while(1){
   // write this as part of Lab 19
      time++; time1++;
          AP_BackgroundProcess();  // handle incoming SNP frames
          // if bump sensor, then stop
          if(((JackiCommand>1)&&(time>1000000))||(JackiCommand>4)||(Bump_Read())){
            JackiCommand=0;
            Motor_Stop();
          }
          /*if(time1>1000000){
            time1 = 0;
            Switch1 = LaunchPad_Input()&0x01;   // Button 1
            if(AP_GetNotifyCCCD(0)){
              OutValue("\n\rNotify Right IR=",Right);
              AP_SendNotification(0);
            }
            Switch2 = (LaunchPad_Input()>>1)&0x01;   // Button 2
            if(AP_GetNotifyCCCD(1)){
              OutValue("\n\rNotify Left IR=",Left);
              AP_SendNotification(1);
            }
          }*/
          in = LaunchPad_Input();
          if(((last&0x01)==0)&&(in&0x01)){
            Clock_Delay1ms(2);  // debounce
            if(JackiCommand==0){
              JackiCommand = 1;
              RunJacki();
            }
          }
          if(((last&0x02)==0)&&(in&0x02)){
            Clock_Delay1ms(2);  // debounce
            if(JackiSpeed<12000){
              JackiSpeed += 2000;
            }else{
              JackiSpeed = 1000;
            }
            RunJacki();
          }
          if(((last&0x03))&&(in&0x03)!=0x03){
            Clock_Delay1ms(2);  // debounce
          }
          last = in;
          time2 = time2 + 1;
          /*if((time2%17777) == 0){              // calibration value is basically a guess to get about 10 Hz
            time2 = 0;
            // take IR distance measurements
            LaunchPad_Output((i&0x01)<<2);     // toggle the blue LED
            // print IR distance average
            //Nokia5110_SetCursor(5, 1);         // five leading spaces, second row
            //Nokia5110_OutUDec(Left);
            //Nokia5110_SetCursor(5, 2);         // five leading spaces, third row
            //Nokia5110_OutUDec(Center);
            //Nokia5110_SetCursor(5, 3);         // five leading spaces, fourth row
            //Nokia5110_OutUDec(Right);
            // print the status of the bump sensors
            bump = Bump_Read();
            Nokia5110_SetCursor(5, 5);           // five leading spaces, sixth row
            if((bump&0x01) == 0){
              Nokia5110_OutChar('0');
            }else{
              Nokia5110_OutChar(' ');
            }
            if((bump&0x02) == 0){
              Nokia5110_OutChar('1');
            }else{
              Nokia5110_OutChar(' ');
            }
            if((bump&0x04) == 0){
              Nokia5110_OutChar('2');
            }else{
              Nokia5110_OutChar(' ');
            }
            if((bump&0x08) == 0){
              Nokia5110_OutChar('3');
            }else{
              Nokia5110_OutChar(' ');
            }
            if((bump&0x10) == 0){
              Nokia5110_OutChar('4');
            }else{
              Nokia5110_OutChar(' ');
            }
            if((bump&0x20) == 0){
              Nokia5110_OutChar('5');
            }else{
              Nokia5110_OutChar(' ');
            }*/
          }
  }
//}
