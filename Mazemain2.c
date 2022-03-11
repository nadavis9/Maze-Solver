// ---------------
//good
#include <stdint.h>
//#include <stdio.h>
#include "msp.h"
#include "CortexM.h"
#include "Clock.h"
#include "SysTickInts.h"
#include "Bump.h"
//#include "Motor.h"
#include "../inc/Tachometer.h"
#include "Ultrasonic.h"
#include "LPF.h"


void Timer_Init(void);


#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))
#define P1_0 (*((volatile uint8_t *)(0x42098040)))



//------------------------------------------------------------------------
// Tachometer Functions
uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0;               // Timer A3 first edge, P10.4
int Done0;                     // set each rising
//Period Measure Functions for Tachometer

void PeriodMeasure0(uint16_t time){
  P2_0 = P2_0^0x01;            // thread profile, P2.0
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;               // setup for next
  Done0 = 1;
}
uint16_t Period1;              // (1/SMCLK) units = 83.3 ns units
uint16_t First1;               // Timer A3 first edge, P10.5
int Done1;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us




void PeriodMeasure1(uint16_t time){
  P1_0 = P1_0^0x01;            // thread profile, P1.0
  Period1 = (time - First1)&0xFFFF; // 16 bits, 83.3 ns resolution
  First1 = time;               // setup for next
  Done1 = 1;
}


//---------------------------------------------------------------------------

uint32_t tach_error;
uint32_t leftS, rightS;

//Proportional Control Global
#define PWMNOMINAL 3000
#define SWING 750
#define DESIRED 45
int32_t UR=2500, UL=2500, mode=0, Error,tempE, Kp=5, SetPoint=250, Ki = 10, Ki_error_sum = 0, Ki_count = 0, Ki_error_result = 0, Kp_tach = 2;
int32_t Kpt = 5, Kit = 15, Kit_error_sum = 0, Kit_count = 0, Kit_error_result = 0;
uint32_t center, left, right, Ocenter, Oleft, Oright;

uint16_t centerFirst, centerPeriod, leftFirst, leftPeriod, rightFirst, rightPeriod, centerDistance, leftDistance, rightDistance;
uint8_t i = 0;
enum TachDirection leftDir;
enum TachDirection rightDir;
int32_t leftSteps, rightSteps, leftDiff, rightDiff;
uint16_t leftTach, rightTach;
uint32_t left_old, right_old, center_old;
int state;



void main(void){
    DisableInterrupts();    //This is here because the Lab17 solutions had it
    //EdgeTrigger_Init();     //Initialize the Bump Sensors
    Clock_Init48MHz();      //Initialize Clock
    Timer_Init();
    Tachometer_Init();
    Motor_Init();
    //TimerA3Capture_Init01(&PeriodMeasure0, &PeriodMeasure1);

     //100Hz
    //UR = UL = PWMNOMINAL;    // reset parameters
    EnableInterrupts();
    //SysTick_Init(480000,3);
               //Initialy7ize TimerA2 for ultrasonic Sensors
    LPF_Init(45, 5); //LPF left ultrasonic
    LPF_Init2(45, 5); //filter right ultrasonic
    LPF_Init5(200, 5); //filter for center ultrasonic sensor
    LPF_Init3(2500, 16); //left tachometer
    LPF_Init4(2500, 16); //right tachometer


    //Bluetooth
    P6->SEL0 &= ~0x40;
    P6->SEL1 &= ~0x40;
    P6->DIR |= 0x40;
    P6->OUT &= ~0x40;

    P4->SEL0 &= ~0x30;
    P4->SEL1 &= ~0x30;
    P4->DIR |= 0x30;
    P4->OUT &= ~0x30;
    //uint8_t j=0;

    while(1){

        //Read in ultrasonic sensors and filter data
        Motor_Forward(1000, 1000);
        Oleft = ultraLeft();
        Oright = ultraRight();
        Ocenter = ultraCenter();
        left = LPF_Calc(Oleft);
        right = LPF_Calc2(Oright);
        center = LPF_Calc5(Ocenter);
        UR = PWMNOMINAL;
        UL = PWMNOMINAL;
        //Error = left - right;
        Error = DESIRED-right;

        int32_t tempE= abs(Error);
        if(center < 50){//close to wall
            state = 1;
            Motor_Backward(UL, UR);
            Clock_Delay1ms(500);
            //continue;

        }


        else if(Error < 20 && Error > -20 ){
            //Centered
            state = 2;
            UL = PWMNOMINAL+Kp*tempE;
            UR = PWMNOMINAL+Kp*tempE;
        }else if(Error <= -20 ){
            //Right > DESIRED, go right
           if(right > 200 && abs(left - right) > 50){ //hard
               state = 3;
               UL = PWMNOMINAL+2*Kp*tempE + SWING;
               UR = PWMNOMINAL-2*Kp*tempE - SWING/2;
               int j;
               Motor_Forward(UL, UR);
               for(j = 0; j < 1000000; j++){
                   int m = 0;
               }
               Motor_Stop();
               Clock_Delay1ms(2000);
               Motor_Forward(PWMNOMINAL, PWMNOMINAL);
               Clock_Delay1ms(1200);
               Motor_Stop();
               Clock_Delay1ms(2000);
           }
           else{ //regular towards right
               state = 4;
            UL = PWMNOMINAL+Kp*tempE + SWING;
            UR = PWMNOMINAL-Kp*tempE - SWING;
           }

        }else if(Error >= 20 ){
            state = 5;
            //Right < DESIRED, go left
            UL = PWMNOMINAL-Kp*tempE - SWING;
            UR = PWMNOMINAL+Kp*tempE + SWING;
        }
        if(left >= 180 && center < 100){
            state = 6;
                //hard left turn
                UL = PWMNOMINAL - Kp*tempE - SWING;
                UR = PWMNOMINAL+Kp*tempE + SWING;
                //Motor_Forward(PWMNOMINAL, PWMNOMINAL);
                int i, k;
                //for(i = 0; i <5000; i++){
                //}
                Motor_Left(UL, UR);

                for(i = 0; i <1200000; i++){
                    k++;
                }
                Motor_Stop();
                Clock_Delay1ms(2000);
                Motor_Forward(PWMNOMINAL, PWMNOMINAL);
               Clock_Delay1ms(800);
               Motor_Stop();
               Clock_Delay1ms(2000);



                }
        if(center < 80 && left < 45 & right <45){
            //turn around
            UL = PWMNOMINAL+2*Kp*tempE + SWING;
           UR = PWMNOMINAL-2*Kp*tempE - SWING/2;
           int j;
           Motor_Right(UL, UR);
           for(j = 0; j < 1500000; j++){
               int m = 0;
           }
           Motor_Stop();
           Clock_Delay1ms(2000);

        }

        Motor_Forward(UL, UR);
        Clock_Delay1ms(100);

        /*
        if(center < 55){
            Motor_Backward(UL, UR);
        }
        else if(Error > -45 && Error < 45){
            //stay straight
            int32_t tempE= abs(Error);
            if(Error < 0){
                //right is bigger than left, so closer to left, shift right
                UL = PWMNOMINAL+Kp*tempE + Ki*abs(Ki_error_result);
                UR = PWMNOMINAL-Kp*tempE - Ki*abs(Ki_error_result);
                Motor_Forward(UL, UR);
            }
            else { //shift left
             UR = PWMNOMINAL + Kp*tempE + Ki*abs(Ki_error_result);
             UL = PWMNOMINAL-Kp*tempE - Ki*abs(Ki_error_result);
             Motor_Forward(UL, UR);

            }
            Clock_Delay1ms(100);
        }
        else if (Error < -45){
            //left a lot smaller than right, turn right
            UL = PWMNOMINAL+Kp*tempE + Ki*abs(Ki_error_result) + SWING;
            UR = PWMNOMINAL-Kp*tempE - Ki*abs(Ki_error_result) - SWING;
            //Motor_Forward(UL, UR);
            Motor_Right(UL, UR);
            //maybe try Motor_Right
            Clock_Delay1ms(600);

        }
        //else if(Error > 45 && center < 80){ //turn left
        else if(Error > 45 && center < 80){
         UR = PWMNOMINAL + Kp*tempE + Ki*abs(Ki_error_result) + SWING;
         UL = PWMNOMINAL-Kp*tempE - Ki*abs(Ki_error_result) - SWING;
         Motor_Left(UL, UR);
         Clock_Delay1ms(600);
        }
        //Clock_Delay1ms(600);

        //Follow Right Wall
        if(right < 60 && right >= 35){
            //Go straight
            if(left > right){
                   UR = PWMNOMINAL + Kp*
                   UL = PWMNOMINAL + Kp*
            }
            else{

            }
            Motor_Forward(UL, UR);
        }
        else if(right > 60 && right < 100){
            //Too far from wall need to shift right
            Motor_Forward(UL + 400, UR);
        }
        else if( right < 35){
            //Too close to right wall, shift left
            Motor_Forward( UL, UR + 800);
        }
        else if(right > 100){ //right turn
            Motor_Forward( UL + 800, UR - 400);
        }
        else if (left > 100 && center < 70){ //left turn
            Motor_Left(UL + 800, 0);
        }
        else if(center < 60 && right < 60 && left < 60){
            //dead end, turn 180
            Motor_Right(0, UR + 800);
        }
        else{
            Motor_Forward(0,0);
            Clock_Delay1ms(3000);
        }*/



    }
}


