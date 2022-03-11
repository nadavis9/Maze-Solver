// ---------------
//good
#include <stdint.h>
#include "msp.h"
#include "CortexM.h"
#include "Clock.h"
#include "SysTickInts.h"
#include "Bump.h"
//#include "Motor.h"
#include "../inc/Tachometer.h"
#include "Ultrasonic.h"
#include "LPF.h"

// ---------------
//Ultrasonic Variables Globals
uint16_t center, left, right;


//-----------------------------------------------------------------------------------


void Timer_Init(void);
//uint32_t ultraCenter(void);
//uint32_t ultraLeft(void);
//uint32_t ultraRight(void);

/*void main(void)
{
    Clock_Init48MHz();
    Timer_Init();

    // P6.4, P6.5, P6.6 are connected to Trig on sensor
    P6->SEL0 &= ~0x70;                  // P6.4, P6.5, P6.6 OUTPUT
    P6->SEL1 &= ~0x70;
    P6->DIR |= 0x70;
    P6->OUT &= ~0x70;


    while(1)
        {
            ultraCenter();
            Clock_Delay1ms(100);
            ultraLeft();
            Clock_Delay1ms(100);
            ultraRight();
            Clock_Delay1ms(100);
        }

}*/

//---------------------------------------------------------------------------------------------
//Bluetooth --check these for P6
/*uint16_t ultraCenter(void)
{
    P6->OUT |= 0x40;                        // P6.6 Trigger
    Clock_Delay1us(10);                     // Wait
    P6->OUT &= ~0x40;                       // Turn off signal
    Clock_Delay1ms(100);                    // Wait for ECHO to return signal
    centerDistance = centerPeriod/178;      // Calculate distance using period measured
    return centerDistance;
}

uint16_t ultraLeft(void)
{
//    P6->OUT |= 0x20;                        // P4.5 Trigger
//    Clock_Delay1us(10);                     // Wait
//    P6->OUT &= ~0x20;                       // Turn off signal
    P4->OUT |= 0x20;
    Clock_Delay1us(10);
    P4->OUT &= ~0x20;
    Clock_Delay1ms(100);
    leftDistance = leftPeriod/178;      // Calculate distance using period measured
    return leftDistance;
}


uint16_t ultraRight(void)
{
//    P6->OUT |= 0x10;                        // P6.4 Trigger
//    Clock_Delay1us(10);                     // Wait
//    P6->OUT &= ~0x10;                       // Turn off signal
    P4->OUT |= 0x10;                        // P4.4 Trigger
    Clock_Delay1us(10);                     // Wait
    P4->OUT &= ~0x10;                       // Turn off signal
    Clock_Delay1ms(100);
    rightDistance = rightPeriod/178;      // Calculate distance using period measured
    return rightDistance;
}
*/
//---------------------------------------------------------------------------------------------
//Non Bluetooth
/*uint16_t ultraCenter(void)//
{
    P6->OUT |= 0x40;                        // P6.6 Trigger
    Clock_Delay1us(10);                     // Wait
    P6->OUT &= ~0x40;                       // Turn off signal
    Clock_Delay1ms(100);                    // Wait for ECHO to return signal
    centerDistance = centerPeriod/178;      // Calculate distance using period measured
    return centerDistance;
}

uint16_t ultraLeft(void)
{
    P6->OUT |= 0x20;                        // P6.5 Trigger
    Clock_Delay1us(10);                     // Wait
    P6->OUT &= ~0x20;                       // Turn off signal
    Clock_Delay1ms(100);
    leftDistance = leftPeriod/178;      // Calculate distance using period measured
    return leftDistance;
}

uint16_t ultraRight(void)
{
    P6->OUT |= 0x10;                        // P6.4 Trigger
    Clock_Delay1us(10);                     // Wait
    P6->OUT &= ~0x10;                       // Turn off signal
    Clock_Delay1ms(100);
    rightDistance = rightPeriod/178;      // Calculate distance using period measured
    return rightDistance;
}*/


/*void Timer_Init(void)
{
    //-------------------------------------------------------------------------
    //Bluetooth Timer
   P7->SEL0 |= 0x70;               // P7.4, 7.5, 7.6 input
   P7->SEL1 &= ~0x70; //this was P7->SEL0 |= ~0x70
   P7->DIR &= ~0x70;

   TIMER_A1->CTL &= ~0x0030;       // Halt Timer_A1
   TIMER_A1->CTL = 0x0200;

   // Trigger on both rising and falling edges
   TIMER_A1->CCTL[4] = 0xC910;     // TA1.4
   TIMER_A1->CCTL[3] = 0xC910;     // TA1.3
   TIMER_A1->CCTL[2] = 0xC910;     // TA1.2

   TIMER_A1->EX0 &= ~0x0007;       // Configure for input clock divider /1                   Why is there no EXO for TIMER_A1?

   // Enabling interrupts
   NVIC->IP[2] = (NVIC->IP[2]&0x00FFFFFF|0x40000000);  // priority 2
   //NVIC->IP[2] = (NVIC->IP[2]&0xFFFF00FF|0x00004000);  // priority 2
   //NVIC->IP[2] = (NVIC->IP[2]&0x00FFFFFF|0x40000000);  // priority 2
   NVIC->ISER[0] = 0x00000800; // enable interrupt 11 in NVIC

   // Continuous counting
   TIMER_A1->CTL |= 0x0024;         // reset and start Timer A1 in continuous up mode
    ----------------------------------------------------------------------
     Non Bluetooth Timer
    // TA2.1 Center

    // TA2.2 Left
    // TA2.4 Right

    P5->SEL0 |= 0xC0;               // P5.6 and 5.7 as input
    P5->SEL1 &= ~0xC0;
    P5->DIR &= ~0xC0;               // make P5.6/P5.7 in

    P6->SEL0 |= 0x80;               // P6.7 input
    P6->SEL1 &= ~0x80;
    P6->DIR &= ~0xC0;               // make P6.7 in

    TIMER_A2->CTL &= ~0x0030;       // Halt Timer_A2
    TIMER_A2->CTL = 0x0200;

    // Trigger on both rising and falling edges
    TIMER_A2->CCTL[1] = 0xC910;     // TA2.1
    TIMER_A2->CCTL[2] = 0xC910;     // TA2.2
    TIMER_A2->CCTL[4] = 0xC910;     // TA2.4

    TIMER_A2->EX0 &= ~0x0007;        // configure for input clock divider /1

    // Enabling interrupts
    NVIC->IP[3] = (NVIC->IP[3]&0xFFFF00FF)|0x00004000; // priority 2
    NVIC->ISER[0] = 0x00002000;      // enable interrupt 13 in NVIC

    // Continuous counting
    TIMER_A2->CTL |= 0x0024;         // reset and start Timer A2 in continuous up mode


}*/

//--------------------------------------------------------------------------------------
//Bluetooth
/*void TA1_N_IRQHandler(void)
{
    // acknowledge capture/compare interrupt 1,2,4
    TIMER_A1->CCTL[4] &= ~0x0001;
    TIMER_A1->CCTL[3] &= ~0x0001;
    TIMER_A1->CCTL[2] &= ~0x0001;

    centerPeriod = TIMER_A1->CCR[3]-centerFirst;
    centerFirst = TIMER_A1->CCR[3];

    leftPeriod = TIMER_A1->CCR[2]-leftFirst;
    leftFirst = TIMER_A1->CCR[2];

    rightPeriod = TIMER_A1->CCR[4]-rightFirst;
    rightFirst = TIMER_A1->CCR[4];

}*/

//---------------------------------------------------------------------------------------
// Non Bluetooth
/*void TA2_N_IRQHandler(void)
{
    // acknowledge capture/compare interrupt 1,2,4
    TIMER_A2->CCTL[1] &= ~0x0001;
    TIMER_A2->CCTL[2] &= ~0x0001;
    TIMER_A2->CCTL[4] &= ~0x0001;

    centerPeriod = TIMER_A2->CCR[1]-centerFirst;
    centerFirst = TIMER_A2->CCR[1];

    leftPeriod = TIMER_A2->CCR[2]-leftFirst;
    leftFirst = TIMER_A2->CCR[2];

    rightPeriod = TIMER_A2->CCR[4]-rightFirst;
    rightFirst = TIMER_A2->CCR[4];

}*/


//-----------------------------------------------------------------------------------


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

uint16_t tach_error;
uint32_t leftS, rightS;

//Proportional Control Global
#define PWMNOMINAL 2500
#define SWING 1000
int32_t UR=2500, UL=2500, mode=0, Error, Kp=10, SetPoint=250, Ki = 15, Ki_error_sum = 0, Ki_count = 0, Ki_error_result = 0;
int32_t Kpt = 5, Kit = 15, Kit_error_sum = 0, Kit_count = 0, Kit_error_result = 0;


uint16_t centerFirst, centerPeriod, leftFirst, leftPeriod, rightFirst, rightPeriod, centerDistance, leftDistance, rightDistance;
uint8_t i = 0;
enum TachDirection leftDir;
enum TachDirection rightDir;
int32_t leftSteps, rightSteps, leftDiff, rightDiff;
uint16_t leftTach, rightTach;



void main(void){
    DisableInterrupts();    //This is here because the Lab17 solutions had it
    //EdgeTrigger_Init();     //Initialize the Bump Sensors
    Clock_Init48MHz();      //Initialize Clock
    Timer_Init();
    //PWM_Init34(15000, 4000, 4000);  //Initialize the motors
    Tachometer_Init();
    Motor_Init();
    TimerA3Capture_Init01(&PeriodMeasure0, &PeriodMeasure1);

     //100Hz
    //UR = UL = PWMNOMINAL;    // reset parameters
    EnableInterrupts();
    SysTick_Init(480000,3);
               //Initialize TimerA2 for ultrasonic Sensors
    LPF_Init(45, 10);
    LPF_Init2(45, 10);
    LPF_Init3(2500, 10);
    LPF_Init4(2500, 10);



    //----------------------------------------------------
    //No Bluetooth
    // P6.4, P6.5, P6.6 are connected to Trig on sensor
    //P6->SEL0 &= ~0x70;
    //P6->SEL1 &= ~0x70;
    //P6->DIR |= 0x70;
    //P6->OUT &= ~0x70;
    //--------------------------------------------------------

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
        //tach_error = Error_Calc();
        //Proportional Control Calculation
        //Motor_Forward(2500, 2500);



    }
        //tach_error = Error_Calc();
        /*if(Center < 30){
            //stop turn right
        }
        if(Error > 165){
            //Turn Left
        }
        else if(Error < -165){
            //hard right
        }*/


        /*centerDistance = ultraCenter();
        Clock_Delay1ms(100);
        left = ultraLeft();
        Clock_Delay1ms(100);
        right = ultraRight();
        Clock_Delay1ms(100);*/

        /*
        //Calculate the distance using the SysTick handler
            //hold the prev value to elimate '0' readings
            uint16_t holdL = left;
            //left = ultraLeft();
            if(left == 0){
                left = holdL;
            }

            uint16_t holdR = right;
            //right = ultraRight();
            if(right == 0){
                right = holdR;
            }

            //Proportional Control Calculation
            Error = left - right; //difference between ultrasonic distance sensors
            if (Error > 40){
              //Closer to right wall, turn left
              //Set the right wheel to a faster speed
              int32_t tempE= abs(Error);
              UR = PWMNOMINAL + Kp*tempE;
              UL = PWMNOMINAL-Kp*tempE;

             }

            else if (Error < -40){
              //Closer to left wall, turn right
              //Set the left wheel to a faster speed
              int32_t tempE= abs(Error);
              UL = PWMNOMINAL+Kp*tempE;
              UR = PWMNOMINAL-Kp*tempE;

            }
            else {
                //Centered
                //Tach Control - balance power on two wheels to go straight
                if(tach_error > 0) {
                    int32_t tempE= abs(tach_error);
                    UR = PWMNOMINAL + Kp*tempE;
                    UL = PWMNOMINAL-Kp*tempE;
                }
                else{
                    int32_t tempE= abs(tach_error);
                    UR = PWMNOMINAL - Kp*tempE;
                    UL = PWMNOMINAL+Kp*tempE;
                }

            }
            //Drive motors forward
            Motor_Forward(UR, UL);
            ClockDelay1ms(100);
            */
      //do nothing in this loop because the
      //distance and motor speed will be calculated through SysTick
    }
//}


void SysTick_Handler(void){
    //Error = left - right; //difference between ultrasonic distance sensors
            //Error = right - left;


    uint32_t left_old, right_old;

    //leftDiff = leftS - UL;
    //rightDiff = rightS - UR;
    leftDiff = 0;
    rightDiff = 0;


            uint16_t holdL = left;
                left_old = ultraLeft();
                left = LPF_Calc(left_old);
                //Clock_Delay1ms(100);
                if(left == 0){
                    left = holdL;
                }

                uint16_t holdR = right;
                right_old = ultraRight();
                right = LPF_Calc2(right_old);
                //Clock_Delay1ms(100);
                if(right == 0){
                    right = holdR;
                }

           /* if(centerDistance < 50){
                     Motor_Forward(0,0);
                     return;
                  }*/
                Error = left - right;
                Ki_count++;
                Ki_error_sum += Error;
                Ki_error_result = Ki_error_sum / Ki_count;

            if (Error > 0){
              //if(centerDistance < 50){
              //   Motor_Forward(0,0);
              //}
              //Closer to right wall, turn left
              //Set the right wheel to a faster speed
              if(Error > 0 && Error < 80){
                  int32_t tempE= abs(Error);
                  UR = PWMNOMINAL + Kp*tempE + Ki*abs(Ki_error_result) + rightDiff;
                  UL = PWMNOMINAL-Kp*tempE - Ki*abs(Ki_error_result) - leftDiff;
                  Motor_Forward(UL, UR);
                  //Clock_Delay1ms(10);
              }
              else{ //hard left turn
                  int32_t tempE= abs(Error);
                  uint32_t Kp_new = 2 * Kp;
                  //UR = PWMNOMINAL + Kp*tempE + 800 + Ki*Ki_error_result + rightDiff;
                  //UL = PWMNOMINAL-Kp*tempE - 800 - Ki*Ki_error_result + leftDiff;
                  //UR = PWMNOMINAL + Kp*tempE + 400 + Ki*abs(Ki_error_result);
                  //UL = PWMNOMINAL - Kp*tempE - 400 - Ki*abs(Ki_error_result);
                  Motor_Forward(4000, 4000);
                  Clock_Delay1ms(200);
                  Motor_Left(0, 4000 + Kp*Error);
                  Clock_Delay1ms(1000);
                  //Ki_error_sum = 0;
                  //Ki_count = 0;
                  Motor_Forward(0,0);
                  Clock_Delay1ms(600);
                  Motor_Forward(4000, 4000);
                  Clock_Delay1ms(1000);
                  LPF_Init(45, 10);


             }
            }
            else if (Error < 0){
              //Closer to left wall, turn right
              //Set the left wheel to a faster speed
               // if(centerDistance < 50){
                //         Motor_Forward(0,0);
                //}
                if(Error < 0 && Error > -80 ){
                  int32_t tempE= abs(Error);
                  UL = PWMNOMINAL+Kp*tempE + Ki*abs(Ki_error_result) + leftDiff;
                  UR = PWMNOMINAL-Kp*tempE - Ki*abs(Ki_error_result) - rightDiff;
                  Motor_Forward(UL, UR);
                  //Clock_Delay1ms(10);
              }
              else { //hard right turn
                  int32_t tempE= abs(Error);
                  //change controller output
                //UL = PWMNOMINAL+Kp*tempE + 800 + Ki*Ki_error_result;
                //UR = PWMNOMINAL-Kp*tempE -800 - Ki*Ki_error_result;
                uint32_t Kp_new = 2 * Kp;
                //UL = PWMNOMINAL+Kp*tempE + 400 + Ki*abs(Ki_error_result);
                //UR = PWMNOMINAL- Kp*tempE - 400 - Ki*abs(Ki_error_result);
                Motor_Forward(4000, 4000);
                Clock_Delay1ms(200);
                Motor_Right(4000 + Kp*Error, 0);
                Clock_Delay1ms(1000);
                //Ki_error_sum = 0;
                //Ki_count = 0;
                Motor_Forward(0,0);
                Clock_Delay1ms(600);
                Motor_Forward(4000, 4000);
                Clock_Delay1ms(1000);
                LPF_Init2(45, 10);
              }

            }
           Tachometer_Get(&leftTach, &leftDir, &leftSteps, &rightTach, &rightDir, &rightSteps);
            leftS = LPF_Calc3(leftTach);
                //Clock_Delay1ms(10);
            rightS = LPF_Calc4(rightTach);


    //Drive motors forward
    //Motor_Forward(UR, UL);
    //Clock_Delay1ms(10);

}
