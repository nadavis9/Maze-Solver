#include "msp.h"
#include "Clock.h"
#include "CortexM.h"
#include "SysTickInts.h"

/**
 * main.c
 */

// 38ms for no object detected

uint16_t centerFirst, centerPeriod, leftFirst, leftPeriod, rightFirst, rightPeriod;
uint16_t centerDistance, leftDistance, rightDistance;
//uint8_t i = 0;

void Timer_Init(void);
uint32_t ultraCenter(void);
uint32_t ultraLeft(void);
uint32_t ultraRight(void);
void ultrasonicSensor(void);

void ultrasonicSensor(void)
{
    Clock_Init48MHz();
    Timer_Init();

    // P6.4, P6.5, P6.6 are connected to Trig on sensor
//    P6->SEL0 &= ~0x70;                  // P6.4, P6.5, P6.6 OUTPUT
//    P6->SEL1 &= ~0x70;
//    P6->DIR |= 0x70;
//    P6->OUT &= ~0x70;

    // P6.6 (Center), P4.5 (Left), P4.4 (Right)  are connected to Trig on sensor
    P6->SEL0 &= ~0x40;
    P6->SEL1 &= ~0x40;
    P6->DIR |= 0x40;
    P6->OUT &= ~0x40;

    P4->SEL0 &= ~0x30;
    P4->SEL1 &= ~0x30;
    P4->DIR |= 0x30;
    P4->OUT &= ~0x30;


    while(1)
        {
            ultraCenter();
            Clock_Delay1ms(100);
            ultraLeft();
            Clock_Delay1ms(100);
            ultraRight();
            Clock_Delay1ms(100);
        }
}

uint32_t ultraCenter(void)
{
    P6->OUT |= 0x40;                        // P6.6 Trigger
    Clock_Delay1us(10);                     // Wait
    P6->OUT &= ~0x40;                       // Turn off signal
    Clock_Delay1ms(100);                    // Wait for ECHO to return signal
    centerDistance = centerPeriod/178;      // Calculate distance using period measured
    return (uint32_t) centerDistance;
}

uint32_t ultraLeft(void)
{
//    P6->OUT |= 0x20;                        // P6.5 Trigger
//    Clock_Delay1us(10);                     // Wait
//    P6->OUT &= ~0x20;                       // Turn off signal
    P4->OUT |= 0x20;
    Clock_Delay1us(10);
    P4->OUT &= ~0x20;
    Clock_Delay1ms(100);
    leftDistance = leftPeriod/178;      // Calculate distance using period measured
    //leftDistance = (uint32_t) leftDistance;
    //leftDistance = leftDistance >> 16;
    return (uint32_t)leftDistance;
}

uint32_t ultraRight(void)
{
//    P6->OUT |= 0x10;                        // P6.4 Trigger
//    Clock_Delay1us(10);                     // Wait
//    P6->OUT &= ~0x10;                       // Turn off signal
    P4->OUT |= 0x10;                        // P6.4 Trigger
    Clock_Delay1us(10);                     // Wait
    P4->OUT &= ~0x10;                       // Turn off signal
    Clock_Delay1ms(100);
    rightDistance = rightPeriod/178;      // Calculate distance using period measured
    //rightDistance = (uint32_t) rightDistance;
    //rightDistance = rightDistance >> 16;
    return (uint32_t) rightDistance;
}

void Timer_Init(void)
{
    /*
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
    */

    // ---------------------------------------------------------------------------
    // Right: P7.4, TA1.4
    // Center: P7.5, TA1.3
    // Left: P7.6, TA1.2

    P7->SEL0 |= 0x70;               // P7.4, 7.5, 7.6 input
    P7->SEL1 &= ~0x70;
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

}

//void TA2_N_IRQHandler(void)
//{
//    // acknowledge capture/compare interrupt 1,2,4
//    TIMER_A2->CCTL[1] &= ~0x0001;
//    TIMER_A2->CCTL[2] &= ~0x0001;
//    TIMER_A2->CCTL[4] &= ~0x0001;
//
//    centerPeriod = TIMER_A2->CCR[1]-centerFirst;
//    centerFirst = TIMER_A2->CCR[1];
//
//    leftPeriod = TIMER_A2->CCR[2]-leftFirst;
//    leftFirst = TIMER_A2->CCR[2];
//
//    rightPeriod = TIMER_A2->CCR[4]-rightFirst;
//    rightFirst = TIMER_A2->CCR[4];
//
//}

void TA1_N_IRQHandler(void)
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

}
