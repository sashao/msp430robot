#ifdef NONE

/*
 * Chris Berg
 * Started - October 2011
 * Version 1 - 11/12/2011
 * Project: Bluetooth controlled RC Car
 * Ultrasonic sensors used for Autonomous (wall following) mode
 * Combined project using Bluetooth and Ultrasonic sensors
 */


//
//
//
//                              \\\
//                              ---
//                               |
//                               |
//                              .-.
//                              | |R2:20K
//                              |_|
//     MSP430G2553               |               HC-SR04 (Front)
//     ------------              |         \\\  -------
//    |         XIN|-            |         --- |       |---
//    |        XOUT|-    R1:10k  |          |  |       |   |
//    |            |        ___  |  1N5818  ---|GND    |   |
//    |        P2.0|-------[___]-o-----|<------|ECHO   |---
//    |        P2.1|-------------|-------------|TRIG   |
//    |        P2.2|-----------| |          ---|VCC(5V)|---
//    |            |           | |          |  |       |   |
//    |            |           | |         \|/ |       |   |
//    |            |           | |             |       |---
//    |            |           | |              -------
//    |            |           | |
//    |            |           | |              HC-SR04 (Right)
//    |    RXD P1.1|----       | |         \\\  -------
//    |    TXD P1.2|-- |       | |         --- |       |---
//    |            | | |       | |          |  |       |   |
//    |            | | |       | |  1N5818  ---|GND    |   |
//    |            | | |       | -----|<-------|ECHO   |---
//    |            | | |       ----------------|TRIG   |
//                 | | |                    ---|VCC(5V)|---
//                 | | |                    |  |       |   |
//                 | | |                   \|/ |       |   |
//                 | | |              \\\      |       |---
//                 | | |              ---       -------
//                 | | |               |   ----------------
//                 | | |               ---|GND            [
//                 | | |           <------|3.3             ]
//                 | | |                  |5.0   HC-06    [
//                 | | -------------------|TXD             ]
//                 | ---------------------|RXD            [
//                 |                      |KEY             ]
//                 |                       ----------------
//                 |
//                 |
//                 |       RX2
//                 |     -------------
//                 |    |             |
//             P1.3|----|6  right     |
//             P1.0|----|7  left      |
//             P1.4|----|11 forward   |
//             P1.5|----|10  backward |
//                 |    |             |
//                       -------------
//
//
//
//


#include <msp430g2553.h>
volatile unsigned int measure; //this is the difference in counts measured by the Ultrasonic timer

volatile unsigned int up=0; //helps the timer determine which edge
//determines what mode to run the car in - this will start as Manual later on
volatile char runMode = 'M'; //W = Wall, M = manual - setting Manual as default

/*
 * These are our constants that drive wall-following
 * At or Below RightLow = too close to wall
 * At or Above RightHigh = too far from wall
 * Will need to fiddle with these values to find something that works well with your car
*/
const unsigned int FrontHighThreshold = 25; //16 initially
const unsigned int FrontLowThreshold = 8;
const unsigned int RightLowThreshold = 10;//6 & 8 didn't work
const unsigned int RightHighThreshold = 20;

/*
 * Configuration for Trigger Pins - these drive the Ultrasonic device
 */
#define UltraPortOut P2OUT
#define UltraPortDirection P2DIR
#define UltraFrontPin BIT1
#define UltraRightPin BIT2

/*
 * Configuration for drive pins - these can be changed to move around pins that drive H Bridge
 * Will need to recode if you need to split these pins across ports
*/
#define DrivePortOut P1OUT
#define DrivePortDirection P1DIR
#define LeftPin BIT0
#define RightPin BIT3
#define ForwardPin BIT4
#define BackPin BIT5



void Left();
void Right();
void Straight();
void Forward();
void Backward();
void Stop();

main()
{
   WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
   //***Configure clock
   // we need DCO@1MHZ -> SMCLCK -> TA
   BCSCTL1 |= CALBC1_1MHZ;         //
   DCOCTL |= CALDCO_1MHZ;          // dco at 1mhz
   BCSCTL2 &= ~SELS;               // select dco for smclck source
   //BCSCTL2 |= DIVS0;               // select no division
   BCSCTL2 &= ~(DIVS1|DIVS0);

   DrivePortDirection |= LeftPin|RightPin|ForwardPin|BackPin;                            // Set P1.0 to output direction
   DrivePortOut &= ~(LeftPin|RightPin|ForwardPin|BackPin);//turn all drive ports off - avoid any issues with outputs
   P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
   P1SEL2 = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
   UCA0CTL1 |= UCSSEL_2;                     // SMCLK
   UCA0BR0 = 104;                            // 1MHz 9600
   UCA0BR1 = 0;                              // 1MHz 9600
   UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
   UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
   IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt

   //***Timer1A? capture configuration
    //rising & falling edge + synchronous + P2.0 (CCI1A) + capture + capture/compare interrupt enable
    TA1CCTL0 |= CM_3 + CCIS_0 + CAP + CCIE;
    //select smclock for timer a source + make ta1ccr0 count continously up + no division
   TA1CTL |= TASSEL_2 + MC_2 + ID_0;


    //***Set up pins for Ultrasonic sensing
   UltraPortDirection = UltraFrontPin|UltraRightPin;
   UltraPortOut &= ~(UltraFrontPin|UltraRightPin);//turn off trigger pins to make sure they're in the correct state
   //Set P2.0 to pick up echo from the HC-SR04
   //Not using a #define element for this - it's tied to the timer
   P2SEL = BIT0;
   P1OUT=0;        // turn led and outputs off

   //set up pins for driving
   DrivePortDirection = LeftPin|RightPin|ForwardPin|BackPin;
   DrivePortOut &= ~(LeftPin|RightPin|ForwardPin|BackPin); //make sure all drive pins are off

   //***Enable interrupts
     _BIS_SR(GIE);   // general interrupt enable

   unsigned int trigPin = UltraRightPin;
   unsigned int device = 0;//0 = Right, 1 = Front
   unsigned int RightMeasure = 0;
   unsigned int FrontMeasure = 0;
   //***Loop
   while (1)
   {
      if (runMode == 'W')//wall following
      {
         //store values from sensor
         if (device == 0) //Right
         {
            if(measure != 0)
               RightMeasure = measure;
            trigPin = UltraFrontPin;
            device = 1;
         }
         else //Front
         {
            if(measure != 0)
               FrontMeasure = measure;
            trigPin = UltraRightPin;
            device = 0;
         }
         //review values from sensor - evaluate if both values are availaable
         if(RightMeasure > 0 && FrontMeasure > 0)
         {
            if (FrontMeasure <= FrontLowThreshold)
            {
               Right();
               Backward();
            }
            else if (FrontMeasure <= FrontHighThreshold)
            {
               Left();
               Forward();
            }
            else if (RightMeasure <= RightLowThreshold)
            {
               Left();
               Forward();
            }
            else if (RightMeasure >= RightHighThreshold)
            {
               Right();
               Forward();
            }
            else
            {
               Straight();
               Forward();
            }
         }
         UltraPortOut |= trigPin;
         up = 1; //Next catch on Timer1A0 should be rising edge - helps with capture timer
         _delay_cycles(10);
         UltraPortOut &= ~trigPin;
         _delay_cycles(50);
         //TODO: Figure out some way to use a timer for the delay instead - use low power mode if possible
         //delay is approximate - must be at least 50ms per device
         _delay_cycles(55000);
      }
      else
      {
         //is there anything special to do in manual mode?
      }
   }
}

void Left()
{
   DrivePortOut &= ~(RightPin);//turn right off if enabled
   DrivePortOut |= LeftPin;
}
void Right()
{
   DrivePortOut &= ~(LeftPin);//turn left off if enabled
   DrivePortOut |= RightPin;
}
void Straight()
{
   DrivePortOut &= ~(LeftPin|RightPin);
}
void Forward()
{
   DrivePortOut &= ~(BackPin);
   DrivePortOut |= ForwardPin;
}
void Backward()
{
   DrivePortOut &= ~(ForwardPin);
   DrivePortOut |= BackPin;
}
void Stop()
{
   DrivePortOut &= ~(ForwardPin|BackPin);
}

//can these be switched to static within the timer?  they don't need to be available outside of the timer
unsigned int measure_1 = 0;
unsigned int measure_2 = 0;

//Timer1_A Capture
//P2.0 ends up triggering this timer
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1A0(void)
{
   if(up) //is this the rising edge?
   {
      measure_1=TA1CCR0;  // take first time measure
   }
   else //is this the falling edge?
   {
      measure_2=TA1CCR0; //take second time measure
      measure=(measure_2-measure_1)/58; // microseconds / 58 = centimeters
   }
   up=!up; //if this was the rising edge, the next one will be a falling edge, and vice-versa
   TA1CTL &= ~TAIFG; //clear timer A interrupt flag, so the chip knows we handled the interrupt
}

/*
 * Incoming commands from Serial device
 * Currently only set to handle single character commands
*/
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
   char buffer = UCA0RXBUF;
   if (runMode == 'M')
   {
     if (UCA0RXBUF == 'l')
        Left();
     else if (UCA0RXBUF == 'r')
        Right();
     else if (UCA0RXBUF == 'm')
        Straight();
     else if (UCA0RXBUF == 'f')
        Forward();
     else if (UCA0RXBUF == 'b')
        Backward();
     else if (UCA0RXBUF == 's')
        Stop();
     else if (UCA0RXBUF == 'W')
     {
        Stop();
      Straight();
      runMode = 'W';
     }
   }
   else
   {
      if (UCA0RXBUF == 'M')
      {
         Stop();
         Straight();
         runMode = 'M';
      }
   }
}

#endif
