#ifndef NONE

//******************************************************************************
//   MSP430G2xx3 Demo - USCI_A0, 9600 UART Echo ISR, DCO SMCLK
//
//   Description: Echo a received character, RX ISR used. Normal mode is LPM0.
//   USCI_A0 RX interrupt triggers TX Echo.
//   Baud rate divider with 1MHz = 1MHz/9600 = ~104.2
//   ACLK = n/a, MCLK = SMCLK = CALxxx_1MHZ = 1MHz
//
//                MSP430G2xx3
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |     P1.2/UCA0TXD|------------>
//            |                 | 9600 - 8N1
//            |     P1.1/UCA0RXD|<------------
//
//   D. Dang
//   Texas Instruments Inc.
//   February 2011
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************
#include  "msp430g2553.h"
#include <string.h>
#include <stdlib.h>

volatile unsigned int measure; //this is the difference in counts measured by the Ultrasonic timer

volatile unsigned int up=0; //helps the timer determine which edge

bool UltraPeriodically = false;
int  UltraTime = 10000;


/*
 * Configuration for Trigger Pins - these drive the Ultrasonic device
 */
#define UltraPortOut P2OUT
#define UltraPortDirection P2DIR
#define UltraFrontPin BIT1
#define UltraRightPin BIT1
#define UltraEcho BIT0

void _delay_cycles(const unsigned int& s) {
    volatile int r = s;
    while (--r);
}


void get_measure() {
    UltraPortOut |= UltraFrontPin;
    up = 1; //Next catch on Timer1A0 should be rising edge - helps with capture timer
    //_delay_cycles(10);
    UltraPortOut &= ~UltraFrontPin;
}

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

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
 P2SEL = UltraEcho;


  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
  DCOCTL = CALDCO_1MHZ;
  P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
  P1SEL2 = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 104;                            // 1MHz 9600
  UCA0BR1 = 0;                              // 1MHz 9600
  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt

  __bis_SR_register(/*LPM0_bits + */GIE);       // Enter LPM0, interrupts enabled


  while (true) {
      _delay_cycles(UltraTime);
      if (UltraPeriodically) {
          get_measure();
      }
  }
}


static int n = 0;

void serialPrint(const char * str) {
    static bool pp = false;
    if (pp) return;
    pp = true;
    for (int i=0; str[i] != '\0'; ++i) {
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = str[i];
    }
    pp = false;
}

void serialPrintInt(const int& i) {
    char buf[10];
    itoa(i, buf, 10);
    serialPrint(buf);
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
}

//  Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    const float delta = 0.1;
//  while (!(IFG2 & UCA0TXIFG));                // USCI_A0 TX buffer ready?
  char c = UCA0RXBUF;
  switch (c)  {
      case 'u': {
          serialPrint("Hello World!\n\n");
          serialPrintInt(++n);
          serialPrint("\n");

        } break;
  case 's': {
      UltraPeriodically = !UltraPeriodically;
    } break;
  case 't': {
      UltraTime = UltraTime*(1-delta);
    } break;
case 'T': {
    UltraTime = UltraTime*(1+delta);
  } break;
}

  char cc[2] = {c, '\0'};
  serialPrint(cc);
  get_measure();
//  UCA0TXBUF = c;                    // TX -> RXed character
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
//      serialPrint("{");
   }
   else //is this the falling edge?
   {
      measure_2=TA1CCR0; //take second time measure
      measure=(measure_2-measure_1)/58; // microseconds / 58 = centimeters
//     char cc[2] = {measure, '\0'};
//     serialPrint(cc);

     serialPrintInt(measure);
   }
   up=!up; //if this was the rising edge, the next one will be a falling edge, and vice-versa
   TA1CTL &= ~TAIFG; //clear timer A interrupt flag, so the chip knows we handled the interrupt

}


#endif
