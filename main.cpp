#ifdef NONE

//#include <iostream>
#include <msp430g2353.h>

//using namespace std;


void delay(unsigned msec) {
    const volatile unsigned long tested_iterations = 25000;
    const volatile unsigned long tested_blinks = 23;
    const volatile unsigned long testing_time_ms = 30000;
    const unsigned long ticks = (long(tested_iterations)*tested_blinks)/testing_time_ms;
    volatile unsigned long i = ticks * msec;
    while(i != 0){
        i--;
    }
}

const int GREEN = BIT6;
const int RED = BIT0;
const int RIGHT = BIT5;
const int LEFT = BIT0;
const int FW = BIT1;
const int BW = BIT2;

const int ONES = RED | GREEN | RIGHT;
const int TWOS = LEFT | FW | BW ;

/*
void forward(int second)
{
do
    {
    P2OUT = FW;
    second--;
    delay(1);
    }
while (second>=0)
P2OUT ^= FW;
}

void full_stop(int iterations)
{
    for (iterations; iterations>=0; iterations-- )
    {
    P2OUT = FW;
    P2OUT ^= FW;
    P2OUT = BW;
    P2OUT ^= FW;
    }
}

void stop()
{
 P2OUT = 0x00
}

void go_forward()
{
 P2OUT = FW;
}

void go_back()
{
 P2OUT = FW;
}

void turn_left()
{

}

*/



int main(void) {
    WDTCTL = WDTPW + WDTHOLD;		// Stop watchdog timer
    P1DIR = ONES;// | BIT5 | BITA | BITB | BITC;					// Set P1.0 to output direction
    P2DIR = TWOS;

    const unsigned move = 3000;
    const unsigned pause = 2000*0;

    delay(move);

//    P2OUT = FW;
//    P1OUT = 0x00;
//    P1OUT ^= GREEN; delay(move);
//    P2OUT = 0x00;
//    P1OUT = RED;   delay(pause);

//    P2OUT = BW;
//    P1OUT = 0x00;
//    P1OUT ^= GREEN; delay(move);
//    P2OUT = 0x00;
//    P1OUT = RED;   delay(pause);

//    P2OUT = LEFT;
//    P1OUT = 0x00;
//    P1OUT ^= GREEN; delay(move);
//    P2OUT = 0x00;
//    P1OUT = RED;   delay(pause);

//    P1OUT = RIGHT;
//    P2OUT = 0x00;
//    P1OUT ^= GREEN; delay(move);
//    P1OUT = 0x00;
//    P1OUT = RED;   delay(pause);

    const unsigned move2 = 1000;
    const unsigned pause2 = 1000;
    const unsigned pause_turn = 0;
    while (true)

    {
        P2OUT = FW;
        P2OUT |= LEFT;
        delay(move);
        P2OUT ^= FW;
        delay(move/10);
        P2OUT = FW;
        P1OUT = RIGHT;
        delay(move);
        P2OUT ^= FW;
        delay(move/10);
    }

    P2OUT = 0x00;
    P1OUT = RED | GREEN;
    while(true) {
        P1OUT ^= RED | GREEN;
        delay(500);
    }
//    while(true) {
//        P1OUT ^= BIT0 | BIT6 | BIT7;				// Toggle P1.0 using exclusive-OR
//        delay(1000);
//    }



    P1OUT = 0x00;
    return 0;
}


#endif
