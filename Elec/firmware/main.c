#include <msp430.h> 

/**
 * main.c
 */

#define TRUE 1
#define FALSE 0

unsigned int startStepping = FALSE;
unsigned int steps = 0;
unsigned int dir = FALSE;

unsigned int plus1loop8(state)
{
    if (state == 7)
    {
        return 0;
    }
    else
    {
        return state + 1;
    }
}

unsigned int minus1loop8(state)
{
    if (state == 0)
    {
        return 7;
    }
    else
    {
        return state - 1;
    }
}

void stepStepper(int p3, int p4, int p5, int p6)
{
    if (p3 == TRUE)
    {
        P1OUT |= BIT3;
    }
    else
    {
        P1OUT &= ~BIT3;
    }
    if (p4 == TRUE)
    {
        P1OUT |= BIT4;
    }
    else
    {
        P1OUT &= ~BIT4;
    }
    if (p5 == TRUE)
    {
        P1OUT |= BIT5;
    }
    else
    {
        P1OUT &= ~BIT5;
    }
    if (p6 == TRUE)
    {
        P1OUT |= BIT6;
    }
    else
    {
        P1OUT &= ~BIT6;
    }
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    // Configure clocks
    CSCTL0 = 0xA500;   // Write password to modify CS registers
    CSCTL1 = DCOFSEL0 + DCOFSEL1;   // DCO = 8 MHz
    CSCTL2 = SELM0 + SELM1 + SELA0 + SELA1 + SELS0 + SELS1; // MCLK = DCO, ACLK = DCO, SMCLK = DCO

    // Configure ports for UCA0
    P2SEL0 &= ~(BIT0 + BIT1);
    P2SEL1 |= BIT0 + BIT1;

    // Configure UCA0
    UCA0CTLW0 = UCSSEL0;
    UCA0BRW = 52;
    UCA0MCTLW = 0x4900 + UCOS16 + UCBRF0;
    UCA0IE |= UCRXIE;

    // global interrupt enable
    _EINT();

    // Stepper set up
    P1DIR |= BIT3 + BIT4 + BIT5 + BIT6;
    unsigned int state = 0;
    // ccw sequence
    int p3[8] = { FALSE, FALSE, FALSE, FALSE, FALSE, TRUE, TRUE, TRUE };
    int p4[8] = { FALSE, FALSE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE };
    int p5[8] = { FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE };
    int p6[8] = { TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE, TRUE };
    stepStepper(p3[state], p4[state], p5[state], p6[state]);

    while (1)
    {
        if (startStepping == TRUE)
        {
            int i = 0;
            while (i < steps)
            {
                if (dir == TRUE) // true = cw
                {
                    state = plus1loop8(state);
                }
                else
                {
                    state = minus1loop8(state);
                }
                stepStepper(p3[state], p4[state], p5[state], p6[state]);
                __delay_cycles(1500);
                i = i + 1;
            }
            startStepping = FALSE;
        }
    }
    return 0;
}

#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    unsigned int RxByte;
    RxByte = UCA0RXBUF;
    if (RxByte > 127)
    {
        steps = (RxByte - 127) * 10;
        dir = TRUE; // true == cw
        startStepping = TRUE;
    }
    else
    {
        steps = (127 - RxByte) * 10;
        dir = FALSE;
        startStepping = TRUE;
    }

}
