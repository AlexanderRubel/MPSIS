#include <msp430.h>

char compareLedTerm = 0;
volatile int i;

void led_init()
{
    P1DIR |= BIT4; //Led on PAD4
    P1OUT &= ~BIT4;
}

void button_init()
{
    P1DIR &= ~BIT4;
    P1REN |= BIT4;
    P1OUT |= BIT4;
    P1IES |= BIT4;
    P1IFG &= ~BIT4;
    P1IE |= BIT4;
}

void adc_init()
{
    //select start scanning from 0
    ADC12CTL1 &= ~(ADC12CSTARTADD0 | ADC12CSTARTADD1 | ADC12CSTARTADD2 | ADC12CSTARTADD3);
    //Source select = ADC12SC
    ADC12CTL1 &= ~(ADC12SHS0 | ADC12SHS1);
    //adc clock select (SMCLK)
    ADC12CTL1 |= ADC12SSEL0 | ADC12SSEL1;
    //Pulse mode enable
    ADC12CTL1 |= ADC12SHP;
    //repeated multichannel mode select
    ADC12CTL1 |= ADC12CONSEQ0 | ADC12CONSEQ1;
    //enable auto sampling (but need first SHI rising edge)
    ADC12CTL0 |= ADC12MSC;
    //set adc resolution for 8 bits
    ADC12CTL2 &= ~(BIT5 | BIT4);
    //set sample rate buffer to 50ksps
    ADC12CTL2 &= ~ADC12SR;
    // channel 0 config (select potentiometer as source)
    ADC12MCTL0 |= ADC12INCH_5;
    // channel 1 config (select temp as source)
    ///set channel 1 as last for adc
    ADC12MCTL1 |= ADC12EOS;
    ///select channel source
    ADC12MCTL1 |= ADC12INCH_9;
}

#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)
{
    for(i=0;i<2500;i++);

    if(!(P1IN & BIT7))
    {
        P1IFG &= ~BIT7;

        if(!compareLedTerm)
            {
                P1OUT &= ~BIT1;
                P1OUT &= ~BIT3;

                //Disable interrupt
               CBINT &= ~CBIE;
               CBINT &= ~CBIIE;

               //enable ADC
               ADC12CTL0 |= ADC12ON;
               //enable ADC convertion and start calc
               ADC12CTL0 |= ADC12ENC | ADC12SC;
               ///enable channel 1 interrupts
               ADC12IE |= ADC12IE1;
               //clear
               ADC12IFG &= ~ADC12IFG1;
            }
        else
        {
            P1OUT &= ~BIT0;
            P8OUT &= ~BIT2;

            //disable interrupts
            ADC12IE &= ~ADC12IE1;
            //disable ADC
            ADC12CTL0 &= ~ADC12ON;
            //disable ADC convertion and reset start calc bit
            ADC12CTL0 &= ~(ADC12ENC | ADC12SC);


            //Enable interrupt (main & inverted)
            CBINT |= CBIE;
            CBINT |= CBIIE;
            //clear
            CBINT &= ~CBIFG;
            CBINT &= ~CBIIFG;
        }

        compareLedTerm ^= BIT0;
    }
}

/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    led_init();
    button_init();

    adc_init();
    comparator_init();

    __bis_SR_register(GIE + LPM0_bits);
    while(1);

    return;
}

#pragma vector=ADC12_VECTOR
__interrupt void adc_interrupt()
{
    short val_U = ADC12MEM1;
    short val_term = ADC12MEM0;

    if(val_U > val_term){
        P1OUT |= BIT0;
        P8OUT &= ~BIT2;
    }
    else{
        P1OUT &= ~BIT0;
        P8OUT |= BIT2;
    }

    ADC12IFG &= ~ADC12IFG1;
}
