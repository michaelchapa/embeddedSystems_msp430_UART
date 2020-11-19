#include <msp430.h>
#include <string.h>

#define ACLK 0x0100         // Timer_A ACLK source
#define UP 0x0010           // Timer_A UP mode
#define UART_CLK_SEL 0x0080 // Specifies accurate SMCLK clock for UART
#define BR0_FOR_9600 0x34   // Value required to use 9600 baud
#define BR1_FOR_9600 0x00   // Value required to use 9600 baud
#define CLK_MOD 0x4911      // Micro-controller will "clean-up" clock signal

// Tells the micro-controller which of the different frequency clock signals it should
// use to coordinate all the different parts of the micro-controller.
// (CSCTL0) Clock System ConTroL 0 register
void select_clock_signals(void) {
    CSCTL0 = 0xA500;    // "Password" to access clock calibration registers (CSCTL[1,2,3])
    CSCTL1 = 0x0046;    // Specifies frequency of main clock
    CSCTL2 = 0x0133;    // Assigns additional clock signals (the peripherals have access to)
    CSCTL3 = 0x0000;    // Use clocks at intended frequency, do not slow them down
}

// Used to Give UART Control of Appropriate Pins
void assign_pins_to_uart(void) {
    P3SEL1 = 0x00;          // 0000 0000
    P3SEL0 = BIT4 | BIT5;   // 0001 1000
    //assigns P3.4 to UART Transmit (TXD) and assigns P3.5 to UART Receive (RXD)
}

//Specify UART Baud Rate (9600 bits/ sec)
void use_9600_baud(void) {
    UCA1CTLW0 = UCSWRST;                    // Put UART into SoftWare ReSeT1 
    					    // (Put into 'Software Hold' while we change baud rate)
    UCA1CTLW0 = UCA1CTLW0 | UART_CLK_SEL;   // Maps clock source for UART to peripheral
    UCA1BR0 = BR0_FOR_9600;                 // Specifies bit rate (baud) of 9600
    UCA1BR1 = BR1_FOR_9600;                 // Specifies bit rate (baud) of 9600
    UCA1MCTLW = CLK_MOD;                    // Tell peripheral to "Clean" the clock signal
    UCA1CTLW0 = UCA1CTLW0 & (~UCSWRST);     // Takes UART out of SoftWare ReSeT
}

int iNum = 9;

void main() {
    WDTCTL = WDTPW | WDTHOLD;               // Stop the watchdog timer
    PM5CTL0 &= ~LOCKLPM5;                   // Enable inputs and outputs

    select_clock_signals();                 // Assigns microcontroller clock signals
    assign_pins_to_uart();                  // P4.2 is for TXD, P4.3 is for RXD
    use_9600_baud();                        // UART operates at 9600 bits/second

    // Setup clock 1
    TA0CCR0 = 4681;                         // Count to 3 seconds
    TA0CTL = ACLK | MC__UP | ID__8;         // Use AuxClock, upMode, decrease
    TA0CCTL0 = CCIE;                        // Enable interrupt for Timer_0

    P1DIR |= BIT0;                          // set red LED as output
    P1OUT &= ~BIT0;                         // red LED Off

    P9DIR |= BIT7;                          // Direct BIT1 pin as output
    P9OUT &= ~BIT7;                         // Green LED = ON;

    // Activate enabled interrupts and wait for interrupt
    UCA1IE = UCRXIE;
    _BIS_SR(GIE);
    while(1) {
        if(UCA1IFG & UCRXIFG) { // Receive any new message?
            UCA1TXBUF = UCA1RXBUF; // New message is buffered in register UCA0RXBUF. 
	    			   // Copy it to Transmission Buffer.
            __delay_cycles(20000);
        }
        UCA0IFG = UCA0IFG & (~UCRXIFG); // Reset the UART receive flag
    }
}

#pragma vector=TIMER0_A0_VECTOR // ISR put in controller program memory
__interrupt void Timer0_ISR (void) {
    if(iNum >= 0) {
        UCA1TXBUF = iNum;
        iNum--;
    }
    else
        P1OUT |= BIT0;    // Turn on red LED
}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI(void) {
    if(strcmp(UCA1RXBUF, "31") == 0)
        P9OUT |= BIT7;
    else
        P9OUT ^= BIT7;
}
