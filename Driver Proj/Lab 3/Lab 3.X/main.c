/*
 * File:   main.c
 * Author: R Vyas
 *  GROUP 2
 * Thomas Mattern 30147476
 * Marcos Perez 30141681
 * 
 * Created on September 25, 2023, 2:52 PM
 */



// LINKED .h SOURCE FILES HERE
#include <xc.h>
#include <p24fxxxx.h>
#include <p24F16KA101.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>


//// CONFIGURATION BITS - PRE-PROCESSOR DIRECTIVES ////

// Code protection 
#pragma config BSS = OFF // Boot segment code protect disabled
#pragma config BWRP = OFF // Boot sengment flash write protection off
#pragma config GCP = OFF // general segment code protecion off
#pragma config GWRP = OFF

// CLOCK CONTROL 
#pragma config IESO = OFF    // 2 Speed Startup disabled
#pragma config FNOSC = FRC  // Start up CLK = 8 MHz
#pragma config FCKSM = CSECMD // Clock switching is enabled, clock monitor disabled
#pragma config SOSCSEL = SOSCLP // Secondary oscillator for Low Power Operation
#pragma config POSCFREQ = MS  //Primary Oscillator/External clk freq betwn 100kHz and 8 MHz. Options: LS, MS, HS
#pragma config OSCIOFNC = ON  //CLKO output disabled on pin 8, use as IO. 
#pragma config POSCMOD = NONE  // Primary oscillator mode is disabled

// WDT
#pragma config FWDTEN = OFF // WDT is off
#pragma config WINDIS = OFF // STANDARD WDT/. Applicable if WDT is on
#pragma config FWPSA = PR32 // WDT is selected uses prescaler of 32
#pragma config WDTPS = PS1 // WDT postscler is 1 if WDT selected

//MCLR/RA5 CONTROL
#pragma config MCLRE = OFF // RA5 pin configured as input, MCLR reset on RA5 diabled

//BOR  - FPOR Register
#pragma config BORV = LPBOR // LPBOR value=2V is BOR enabled
#pragma config BOREN = BOR0 // BOR controlled using SBOREN bit
#pragma config PWRTEN = OFF // Powerup timer disabled
#pragma config I2C1SEL = PRI // Default location for SCL1/SDA1 pin

//JTAG FICD Register
#pragma config BKBUG = OFF // Background Debugger functions disabled
#pragma config ICS = PGx2 // PGC2 (pin2) & PGD2 (pin3) are used to connect PICKIT3 debugger

// Deep Sleep RTCC WDT
#pragma config DSWDTEN = OFF // Deep Sleep WDT is disabled
#pragma config DSBOREN = OFF // Deep Sleep BOR is disabled
#pragma config RTCOSC = LPRC// RTCC uses LPRC 32kHz for clock
#pragma config DSWDTOSC = LPRC // DeepSleep WDT uses Lo Power RC clk
#pragma config DSWDTPS = DSWDTPS7 // DSWDT postscaler set to 32768 


// GLOBAL VARIABLES



// MACROS
#define Nop() {__asm__ volatile ("nop");}
#define ClrWdt() {__asm__ volatile ("clrwdt");}
#define Sleep() {__asm__ volatile ("pwrsav #0");}   // set sleep mode
#define Idle() {__asm__ volatile ("pwrsav #1");}
#define dsen() {__asm__ volatile ("BSET DSCON, #15");}

// TODO implement delay, there should be a shortest possible delay for PB1 and PB2, must explain
void delay_ms(uint16_t time_ms) {
    uint16_t clkval;
    
    if(time_ms <= 16){
        clkval = 8;
    }
    else if(time_ms > 16 && time_ms <= 262)
    {
        clkval = 500;
    }
    else
    {
        clkval = 32;
    }
        
    NewClk(clkval);
    
    T2CONbits.TSIDL = 0; //operate in idle mode
    T2CONbits.T32 = 0; //operate timer 2 as 16 bit timer
    T2CONbits.TCKPS = 0b00; //prescaler = 1
    T2CONbits.TCS = 0; //use internal clock
    
    IPC1bits.T2IP = 7; //setting interrupt priority to highest
    IFS0bits.T2IF = 0; //clear timer 2 flag
    IEC0bits.T2IE = 1; //enable timer 2 interrupt
    
    TMR2 = 0; //clear TMR2 register
    PR2 = (time_ms * (clkval/2)); //calculating pr2 for clock of clkval freq
    
    T2CONbits.TON = 1; //starting timer 2
    
    
    
    Idle();
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0; //clear timer 2 interrupt flag
    TMR2 = 0;
    T2CONbits.TON = 0; //stop timer
    
    return;
}
        
int main(void) {
    
     //Clock output on REFO pin - pin 18 RB15
     TRISBbits.TRISB15 = 0;  // Set RB15 as output for REFO
     REFOCONbits.ROEN = 1; // Ref oscillator is enabled
     REFOCONbits.ROSSLP = 0; // Ref oscillator is disabled in sleep
     REFOCONbits.ROSEL = 0; // Output base clk showing clock switching
     REFOCONbits.RODIV = 0b0000;
     
     //IO Inititalizations
     // RA4 RB4 and RA2 input, 
     TRISBbits.TRISB4 = 1;
     TRISAbits.TRISA4 = 1;
     TRISAbits.TRISA2 = 1;
     
     AD1PCFG = 0xFFFF;  // Turn all analog pins to digital
     CNPU1bits.CN0PUE = 1; // Enables pull up resistor on RA4/CN0
     CNPU1bits.CN1PUE = 1; // Enables pull up resistor on RB4/CN1
     CNPU2bits.CN30PUE = 1; // Enables pull up resistor on RA2/CN30
    // RB8 output
     TRISBbits.TRISB8 = 0;
    // RA2 = PB1 (led blinks at 1s interval)
    // RB4 = PB2 (led blinks at 2s interval)
    // RA4 = PB3 (led blinks at 3s intervals)
    // > 1 PB's on led stays on
    // 0 PB's means no LEDS
    while (1) {
        // variables to track which buttons on
        int PB1 = 0; // RA2
        int PB2 = 0; // RA4
        int PB3 = 0; // RB8
        // check which buttons are pressed 
        if (PORTAbits.RA2 == 0) PB1++;
        if (PORTAbits.RA4 == 0) PB2++;
        if (PORTBbits.RB4 == 0) PB2++;
        // if 0 buttons are on set output to 0, led off
        if (PB1 == 0 && PB2 == 0 && PB3 == 0) {
            LATBbits.LATB8 = 0;  
        }
        // if there is more than 1 button on, leave led on until button is released
        else if(PB1 == 1 && PB3 == 1){
             LATBbits.LATB8 = 1;
            delay_ms(1);
            LATBbits.LATB8 = 0;
            delay_ms(1);
        }
        // if RA4 or PB3 is on, turn on led, delay for 3 seconds, then turn off
        else if (PB3) {
            LATBbits.LATB8 = 1;
            delay_ms(3000);
            LATBbits.LATB8 = 0;
            delay_ms(3000);
        }
        // if RB4 or PB2 is on, turn on led, delay for 2 seconds, then turn off
        else if (PB2) {
            LATBbits.LATB8 = 1;
            delay_ms(2000);
            LATBbits.LATB8 = 0;
            delay_ms(2000);
        } 
        // if RA2 or PB1 is on, turn on led, delay for 2 seconds, then turn off
        else if (PB1) {
            LATBbits.LATB8 = 1;
            delay_ms(1000);
            LATBbits.LATB8 = 0;
            delay_ms(1000);
        }
        // reset active buttons
        PB1 = 0;
        PB2 = 0;
        PB3 = 0;

    }
    
    return 0;
}