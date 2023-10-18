/*
 * File:   main.c
 * Author: marcos
 *
 * Created on September 21, 2023, 1:27 PM
 */


#include "xc.h"
#include <p24F16KA101.h>

//UCID = 30141681

int main(void) {
    TRISA = TRISA | 0x0017; //Activates input on A[0-2],[4]
    
    TRISB = TRISB & 0xcc68; //Activates output on B (1100 1100 0110 1000)
    
    while(1){
        uint16_t INPUT = PORTA & 0x0017;
        
        //PORTA & 0x0001 == 0x0001
        if(INPUT == 0x0001){
        LATB = LATB | 0x0007; //Output = 3 (0 0000 0XXX)
        }
        else if(INPUT == 0x0002){
            LATB = LATB | 0x0000; //Output = 0 (0 0000 0000)
        }
        else if(INPUT == 0x0003){
            LATB = LATB | 0x0001; //Output = 1 (0 0000 000X)
        }
        else if(INPUT == 0x0004){
            LATB = LATB | 0x0017; //Output = 4 (0 0000 XXXX)
        }
        else if(INPUT == 0x0005){
            LATB = LATB | 0x0001; //Output = 1 (0 0000 000X)
        }
        else if(INPUT == 0x0006){
            LATB = LATB | 0x01d7; //Output = 6 (0 00XX XXXX)
        }
        else if(INPUT == 0x0007){
            LATB = LATB | 0x13d7; //Output = 8 (0 XXXX XXXX)
        }
        else if(INPUT == 0x0010){
            LATB = LATB | 0x0001; //Output = 1 (0 0000 000X)
        }
        else if(INPUT == 0x0011){
            LATB = LATB | 0x0000; //Output = 0 (0 0000 0000)
        }
        
        LATB = 0x0000; //Reset LED status
    }
    
    
    
    return 0;
}
