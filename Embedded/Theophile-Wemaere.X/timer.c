#include <xc.h>
#include "timer.h"
#include "IO.h"
#include "PWM.h"
#include "ADC.h"
#include "main.h"
#include "robot.h"
#include "UART_Protocol.h"
#include "QEI.h"
// Initialisation d?un timer 32 bits
unsigned long timestamp;

void InitTimer23(void) {
    T3CONbits.TON = 0; // Stop any 16?bit Timer3 operation
    T2CONbits.TON = 0; // Stop any 16/32? bit Timer3 operation
    T2CONbits.T32 = 1; // Enable 32?bit Timer mode
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TCKPS = 0b00; // S e l e c t 1: 1 P r e s c a l e r
    TMR3 = 0x00; // Clear 32bit Timer (msw)
    TMR2 = 0x00; // Clear 32bit Timer (lsw)
    PR3 = 0x04C4; // Load 32bit period value (msw)
    PR2 = 0xB400; // Load 32bit p e ri o d v al u e (lsw)
    IPC2bits.T3IP = 0x01; // Se t Timer3 I n t e r r u p t P r i o r i t y L e v el
    IFS0bits.T3IF = 0; // Cle a r Timer3 I n t e r r u p t Flag
    IEC0bits.T3IE = 1; // Enable Timer3 i n t e r r u p t
    T2CONbits.TON = 1; // S t a r t 32?b i t Timer

}
// I n t e r r u p t i o n du time r 32 b i t s s u r 2?3

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits . T3IF = 0; // Cle a r Timer3 I n t e r r u p t Flag
    //    if(toggle ==0)
    //    {
    //        PWMSetSpeed(20,MOTEUR_DROIT);
    //        PWMSetSpeed(20,MOTEUR_GAUCHE);
    //        toggle=1;
    //    }
    //    else
    //    {
    //        PWMSetSpeed(-20,MOTEUR_DROIT);
    //        PWMSetSpeed(-20,MOTEUR_GAUCHE);
    //        toggle=0;
    //    }

}
// I n i t i a l i s a t i on d'un timer 16bits

void InitTimer1(void) {
    //Timer1 pour horodater les mesures (1ms)
    T1CONbits.TON = 0; // Di s a bl e Timer
    T1CONbits.TCS = 0; // cl o c k s o u r c e = i n t e r n a l cl o c k
    SetFreqTimer1(250);
    IFS0bits.T1IF = 0; // Cle a r Timer I n t e r r u p t Flag
    IEC0bits.T1IE = 1; // Enable Timer i n t e r r u p t
    T1CONbits.TON = 1; // Enable Timer
}

void SetFreqTimer1(float freq) {
    if (FCY / freq > 65535) {
        T1CONbits.TCKPS = 0b01; //01 = 1:8 prescaler value
        if (FCY / freq / 8 > 65535) {
            T1CONbits.TCKPS = 0b10; //10 = 1:64 prescaler value
            if (FCY / freq / 64 > 65535) {
                T1CONbits.TCKPS = 0b11; //11 = 1:256 prescaler value
                PR1 = (int) (FCY / freq / 256);
            } else
                PR1 = (int) (FCY / freq / 64);
        } else
            PR1 = (int) (FCY / freq / 8);
    } else
        PR1 = (int) (FCY / freq);
}

// Interruption du timer 1
int subSamplingCounterT1 = 0;

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;
    ADC1StartConversionSequence();
    QEIUpdateData();
    UpdateAsservissemment();
    PWMSetSpeedConsignePolaire();
//    PWMUpdateSpeed();

    if (subSamplingCounterT1++ % 10 == 0) {
        //        unsigned char MOTEUR[] = {MOTEUR_GAUCHE_DUTY_CYCLE / 40, MOTEUR_DROIT_DUTY_CYCLE / 40};
        //        unsigned char LED1[] = {1, LED_ORANGE};
        //        unsigned char LED2[] = {2, LED_BLEUE};
        //        unsigned char LED3[] = {3, LED_BLANCHE};
        //        UartEncodeAndSendMessage(0x0040, 2, MOTEUR);
        //        UartEncodeAndSendMessage(0x0020, 2, LED1);
        //        UartEncodeAndSendMessage(0x0020, 2, LED2);
        //        UartEncodeAndSendMessage(0x0020, 2, LED3);
        //        unsigned char IR[] = {robotState.distanceTelemetreDroit, robotState.distanceTelemetreCentre, robotState.distanceTelemetreGauche};
        //        UartEncodeAndSendMessage(0x0030, 3, IR);

        SendPIDData();
        SendPositionData();
    }

    //LED_BLANCHE = !LED_BLANCHE; 
}

void InitTimer4(void) {
    //Timer4 pour horodater les mesures (1ms)
    T4CONbits.TON = 0; // Disable Timer
    SetFreqTimer4(1000);
    T4CONbits.TCS = 0; // clock source = internalclock
    IFS1bits.T4IF = 0; // Clear Timer Interrupt Flag
    IEC1bits.T4IE = 1; // Enable Timer interrupt
    T4CONbits.TON = 1; // Enable Timer    
}

void SetFreqTimer4(float freq) {
    T4CONbits.TCKPS = 0b00; //00 = 1:1 prescaler value
    if (FCY / freq > 65535) {
        T4CONbits.TCKPS = 0b01; //01 = 1:8 prescaler value
        if (FCY / freq / 8 > 65535) {
            T4CONbits.TCKPS = 0b10; //10 = 1:64 prescaler value
            if (FCY / freq / 64 > 65535) {
                T4CONbits.TCKPS = 0b11; //11 = 1:256 prescaler value
                PR4 = (int) (FCY / freq / 256);
            } else
                PR4 = (int) (FCY / freq / 64);
        } else
            PR4 = (int) (FCY / freq / 8);
    } else
        PR4 = (int) (FCY / freq);
}
// Interruption du timer 4

unsigned long millis = 0;

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {
    IFS1bits.T4IF = 0;
    timestamp++;
}

