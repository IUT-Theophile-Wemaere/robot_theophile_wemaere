 /*
 * File:   IO.c
 */

#include <xc.h>
#include "IO.h"

void InitIO()
{
    // IMPORTANT : désactiver les entrées analogiques, sinon on perd les entrées numériques
    ANSELA = 0; // 0 desactive
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    ANSELF = 0;
    ANSELG = 0;
    
    
    //********** Configuration des sorties : _TRISxx = 0 ********************************
    // LED
    
    _TRISC10 = 0; // LED Orange
    _TRISG6 = 0; //LED Blanche
    _TRISG7 = 0; // LED Bleue
    
    // Moteurs 
    _TRISB14=0; //Moteur 1 IN1
    _TRISB15=0; //moteur 1 IN2
    
    _TRISC6=0; //Moteur 6 IN1
    _TRISC7=0; //moteur 6 IN2

    //********** Configuration des entrées : _TRISxx = 1 ********************************    
    _TRISA1=1; //JACK
    
    // échange données PC
    _U1RXR=24; // config sortie RP vers entrée Rx1
    _RP36R=0b00001; // config sortie Tx vers entrée RP 
    
    //QEI (odométrie)
    _QEA2R = 97;    //assign QEI A to pin RP97
    _QEB2R = 96;    //assign QEI B to pin RP96
    
    _QEA1R = 70;    //assign QEI A to pin RP70
    _QEB1R = 69;    //assign QEI B to pin RP69
}
