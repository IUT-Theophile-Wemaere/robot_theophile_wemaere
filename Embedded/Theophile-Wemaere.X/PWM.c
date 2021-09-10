#include <xc.h> // library xc.h inclut tous les uC
#include "IO.h"
#include "PWM.h"
#include "robot.h"
#include "toolBox.h"
#include "timer.h"
#include "QEI.h"
#include "Utilities.h"
#include "UART_Protocol.h"


#define PWMPER 40.0
float acceleration = 15;

void InitPWM(void) {
    PTCON2bits.PCLKDIV = 0b000; //Divide by 1
    PTPER = 100 * PWMPER; //Période en pourcentage

    //Réglage PWM moteur 1 sur hacheur 1
    IOCON1bits.POLH = 1; //High = 1 and active on low =0
    IOCON1bits.POLL = 1; //High = 1
    IOCON1bits.PMOD = 0b01; //Set PWM Mode to Redundant
    FCLCON1 = 0x0003; //Désactive la gestion des faults

    //Reglage PWM moteur 2 sur hacheur 6
    IOCON6bits.POLH = 1; //High = 1
    IOCON6bits.POLL = 1; //High = 1
    IOCON6bits.PMOD = 0b01; //Set PWM Mode to Redundant
    FCLCON6 = 0x0003; //Désactive la gestion des faults

    /*Enable PWM Module */
    PTCONbits.PTEN = 1;
}

//void PWMSetSpeed(float vitesseEnPourcents, int moteur)
//{
//robotState.vitesseGaucheCommandeCourante = vitesseEnPourcents;
//if(moteur==1)
//{
//    robotState.vitesseGaucheCommandeCourante = vitesseEnPourcents;
//    if(vitesseEnPourcents>0)
//    {
//        MOTEUR_GAUCHE_L_PWM_ENABLE = 0; //Pilotage  de la pin en mode IO
//        MOTEUR_GAUCHE_L_IO_OUTPUT = 1; //Mise à 1 de la pin
//        MOTEUR_GAUCHE_H_PWM_ENABLE = 1; //Pilotage de la pin en mode PWM
//        MOTEUR_GAUCHE_DUTY_CYCLE = Abs(robotState.vitesseGaucheCommandeCourante*PWMPER); 
//    }
//    else
//    {
//       MOTEUR_GAUCHE_L_PWM_ENABLE = 1; //Pilotage  de la pin en mode IO
//       MOTEUR_GAUCHE_H_IO_OUTPUT = 1; //Mise à 1 de la pin
//       MOTEUR_GAUCHE_H_PWM_ENABLE = 0; //Pilotage de la pin en mode PWM
//       MOTEUR_GAUCHE_DUTY_CYCLE = Abs(robotState.vitesseGaucheCommandeCourante*PWMPER); 
//    }
//}
//else
//{
//    robotState.vitesseDroiteCommandeCourante = vitesseEnPourcents;
//    if(vitesseEnPourcents>0)
//    {
//        MOTEUR_DROIT_L_PWM_ENABLE = 0; //Pilotage  de la pin en mode IO
//        MOTEUR_DROIT_L_IO_OUTPUT = 1; //Mise à 1 de la pin
//        MOTEUR_DROIT_H_PWM_ENABLE = 1; //Pilotage de la pin en mode PWM
//        MOTEUR_DROIT_DUTY_CYCLE = Abs(robotState.vitesseDroiteCommandeCourante*PWMPER); 
//    }
//    else
//    {
//       MOTEUR_DROIT_L_PWM_ENABLE = 1; //Pilotage  de la pin en mode IO
//       MOTEUR_DROIT_H_IO_OUTPUT = 1; //Mise à 1 de la pin
//       MOTEUR_DROIT_H_PWM_ENABLE = 0; //Pilotage de la pin en mode PWM
//       MOTEUR_DROIT_DUTY_CYCLE = Abs(robotState.vitesseDroiteCommandeCourante*PWMPER); 
//    }
//}
//}

void PWMUpdateSpeed(void) {
    // Cette fonction est appelée sur timer et permet de suivre des rampes d?accélération
    if (robotState.vitesseDroiteCommandeCourante < robotState.vitesseDroiteConsigne)
        robotState.vitesseDroiteCommandeCourante = Min1(robotState.vitesseDroiteCommandeCourante + acceleration, robotState.vitesseDroiteConsigne);

    if (robotState.vitesseDroiteCommandeCourante > robotState.vitesseDroiteConsigne)
        robotState.vitesseDroiteCommandeCourante = Max1(robotState.vitesseDroiteCommandeCourante - acceleration, robotState.vitesseDroiteConsigne);

    if (robotState.vitesseDroiteCommandeCourante > 0) {
        MOTEUR_DROIT_H_PWM_ENABLE = 0; //pilotage de la pin en mode IO
        MOTEUR_DROIT_H_IO_OUTPUT = 1; //Mise à 1 de la pin
        MOTEUR_DROIT_L_PWM_ENABLE = 1; //Pilotage de la pin en mode PWM
    } else {

        MOTEUR_DROIT_L_PWM_ENABLE = 0; //pilotage de la pin en mode IO
        MOTEUR_DROIT_L_IO_OUTPUT = 1; //Mise à 1 de la pin
        MOTEUR_DROIT_H_PWM_ENABLE = 1; //Pilotage de la pin en mode PWM
    }
    MOTEUR_DROIT_DUTY_CYCLE = Abs1(robotState.vitesseDroiteCommandeCourante) * PWMPER;

    if (robotState.vitesseGaucheCommandeCourante < robotState.vitesseGaucheConsigne)
        robotState.vitesseGaucheCommandeCourante = Min1(robotState.vitesseGaucheCommandeCourante + acceleration, robotState.vitesseGaucheConsigne);

    if (robotState.vitesseGaucheCommandeCourante > robotState.vitesseGaucheConsigne)
        robotState.vitesseGaucheCommandeCourante = Max1(robotState.vitesseGaucheCommandeCourante - acceleration, robotState.vitesseGaucheConsigne);

    if (robotState.vitesseGaucheCommandeCourante > 0) {
        MOTEUR_GAUCHE_L_PWM_ENABLE = 0; //pilotage de la pin en mode IO
        MOTEUR_GAUCHE_L_IO_OUTPUT = 1; //Mise à 1 de la pin
        MOTEUR_GAUCHE_H_PWM_ENABLE = 1; //Pilotage de la pin en mode PWM
    } else {
        MOTEUR_GAUCHE_H_PWM_ENABLE = 0; //pilotage de la pin en mode IO
        MOTEUR_GAUCHE_H_IO_OUTPUT = 1; //Mise à 1 de la pin
        MOTEUR_GAUCHE_L_PWM_ENABLE = 1; //Pilotage de la pin en mode PWM
    }
    MOTEUR_GAUCHE_DUTY_CYCLE = Abs1(robotState.vitesseGaucheCommandeCourante) * PWMPER;
}

void PWMSetSpeedConsigne(float vitesseEnpourcents, int moteur) {
    if (moteur == 0) {
        robotState.vitesseDroiteConsigne = -vitesseEnpourcents;
    } else {
        robotState.vitesseGaucheConsigne = -vitesseEnpourcents;
    }
}

void PWMSetSpeedConsignePolaire() {
    /****************** CorrectionAngulaire **********************/
    robotState.vitesseAngulaireErreur = robotState.vitesseAngulaireConsigne - robotState.vitesseAngulaireFromOdometry;

    robotState.CorrectionAngulaireKp = robotState.KpAngulaire * robotState.vitesseAngulaireErreur;
    robotState.CorrectionAngulaireKp = LimitToInterval1(robotState.CorrectionAngulaireKp, -robotState.KpAngulaireMax, robotState.KpAngulaireMax);
    robotState.CorrectionAngulaireKi = (robotState.KiAngulaire * robotState.vitesseAngulaireErreur) / FREQ_ECH_QEI + robotState.CorrectionAngulaireKi;
    robotState.CorrectionAngulaireKi = LimitToInterval1(robotState.CorrectionAngulaireKi, -robotState.KiAngulaireMax, robotState.KiAngulaireMax);

    robotState.CorrectionAngulaireKd = (robotState.vitesseAngulaireFromOdometry_1 - robotState.vitesseAngulaireFromOdometry) * FREQ_ECH_QEI;
    robotState.CorrectionAngulaireKd = LimitToInterval1(robotState.CorrectionAngulaireKd, -robotState.KdAngulaireMax, robotState.KdAngulaireMax);

    robotState.vitesseAngulaireCorrection = robotState.CorrectionAngulaireKp + robotState.CorrectionAngulaireKi;
    //robotState.vitesseAngulaireCorrection = CorrecteurVitesseAngulaire(robotState.vitesseAngulaireErreur);
    robotState.vitesseAngulaireCommande = robotState.vitesseAngulaireCorrection * COEFF_VITESSE_ANGULAIRE_PERCENT;
 
    /********************** Correction Lineaire *******************************/
    robotState.vitesseLineaireErreur = robotState.vitesseLineaireConsigne - robotState.vitesseLineaireFromOdometry;

    robotState.CorrectionLineaireKp = robotState.KpLineaire * robotState.vitesseLineaireErreur;
    robotState.CorrectionLineaireKp = LimitToInterval1(robotState.CorrectionLineaireKp, -robotState.KpLineaireMax, robotState.KpLineaireMax);
    robotState.CorrectionLineaireKi = (robotState.KiLineaire * robotState.vitesseLineaireErreur) / FREQ_ECH_QEI + robotState.CorrectionLineaireKi;
    robotState.CorrectionLineaireKi = LimitToInterval1(robotState.CorrectionLineaireKi, -robotState.KiLineaireMax, robotState.KiLineaireMax);

    robotState.vitesseLineaireCorrection = robotState.CorrectionLineaireKp + robotState.CorrectionLineaireKi;
    //robotState.vitesseLineaireCorrection = CorrecteurVitesseLineaire(robotState.vitesseLineaireErreur);
    robotState.vitesseLineaireCommande = robotState.vitesseLineaireCorrection * COEFF_VITESSE_LINEAIRE_PERCENT;

    robotState.CorrectionAngulaireKd = (robotState.vitesseLineaireFromOdometry_1 - robotState.vitesseLineaireFromOdometry) * FREQ_ECH_QEI;
    robotState.CorrectionAngulaireKd = LimitToInterval1(robotState.CorrectionAngulaireKd, -robotState.KdAngulaireMax, robotState.KdAngulaireMax);

    /************* Génération des consignes droites et gauches ******************/
    robotState.vitesseDroiteConsigne = robotState.vitesseLineaireCommande + robotState.vitesseAngulaireCommande * DISTROUES / 2;
    robotState.vitesseDroiteConsigne = LimitToInterval1(robotState.vitesseDroiteConsigne, -100, 100);
    robotState.vitesseGaucheConsigne = robotState.vitesseLineaireCommande - robotState.vitesseAngulaireCommande * DISTROUES / 2;
    robotState.vitesseGaucheConsigne = LimitToInterval1(robotState.vitesseGaucheConsigne, -100, 100);
}

void SendPIDData(void) {
    unsigned char payloadPID[64];

    getBytesFromFloat(payloadPID, 0, (float) robotState.vitesseAngulaireErreur);
    getBytesFromFloat(payloadPID, 4, (float) (robotState.vitesseAngulaireCommande));
    getBytesFromFloat(payloadPID, 8, (float) (robotState.KpAngulaire));
    getBytesFromFloat(payloadPID, 12, (float) (robotState.CorrectionAngulaireKp));
    getBytesFromFloat(payloadPID, 16, (float) (robotState.KiAngulaire));
    getBytesFromFloat(payloadPID, 20, (float) (robotState.CorrectionAngulaireKi));
    getBytesFromFloat(payloadPID, 24, (float) (robotState.KdAngulaire));
    getBytesFromFloat(payloadPID, 28, (float) (robotState.CorrectionAngulaireKd));

    getBytesFromFloat(payloadPID, 32, (float) robotState.vitesseLineaireErreur);
    getBytesFromFloat(payloadPID, 36, (float) (robotState.vitesseLineaireCommande));
    getBytesFromFloat(payloadPID, 40, (float) (robotState.KpLineaire));
    getBytesFromFloat(payloadPID, 44, (float) (robotState.CorrectionLineaireKp));
    getBytesFromFloat(payloadPID, 48, (float) (robotState.KiLineaire));
    getBytesFromFloat(payloadPID, 52, (float) (robotState.CorrectionLineaireKi));
    getBytesFromFloat(payloadPID, 56, (float) (robotState.KdLineaire));
    getBytesFromFloat(payloadPID, 60, (float) (robotState.CorrectionLineaireKd));
    UartEncodeAndSendMessage(ASSERVISSEMENT, 64, payloadPID);
}