#include "Asservissement.h"
#include "robot.h"
#include "QEI.h"
#include "Utilities.h"

void SetupPidAssservissement(volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd,
        double proportionelleMax, double integraleMax, double deriveeMax) {
    PidCorr -> Kp = Kp/10; 
    // limite ang : 8 => 4
    // limite lin : 9.2 => 4.6
    PidCorr -> erreurProportionnelleMax = proportionelleMax;
    PidCorr -> Ki = Ki; 
    // limite ang : 100 => 70
    //limite lim 110 => 77
    PidCorr -> erreurIntegraleMax = integraleMax;
    PidCorr -> Kd = Kd/1000; 
    // limite ang : 0.035 => 0.03
    // Limite lim : 0.005 => 0.002
    PidCorr -> erreurDeriveeMax = deriveeMax;
}

double Correcteur(volatile PidCorrector* PidCorr, double erreur) {
    PidCorr -> erreur = erreur;
    double erreurProportionnelle = LimitToInterval(erreur, -PidCorr->erreurProportionnelleMax, PidCorr->erreurProportionnelleMax);
    PidCorr -> corrP = erreurProportionnelle * PidCorr->Kp;

    PidCorr -> erreurIntegrale += erreur / FREQ_ECH_QEI;
    PidCorr -> erreurIntegrale = LimitToInterval(PidCorr->erreurIntegrale, -PidCorr->erreurIntegraleMax / PidCorr->Ki, PidCorr->erreurIntegraleMax / PidCorr->Ki);
    PidCorr -> corrI = PidCorr->erreurIntegrale * PidCorr->Ki;

    double erreurDerivee = (erreur - PidCorr->epsilon_1) * FREQ_ECH_QEI;
    double deriveeBornee = LimitToInterval(erreurDerivee, -PidCorr->erreurDeriveeMax / PidCorr->Kd, PidCorr->erreurDeriveeMax / PidCorr->Kd);
    PidCorr->epsilon_1 = erreur;
    PidCorr->corrD = deriveeBornee * PidCorr->Kd;

    return (PidCorr->corrP + PidCorr->corrI + PidCorr->corrD);
}

//void SetUpPiAsservissementVitesseAngulaire() {
//    Reglage de Ziegler Nichols sans depassements : un tout petit peu mou
//    Implémente un correcteur PI   
//    robotState.KpAngulaire = 0; // Limite : 6 => kp = 3 
//    robotState.KiAngulaire = 0;
//    robotState.KdAngulaire = 0;
//
//    robotState.KpAngulaireMax = 1000;
//    robotState.KiAngulaireMax = 1000;
//    robotState.KdAngulaireMax = 1000;
//}
//
//void SetUpPiAsservissementVitesseLineaire() {
//    Reglage de Ziegler Nichols sans depassements : un tout petit peu mou
//    Implémente un correcteur PI
//    robotState.KpLineaire = 0;  //limite : 5.7 => kp = 2.8
//    robotState.KiLineaire = 0; 
//    robotState.KdLineaire = 0;
//    
//    robotState.KpLineaireMax = 1000;
//    robotState.KiLineaireMax = 1000;
//    robotState.KdLineaireMax = 1000;
//}

