#include "Asservissement.h"
#include "robot.h"

void SetUpPiAsservissementVitesseAngulaire() {
    //Reglage de Ziegler Nichols sans depassements : un tout petit peu mou
    //Implémente un correcteur PI   
    robotState.KpAngulaire = 0; // Limite : 6 => kp = 3 
    robotState.KiAngulaire = 0;
    robotState.KdAngulaire = 0;

    robotState.KpAngulaireMax = 1000;
    robotState.KiAngulaireMax = 1000;
    robotState.KdAngulaireMax = 1000;
}

void SetUpPiAsservissementVitesseLineaire() {
    //Reglage de Ziegler Nichols sans depassements : un tout petit peu mou
    //Implémente un correcteur PI
    robotState.KpLineaire = 0;  //limite : 5.7 => kp = 2.8
    robotState.KiLineaire = 0; 
    robotState.KdLineaire = 0;
    
    robotState.KpLineaireMax = 1000;
    robotState.KiLineaireMax = 1000;
    robotState.KdLineaireMax = 1000;
}