#include "Asservissement.h"
#include "robot.h"

void SetUpPiAsservissementVitesseAngulaire() {
    //Reglage de Ziegler Nichols sans depassements : un tout petit peu mou
    //Implémente un correcteur PI   
    robotState.KpAngulaire = 1; 
    robotState.KiAngulaire = 2;
    robotState.KdAngulaire = 3;

    robotState.KpAngulaireMax = 4;
    robotState.KiAngulaireMax = 5;
    robotState.KdAngulaireMax = 6;
}

void SetUpPiAsservissementVitesseLineaire() {
    //Reglage de Ziegler Nichols sans depassements : un tout petit peu mou
    //Implémente un correcteur PI
    robotState.KpLineaire = 1.1; //8
    robotState.KiLineaire = 2.1; //112
    robotState.KdLineaire = 3.1;
    
    robotState.KpLineaireMax = 4.1;
    robotState.KiLineaireMax = 5.1;
    robotState.KdLineaireMax = 6.1;
}