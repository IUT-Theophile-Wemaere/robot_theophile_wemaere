#include "Asservissement.h"
#include "robot.h"

void SetUpPiAsservissementVitesseAngulaire() {
    //Reglage de Ziegler Nichols sans depassements : un tout petit peu mou
    //Implémente un correcteur PI   
    robotState.KpAngulaire = 3.25; //6.50;
    robotState.KiAngulaire = 175; //230
    robotState.KdAngulaire = 0;

    robotState.KpAngulaireMax = 10;
    robotState.KiAngulaireMax = 10;
    robotState.KdAngulaireMax = 10;
}

void SetUpPiAsservissementVitesseLineaire() {
    //Reglage de Ziegler Nichols sans depassements : un tout petit peu mou
    //Implémente un correcteur PI
    robotState.KpLineaire = 4; //8
    robotState.KiLineaire = 84; //112
    robotState.KdLineaire = 0;
    
    robotState.KpLineaireMax = 10;
    robotState.KiLineaireMax = 10;
    robotState.KdLineaireMax = 10;
}