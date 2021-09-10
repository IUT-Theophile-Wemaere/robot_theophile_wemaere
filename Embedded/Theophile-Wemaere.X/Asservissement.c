#include "Asservissement.h"
#include "robot.h"

void SetUpPiAsservissementVitesseAngulaire() {
    //Reglage de Ziegler Nichols sans depassements : un tout petit peu mou
    //Implémente un correcteur PI   
    robotState.KpAngulaire = 50;
    robotState.KiAngulaire = 1;
    robotState.KdAngulaire = 0;

    robotState.KpAngulaireMax = 5000;
    robotState.KiAngulaireMax = 5000;
    robotState.KdAngulaireMax = 100;
}

void SetUpPiAsservissementVitesseLineaire() {
    //Reglage de Ziegler Nichols sans depassements : un tout petit peu mou
    //Implémente un correcteur PI
    robotState.KpLineaire = 0;
    robotState.KiLineaire = 0;
    robotState.KdLineaire = 0;
}