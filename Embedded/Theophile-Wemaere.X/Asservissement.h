#ifndef ASSERVISSEMENT_H
#define	ASSERVISSEMENT_H

void SetUpPiAsservissementVitesseAngulaire();
void SetUpPiAsservissementVitesseLineaire();

typedef struct _PidCorrector
{
    double Kp;
    double Ki;
    double Kd;
    double erreurProportionnelleMax;
    double erreurIntegraleMax;
    double erreurDeriveeMax;
    double erreurIntegrale;
    double epsilon_1;
    double erreur;
    
    //For debug only
    double corrP;
    double corrI;
    double corrD;
}PidCorrector;

void SetupPidAssservissement(volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd,
        double proportionelleMax, double integralMax, double deriveeMax);
double Correcteur(volatile PidCorrector* PidCorr, double erreur);

#endif	

