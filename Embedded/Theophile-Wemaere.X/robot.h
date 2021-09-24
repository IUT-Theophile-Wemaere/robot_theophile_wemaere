#ifndef ROBOT_H
#define ROBOT_H
//float acceleration = 5;
#include "Asservissement.h"

typedef struct robotStateBITS {

    union {

        struct {
            unsigned char taskEnCours;
            float vitesseGaucheConsigne;
            float vitesseGaucheCommandeCourante;
            float vitesseDroiteConsigne;
            float vitesseDroiteCommandeCourante;
            float distanceTelemetreDroit;
            float distanceTelemetreDroit2;
            float distanceTelemetreCentre;
            float distanceTelemetreGauche;
            float distanceTelemetreGauche2;
            double vitesseDroitFromOdometry;
            double vitesseGaucheFromOdometry;
            double vitesseLineaireFromOdometry;
            double vitesseAngulaireFromOdometry;
            double vitesseLineaireFromOdometry_1;
            double vitesseAngulaireFromOdometry_1;
            double xPosFromOdometry_1;
            double yPosFromOdometry_1;
            double angleRadianFromOdometry_1;
            double xPosFromOdometry;
            double yPosFromOdometry;
            double angleRadianFromOdometry;

            double KpLineaire;
            double KiLineaire;
            double KdLineaire;
            double KpLineaireMax;
            double KiLineaireMax;
            double KdLineaireMax;

            double KpAngulaire;
            double KiAngulaire;
            double KdAngulaire;
            double KpAngulaireMax;
            double KiAngulaireMax;
            double KdAngulaireMax;

            double vitesseLineaireConsigne;
            double vitesseLineaireCommande;
            double vitesseLineaireErreur;
            double vitesseLineaireCorrection;
            double CorrectionLineaireKp;
            double CorrectionLineaireKi;
            double CorrectionLineaireKd;

            double vitesseAngulaireConsigne;
            double vitesseAngulaireCommande;
            double vitesseAngulaireErreur;
            double vitesseAngulaireCorrection;
            double CorrectionAngulaireKp;
            double CorrectionAngulaireKi;
            double CorrectionAngulaireKd;
            
            PidCorrector PidX, PidTheta;
            double xCorrectionVitessePourcent;
            double thetaCorrectionVitessePourcent;
        }
        ;
    }
    ;
} ROBOT_STATE_BITS;

extern volatile ROBOT_STATE_BITS robotState;
#endif /* ROBOT_H */
