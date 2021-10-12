#ifndef PWM_H
#define PWM_H
#define MOTEUR_DROIT 0
#define MOTEUR_GAUCHE 1


void InitPWM(void);
void PWMUpdateSpeed(void);
//void PWMSetSpeed(float vitesseEnPourcents, int moteur);
void PWMSetSpeedConsigne(float vitesseEnpourcents, int moteur);

void PWMSetSpeedConsignePolaire();
void SendPIDData(void);
void UpdateAsservissemment();

#define COEFF_VITESSE_LINEAIRE_PERCENT 70
#define COEFF_VITESSE_ANGULAIRE_PERCENT 77.5

#endif /* PWM_H */