#include "QEI.h"
#include <p33EP512GM306.h>
#include "robot.h"
#include <math.h>
#include "Utilities.h"
#include "timer.h"
#include "UART_Protocol.h"

#define DISTROUES 0.218
#define WHEEL_DIAMETER 0.0426
#define POINT_TO_METER  (WHEEL_DIAMETER * PI / 8192.0)
#define FREQ_ECH_QEI 250
#define POSITION_DATA 0x0061

double QeiDroitPosition_T_1,QeiGauchePosition_T_1,QeiDroitPosition,QeiGauchePosition,delta_d,delta_g,delta_theta ,dx;

void InitQEI1(void)
{
    QEI1IOCbits.SWPAB = 1; //QEAx and QEBx are swapped
    QEI1GECL = 0xFFFF;
    QEI1GECH = 0xFFFF;
    QEI1CONbits.QEIEN = 1; // Enable QEI Module
}

void InitQEI2(void)
{
    QEI2IOCbits.SWPAB = 1; //QEAx and QEBx are not swapped
    QEI2GECL = 0xFFFF;
    QEI2GECH = 0xFFFF;
    QEI2CONbits.QEIEN = 1; // Enable QEI Module
}

void QEIUpdateData(void)
{
    
    // on sauvgarde les ancinees valeurs 
    QeiDroitPosition_T_1 = QeiDroitPosition;
    QeiGauchePosition_T_1 = QeiGauchePosition;
    
    if(reset==1)
    {
        robotState.xPosFromOdometry = 0;
        robotState.yPosFromOdometry = 0;
        robotState.angleRadianFromOdometry = 0;
        reset=0;
    }

    //On réactualise les valeurs des positions
    long QEI1RawValue =POS1CNTL;
    QEI1RawValue+=((long)POS1HLD<<16);

    long QEI2RawValue = POS2CNTL;
    QEI2RawValue += ((long)POS2HLD<<16);

    // Conversion en mm ( r\ 'egl \ 'e pour la taille des roues codeuses )
    QeiDroitPosition = -POINT_TO_METER*QEI1RawValue ;
    QeiGauchePosition = POINT_TO_METER*QEI2RawValue;

    //calcul des deltas de positions
    delta_d = QeiDroitPosition - QeiDroitPosition_T_1; 
    delta_g = QeiGauchePosition - QeiGauchePosition_T_1; 

    // delta_theta = (delta_d - detla_g) / DISTROUES;
    delta_theta = (delta_d - delta_g)/(DISTROUES); 
    dx =(delta_d + delta_g)/2; 

    //calcul des vitesses
    // attention à remultiplier par la fréquence d?échantillonnage
    robotState.vitesseDroitFromOdometry = delta_d*FREQ_ECH_QEI;
    robotState.vitesseGaucheFromOdometry = delta_g*FREQ_ECH_QEI;
    robotState.vitesseLineaireFromOdometry = (robotState.vitesseDroitFromOdometry + robotState.vitesseGaucheFromOdometry)/2 ;
    robotState.vitesseAngulaireFromOdometry = delta_theta*FREQ_ECH_QEI;

    //Mise à jour du postionnement  terrain à t-1
    robotState.xPosFromOdometry_1 = robotState.xPosFromOdometry ;
    robotState.yPosFromOdometry_1 = robotState.yPosFromOdometry ;
    robotState.angleRadianFromOdometry_1 = robotState.angleRadianFromOdometry ;

    // Calcul des positions dans le referentiel du terrain
    robotState.xPosFromOdometry = robotState.xPosFromOdometry_1 + (robotState.vitesseLineaireFromOdometry/FREQ_ECH_QEI )* cos(robotState.angleRadianFromOdometry_1);
    robotState.yPosFromOdometry = robotState.yPosFromOdometry_1 + (robotState.vitesseLineaireFromOdometry/FREQ_ECH_QEI )* sin(robotState.angleRadianFromOdometry_1);
    robotState.angleRadianFromOdometry = robotState.angleRadianFromOdometry_1 + delta_theta; 
    if ( robotState.angleRadianFromOdometry > PI )
        robotState.angleRadianFromOdometry -= 2*PI ;
    if ( robotState.angleRadianFromOdometry < -PI )
        robotState.angleRadianFromOdometry += 2*PI ;
} 

void SendPositionData(void)
{
    unsigned char positionPayload[24];
    getBytesFromInt32(positionPayload,0,timestamp ) ;
    getBytesFromFloat ( positionPayload,4,(float)(robotState.xPosFromOdometry)) ;
    getBytesFromFloat ( positionPayload,8,(float)(robotState.yPosFromOdometry)) ;
    getBytesFromFloat ( positionPayload,12,(float)(robotState.angleRadianFromOdometry));
    getBytesFromFloat ( positionPayload,16,(float)(robotState.vitesseLineaireFromOdometry)) ;
    getBytesFromFloat ( positionPayload,20,(float)(robotState.vitesseAngulaireFromOdometry)) ;
    UartEncodeAndSendMessage (POSITION_DATA,24,positionPayload) ;
}
