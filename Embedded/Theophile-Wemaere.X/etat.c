#include "etat.h"
#include <xc.h>
#include "main.h"
#include "robot.h"

#define PAS_D_OBSTACLE 0
#define OBSTACLE_A_GAUCHE 1 
#define OBSTACLE_SUR_LA_GAUCHE 2  //tourne sur place vers la droite
#define OBSTACLE_A_DROITE 3
#define OBSTACLE_SUR_LA_DROITE 4 //tourne sur place vers la gauche
#define OBSTACLE_EN_FACE 5 //tourne sur place vers la gauche/la droite 
#define TUNNEL_GAUCHE 6
#define TUNNEL_DROITE 7

int obstacle;

int etat(void)
{ 
        if (robotState.distanceTelemetreGauche < 25 && //cas n° 1
        robotState.distanceTelemetreDroit < 25 ) 
        obstacle = OBSTACLE_EN_FACE;    
        
        else if (robotState.distanceTelemetreCentre < 35 ) //cas n° 3
        obstacle = OBSTACLE_EN_FACE; 
        
        else if (robotState.distanceTelemetreGauche < 35 && //cas n° 7
        robotState.distanceTelemetreCentre > 25 &&
        robotState.distanceTelemetreDroit > 35 ) 
        obstacle = OBSTACLE_SUR_LA_GAUCHE;  
        
        else if (robotState.distanceTelemetreGauche > 35 && //cas n° 8
        robotState.distanceTelemetreCentre > 25 &&
        robotState.distanceTelemetreDroit <  35 ) 
        obstacle = OBSTACLE_SUR_LA_DROITE;
        
        else if(robotState.distanceTelemetreGauche2 < 10 && robotState.distanceTelemetreDroit2 < 10 ) //cas tunnel
        {
            if ((robotState.distanceTelemetreGauche2 < robotState.distanceTelemetreDroit2)||(robotState.distanceTelemetreGauche < 5))
                obstacle = TUNNEL_GAUCHE  ;
            else if ((robotState.distanceTelemetreGauche2 > robotState.distanceTelemetreDroit2)||(robotState.distanceTelemetreDroit < 5))
                obstacle = TUNNEL_DROITE  ;
//            else if (robotState.distanceTelemetreGauche2 == robotState.distanceTelemetreDroit2)
//                obstacle = PAS_D_OBSTACLE  ; 
        }
         
        else if ( robotState.distanceTelemetreGauche2 < 15) //cas n° 30 
        obstacle = OBSTACLE_A_GAUCHE;
        
        else if (robotState.distanceTelemetreDroit2 < 15) //cas n° 30
        obstacle = OBSTACLE_A_DROITE;
        
        else 
        obstacle = PAS_D_OBSTACLE;

//        else if ( robotState.distanceTelemetreGauche2 > gauche2 && //cas n° 2
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 > droit2) 
//        obstacle = OBSTACLE_SUR_LA_DROITE;
//
//
//        else if ( robotState.distanceTelemetreGauche2 > gauche2 && //cas n° 4
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 > droit2) 
//        obstacle = OBSTACLE_SUR_LA_DROITE;
//
//        else if ( robotState.distanceTelemetreGauche2 > gauche2 && //cas n° 5
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit > droit &&
//        robotState.distanceTelemetreDroit2 > droit2) 
//        obstacle = OBSTACLE_SUR_LA_GAUCHE;
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 6
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre > centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 < droit2) 
//        obstacle = OBSTACLE_CUL_DE_SAC;  
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 9
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre > centre &&
//        robotState.distanceTelemetreDroit > droit &&
//        robotState.distanceTelemetreDroit2 > droit2) 
//        obstacle = OBSTACLE_A_GAUCHE;
//
//        else if ( robotState.distanceTelemetreGauche2 > gauche2 && //cas n° 10
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre > centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 < droit2) 
//        obstacle = OBSTACLE_A_DROITE;  
//
//        else if ( robotState.distanceTelemetreGauche2 > gauche2 && //cas n° 11
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 < droit2) 
//        obstacle = OBSTACLE_SUR_LA_DROITE;  
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 12
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit > droit &&
//        robotState.distanceTelemetreDroit2 > droit2) 
//        obstacle = OBSTACLE_SUR_LA_GAUCHE;
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 13
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 < droit2 ) 
//        obstacle = OBSTACLE_CUL_DE_SAC;
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 14
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 > droit2 ) 
//        obstacle = OBSTACLE_SUR_LA_GAUCHE; 
//
//        else if ( robotState.distanceTelemetreGauche2 > gauche2 && //cas n° 15
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 < droit2 ) 
//        obstacle = OBSTACLE_SUR_LA_DROITE; 
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 16
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 < droit2 ) 
//        obstacle = OBSTACLE_CUL_DE_SAC;
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 17
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit > droit &&
//        robotState.distanceTelemetreDroit2 < droit2 ) 
//        obstacle = OBSTACLE_CUL_DE_SAC;
//
//        else if ( robotState.distanceTelemetreGauche2 > gauche2 && //cas n° 18
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit > droit &&
//        robotState.distanceTelemetreDroit2 < droit2) 
//        obstacle = OBSTACLE_SUR_LA_DROITE;
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 19
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit > droit &&
//        robotState.distanceTelemetreDroit2 > droit2) 
//        obstacle = OBSTACLE_SUR_LA_GAUCHE; 
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 20
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit > droit &&
//        robotState.distanceTelemetreDroit2 < droit2) 
//        obstacle = OBSTACLE_CUL_DE_SAC;
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 21
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre > centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 > droit2) 
//        obstacle = OBSTACLE_SUR_LA_GAUCHE; 
//
//        else if ( robotState.distanceTelemetreGauche2 > gauche2 && //cas n° 22
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre > centre &&
//        robotState.distanceTelemetreDroit > droit &&
//        robotState.distanceTelemetreDroit2 > droit2) 
//        obstacle = OBSTACLE_SUR_LA_DROITE;
//
//        else if ( robotState.distanceTelemetreGauche2 > gauche2 && //cas n° 23
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre > centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 < droit2) 
//        obstacle = OBSTACLE_SUR_LA_DROITE;
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 24
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre > centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 > droit2) 
//        obstacle = OBSTACLE_SUR_LA_GAUCHE; 
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 25
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre > centre &&
//        robotState.distanceTelemetreDroit > droit &&
//        robotState.distanceTelemetreDroit2 < droit2) 
//        obstacle = OBSTACLE_CUL_DE_SAC; 
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 26
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre > centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 < droit2) 
//        obstacle = OBSTACLE_CUL_DE_SAC; 
//
//        else if ( robotState.distanceTelemetreGauche2 > gauche2 && //cas n° 27
//        robotState.distanceTelemetreGauche < gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit > droit &&
//        robotState.distanceTelemetreDroit2 < droit2) 
//        obstacle = OBSTACLE_SUR_LA_DROITE;
//
//        else if ( robotState.distanceTelemetreGauche2 < gauche2 && //cas n° 28
//        robotState.distanceTelemetreGauche > gauche &&
//        robotState.distanceTelemetreCentre < centre &&
//        robotState.distanceTelemetreDroit < droit &&
//        robotState.distanceTelemetreDroit2 > droit2) 
//        obstacle = OBSTACLE_SUR_LA_GAUCHE; 
        
        return obstacle;
}