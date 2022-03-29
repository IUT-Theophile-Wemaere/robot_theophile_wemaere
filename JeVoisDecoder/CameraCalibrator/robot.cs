using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Interfacerobot
{
    public class Robot
    {
        public Queue<byte> byteListReceived = new Queue<byte>();
        public string ReceivedText = "";
        public float distanceTelemetreGauche2 ;
        public float distanceTelemetreGauche;
        public float distanceTelemetreCentre;
        public float distanceTelemetreDroit;
        public float distanceTelemetreDroit2;
        public float positionXOdo;
        public float positionYOdo;
        public float AngleRadOdo;
        public float vLinéaireOdo;
        public float vAngulaireOdo;

        public Robot()
        {

        }
    }
}
