using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;   
using System.Windows.Navigation;
using System.Windows.Shapes;
using ExtendedSerialPort;
using System.Windows.Threading;
using System.Globalization;
using System.Collections.Concurrent;
using System.Timers;

namespace Interfacerobot
{
    public partial class MainWindow : Window
    {

        ReliableSerialPort serialPort1;

        public MainWindow()
        {
            serialPort1 = new ReliableSerialPort("COM3", 460800, Parity.None, 8, StopBits.One);
            serialPort1.DataReceived += SerialPort1_DataReceived;
            serialPort1.Open();
        }
       
        string jevoisData,jevoisString;


        ConcurrentQueue<byte> receivedData = new ConcurrentQueue<byte>();
        private void SerialPort1_DataReceived(object sender, DataReceivedArgs e)
        {
            foreach (var c in e.Data)
            {
                ProcessJeVoisData(c);
            }
        }

        string jeVoisCurrentFrame = "";
        void ProcessJeVoisData(byte c)
        {
            if(c=='D')
            {
                AnalyzeJeVoisData(jeVoisCurrentFrame);
                jeVoisCurrentFrame = "";            
            }
            jeVoisCurrentFrame += Encoding.UTF8.GetString(new byte[]{c}, 0, 1); 
        }

        void AnalyzeJeVoisData(string s)
        {
            string[] jeVoisArray = s.Split(' ');
            if (jeVoisArray.Length == 12) //size of the JeVois input payload
            {
                PrintJeVoisData(jeVoisArray);
            }
        }

        private void PrintJeVoisData(string[] inputArray)
        {
            string Dim, ID, result;
            float x, y, z, w, h, d, q1, q2, q3, q4;
            Dim=inputArray[0];
            ID=inputArray[1].Substring(1);
            x = float.Parse(inputArray[2], CultureInfo.InvariantCulture.NumberFormat); 
            y = float.Parse(inputArray[3], CultureInfo.InvariantCulture.NumberFormat);
            z = float.Parse(inputArray[4], CultureInfo.InvariantCulture.NumberFormat);
            w = float.Parse(inputArray[5], CultureInfo.InvariantCulture.NumberFormat);
            h = float.Parse(inputArray[6], CultureInfo.InvariantCulture.NumberFormat);
            d = float.Parse(inputArray[7], CultureInfo.InvariantCulture.NumberFormat);
            q1 = float.Parse(inputArray[8], CultureInfo.InvariantCulture.NumberFormat);
            q2 = float.Parse(inputArray[9], CultureInfo.InvariantCulture.NumberFormat);
            q3 = float.Parse(inputArray[10], CultureInfo.InvariantCulture.NumberFormat);
            q4 = float.Parse(inputArray[11], CultureInfo.InvariantCulture.NumberFormat);
            result = String.Format("Dim : {0} | ID : {1} | X = {2} | Y = {3} | Z = {4} | Width = {5} | Height = {6} | Depth = {7} | q1 = {8} | q2 = {9} | q3 = {10} | q4 = {11}", Dim,ID,x,y,z,w,h,d,q1,q2,q3,q4);
            Console.WriteLine(result);

            double qangle = 2 * Math.Acos(q1);
            double qx = q2 / Math.Sqrt(1 - q1 * q1);
            double qy = q3 / Math.Sqrt(1 - q1 * q1);
            double qz = q4 / Math.Sqrt(1 - q1 * q1);
            //result = String.Format(" q1 = {0} | q2 = {1} | q3 = {2} | q4 = {3}", qangle,qx,qy,qz);
            Console.WriteLine(Utilities.Toolbox.Modulo2PiAngleRad(CalculateQuaternions(q1,q2,q3,q4)));



            //double q = q1 + q2 + q3 + q4;
            //double norm = Math.Sqrt(Math.Pow(q1,2) + Math.Pow(q2,2) + Math.Pow(q3,2) + Math.Pow(q4,2));
            ////Console.WriteLine("q : " + q);
            ////Console.WriteLine("norm : " + norm);
            //double aX = q1 / Math.Sqrt(Math.Pow(q2, 2) + Math.Pow(q3, 2) + Math.Pow(q4, 2));
            //double aY = q2 / Math.Sqrt(Math.Pow(q2, 2) + Math.Pow(q3, 2) + Math.Pow(q4, 2));
            //double aZ = q3 / Math.Sqrt(Math.Pow(q2, 2) + Math.Pow(q3, 2) + Math.Pow(q4, 2));
            //Console.WriteLine("aX : " + aX + " aY : " + aY + " aZ : " + aZ);
            //double theta = Math.Atan2(Math.Sqrt(Math.Pow(q2, 2) + Math.Pow(q3, 2) + Math.Pow(q4, 2)), q1);
            //Console.WriteLine("angle : " + theta);

            //double angleCorrected = Utilities.Toolbox.Modulo2PiAngleRad(theta);
            //Console.WriteLine("angle corrected: " + angleCorrected);


        }

        private double CalculateQuaternions(double q1, double q2, double q3, double q4)
        {
            if (q1 > 1) q1 = q1/ Math.Sqrt(Math.Pow(q2, 2) + Math.Pow(q3, 2) + Math.Pow(q4, 2)); // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised

            double angle = 2 * Math.Acos(q1);

            double x, y, z;

            double s = Math.Sqrt(1 - q1 * q1); // assuming quaternion normalised then w is less than 1, so term always positive.
            if (s < 0.001)
            { // test to avoid divide by zero, s is always positive due to sqrt
              // if s close to zero then direction of axis not important
                x = q2; // if it is important that axis is normalised then replace with x=1; y=z=0;
                y = q3;
                z = q4;
            }
            else
            {
                x = q2 / s; // normalise axis
                y = q3 / s;
                z = q4 / s;
            }
            //string result = String.Format(" x = {0} | y = {1} | z = {2}}", x, y, z);
            //Console.WriteLine(result);
            return angle;
        }

    }
}