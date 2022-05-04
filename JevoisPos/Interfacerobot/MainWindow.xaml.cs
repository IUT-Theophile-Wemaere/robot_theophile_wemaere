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
using System.IO;
using Microsoft.Win32;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics;

//  A       B
//   _______
//  |       |
//  |       |
//  |_______|
//  D       C




namespace Interfacerobot
{
    public partial class MainWindow : System.Windows.Window
    {

        ReliableSerialPort serialPort1;

        public MainWindow()
        {
            serialPort1 = new ReliableSerialPort("COM7", 460800, Parity.None, 8, StopBits.One);
            serialPort1.DataReceived += SerialPort1_DataReceived;
            serialPort1.Open();
        }

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
            if (c == 'D')
            {
                AnalyzeJeVoisData(jeVoisCurrentFrame);
                jeVoisCurrentFrame = "";
            }
            jeVoisCurrentFrame += Encoding.UTF8.GetString(new byte[] { c }, 0, 1);
        }

        void AnalyzeJeVoisData(string s)
        {
            string[] jeVoisArray = s.Split(' ');
            if (jeVoisArray.Length == 11) //size of the JeVois input payload
            {
                AnalyzeData(jeVoisArray);
            }
        }

        //liste contenant toutes les valeurs théoriques de la position / taille des tags aruco -> lookup table (lut)

        List<ArucoLutElement> ArucoLut = new List<ArucoLutElement>();

        private void Analyze_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog opfd = new OpenFileDialog();
            if (opfd.ShowDialog() == true)
            {
                ArucoLut = new List<ArucoLutElement>();
                using (var reader = new StreamReader(opfd.FileName))
                {

                    while (!reader.EndOfStream)
                    {
                        var line = reader.ReadLine();
                        var values = line.Split(';');
                        int pos = 0;

                        Console.WriteLine(values[0]);

                        ArucoLutElement lutElement = new ArucoLutElement();
                        
                        lutElement.dist0M = float.Parse(values[pos++], CultureInfo.InvariantCulture.NumberFormat);

                        Console.WriteLine(lutElement.dist0M);

                        double xA = float.Parse(values[pos++]);
                        double yA = int.Parse(values[pos++]);
                        double xB = int.Parse(values[pos++]);
                        double yB = int.Parse(values[pos++]);
                        double xC = int.Parse(values[pos++]);
                        double yC = int.Parse(values[pos++]);
                        double xD = int.Parse(values[pos++]);
                        double yD = int.Parse(values[pos++]);

                        lutElement.segAB = Math.Sqrt(Math.Pow((xA - xB), 2) + Math.Pow((yA - yB), 2));
                        lutElement.segBC = Math.Sqrt(Math.Pow((xB - xC), 2) + Math.Pow((yB - yC), 2));
                        lutElement.segCD = Math.Sqrt(Math.Pow((xC - xD), 2) + Math.Pow((yC - yD), 2));
                        lutElement.segDA = Math.Sqrt(Math.Pow((xD - xA), 2) + Math.Pow((yD - yA), 2));

                        ArucoLut.Add(lutElement);

                        //Génération du symétrique par rapport à l'axe vertical
                        //pas necessaire ?
                        /*if (xField != 0)
                        {
                            ArucoLutElement lutElementSym = new ArucoLutElement();

                            lutElementSym.ptA = new PointD(-lutElement.ptA.X, lutElement.ptA.Y);
                            lutElementSym.ptB = new PointD(-lutElement.ptB.X, lutElement.ptB.Y);
                            lutElementSym.ptC = new PointD(-lutElement.ptC.X, lutElement.ptC.Y);
                            lutElementSym.ptD = new PointD(-lutElement.ptD.X, lutElement.ptD.Y);
                            ArucoLut.Add(lutElementSym);
                        }*/

                    }
                }
            }
        }

        double xA, xB, xC, xD, yA, yB, yC, yD;
        string id;

        string[] BaliseArucoViolette = {"51", "52", "54"};
        string[] BaliseArucoJaune = { "71", "72", "73" };



        private void AnalyzeData(string[] inputArray)
        {
            for(int i = 0; i< inputArray.Length;i++)
            {
                inputArray[i] =  inputArray[i].Replace('.', ',');
            }

            if(ArucoLut.Count > 0)
            {
                id = inputArray[1].Substring(1); 
                xA = double.Parse(inputArray[3]);
                yA = double.Parse(inputArray[4]);
                xB = double.Parse(inputArray[5]);
                yB = double.Parse(inputArray[6]);
                xC = double.Parse(inputArray[7]);
                yC = double.Parse(inputArray[8]);
                xD = double.Parse(inputArray[9]);
                yD = double.Parse(inputArray[10]);
                
                if( BaliseArucoViolette.Contains(id) )
                {
                    double segAB = Math.Sqrt(Math.Pow((xA - xB), 2) + Math.Pow((yA - yB), 2));
                    double segBC = Math.Sqrt(Math.Pow((xB - xC), 2) + Math.Pow((yB - yC), 2));
                    double segCD = Math.Sqrt(Math.Pow((xC - xD), 2) + Math.Pow((yC - yD), 2));
                    double segDA = Math.Sqrt(Math.Pow((xD - xA), 2) + Math.Pow((yD - yA), 2));

                    /// On calcule la distance à chaque element de la LUT
                    var ArucoClosestList = ArucoLut.OrderBy(p => Math.Abs(segAB - p.segAB)).ToList();

                    var closest_value = ArucoClosestList[0];
                    //Console.WriteLine( segAB + " | " + closest_value.segAB );
                    Console.WriteLine("Distance calculée à partir de la balise n° " + id + " : " + closest_value.dist0M.ToString("F1") );
                }

                
            }

        }

        private void button_Click(object sender, RoutedEventArgs e)
        {

            string path = "C:\\GitHub\\robot_theophile_wemaere\\JeVoisPos\\Data\\log.csv";

            string distance = tbXreel.Text;

            tbXreel.Text = ""; tbYreel.Text = ""; tbTheta.Text = "";

            string log = distance + ";" + (int)xA + ";" + (int)yA + ";" + (int)xB + ";" + (int)yB + ";" + (int)xC + ";" + (int)yC + ";" + (int)xD + ";" + (int)yD;

            using (StreamWriter file = new StreamWriter(path, append: true))
            {
                file.WriteLine(log);
            }
        }
    }
     
    public class ArucoLutElement
    {
        public double dist0M;    
        public double segAB;
        public double segBC;
        public double segCD;
        public double segDA;
    }

    public class PointD
    {
        public double X;
        public double Y;
        public PointD(double x, double y)
        {
            X = x;
            Y = y;
        }
    }

    public static class AlgebraTools
    {
        public static double? Cross(Vector<double> left, Vector<double> right)
        {
            double? result = null;
            if ((left.Count == 2 && right.Count == 2))
            {                
                result = left[0] * right[1] - left[1] * right[0];
            }

            return result;
        }
    }
}
