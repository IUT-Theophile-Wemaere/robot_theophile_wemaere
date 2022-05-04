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
            serialPort1 = new ReliableSerialPort("COM3", 460800, Parity.None, 8, StopBits.One);
            serialPort1.DataReceived += SerialPort1_DataReceived;
            serialPort1.Open();
        }


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

        List<ArucoLutElement> ArucoLut = new List<ArucoLutElement>();


        private void Analyze_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog opfd = new OpenFileDialog();
            if (opfd.ShowDialog() == true)
            {
                ArucoLut = new List<ArucoLutElement>();
                using (var reader = new StreamReader(opfd.FileName))
                {
                    //List<int> ThetaAruco = new List<int>();
                    //List<int> x1List = new List<int>();
                    //List<int> x2List = new List<int>();
                    //List<int> x3List = new List<int>();
                    //List<int> x4List = new List<int>();
                    //List<int> y1List = new List<int>();
                    //List<int> y2List = new List<int>();
                    //List<int> y3List = new List<int>();
                    //List<int> y4List = new List<int>();

                    while (!reader.EndOfStream)
                    {
                        var line = reader.ReadLine();
                        var values = line.Split(';');
                        int pos = 0;

                        ArucoLutElement lutElement = new ArucoLutElement();
                        lutElement.xField = int.Parse(values[pos++]);
                        lutElement.yField = int.Parse(values[pos++]);
                        lutElement.thetaField = int.Parse(values[pos++]);

                        double x = int.Parse(values[pos++]);
                        double y = int.Parse(values[pos++]);
                        lutElement.pt1 = new PointD(x, y);
                        x = int.Parse(values[pos++]);
                        y = int.Parse(values[pos++]);
                        lutElement.pt2 = new PointD(x, y);
                        x = int.Parse(values[pos++]);
                        y = int.Parse(values[pos++]);
                        lutElement.pt3 = new PointD(x, y);
                        x = int.Parse(values[pos++]);
                        y = int.Parse(values[pos++]);
                        lutElement.pt4 = new PointD(x, y);

                        lutElement.xMeasured = (lutElement.pt1.X + lutElement.pt2.X + lutElement.pt3.X + lutElement.pt4.X) / 4.0;
                        lutElement.yMeasured = (lutElement.pt1.Y + lutElement.pt2.Y + lutElement.pt3.Y + lutElement.pt4.Y) / 4.0;

                        ArucoLut.Add(lutElement);

                        //Génération du symétrique par rapport à l'axe vertical

                        if (lutElement.xField != 0)
                        {
                            ArucoLutElement lutElementSym = new ArucoLutElement();

                            lutElementSym.xField = -lutElement.xField;
                            lutElementSym.yField = lutElement.yField;
                            lutElementSym.xMeasured = -lutElement.xMeasured;
                            lutElementSym.yMeasured = lutElement.yMeasured;
                            lutElementSym.pt1 = new PointD(-lutElement.pt1.X, lutElement.pt1.Y);
                            lutElementSym.pt2 = new PointD(-lutElement.pt2.X, lutElement.pt2.Y);
                            lutElementSym.pt3 = new PointD(-lutElement.pt3.X, lutElement.pt3.Y);
                            lutElementSym.pt4 = new PointD(-lutElement.pt4.X, lutElement.pt4.Y);

                            ArucoLut.Add(lutElementSym);
                        }

                    }
                }
            }
        }

        double xA, xB, xC, xD, yA, yB, yC, yD;
        string id;

        string[] BaliseArucoViolette = {"51", "52", "53"};



        private void AnalyzeData(string[] inputArray)
        {
            for(int i =0; i< inputArray.Length;i++)
            {
                inputArray[i] =  inputArray[i].Replace('.', ',');
            }
            if(ArucoLut.Count > 0)
            {
                id = inputArray[1]; 
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
                    Console.WriteLine("Distance calculée à partir de la balise" + id + " : " );
                }

                
            }

        }

        private void button_Click(object sender, RoutedEventArgs e)
        {

            string path = "C:\\Users\\Table 12\\Documents\\GitHub\\robot_theophile_wemaere\\JeVoisDecoder\\Data\\log.csv";

            string Xreel = tbXreel.Text;
            string Yreel = tbYreel.Text;
            string Theta = tbTheta.Text;

            tbXreel.Text = ""; tbYreel.Text = ""; tbTheta.Text = "";

            string log = Xreel + ";" + Yreel + ";" + Theta + ";" + xA + ";" + yA + ";" + xB + ";" + yB + ";" + xC + ";" + yC + ";" + xD + ";" + yD;

            using (StreamWriter file = new StreamWriter(path, append: true))
            {
                file.WriteLine(log);
            }
        }
    }
     
    public class ArucoLutElement
    {
        public double xField;
        public double yField;
        public double xMeasured;
        public double yMeasured;
        public double thetaField;
        public double segAB;
        public double segBC;
        public double segCD;
        public double segDA;
        public PointD pt1;
        public PointD pt2;
        public PointD pt3;
        public PointD pt4;
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
