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
using MouseKeyboardActivityMonitor.WinApi;
using MouseKeyboardActivityMonitor;
using System.Windows.Forms;
using Utilities;
using WpfAsservissementDisplay;

namespace Interfacerobot
{
    public partial class MainWindow : Window
    {

        ReliableSerialPort serialPort1;
        DispatcherTimer timerAffichage;
        Robot robot = new Robot();
        //AsservissementXYThetaControl asservissement = new AsservissementXYThetaControl();
        private readonly KeyboardHookListener m_KeyboardHookManager;

        int i;
        public MainWindow()
        {
            InitializeComponent();
            serialPort1 = new ReliableSerialPort("COM7", 115200, Parity.None, 8, StopBits.One);
            serialPort1.DataReceived += SerialPort1_DataReceived;
            serialPort1.Open();

            timerAffichage = new DispatcherTimer();
            timerAffichage.Interval = new TimeSpan(0, 0, 0, 0, 100);
            timerAffichage.Tick += TimerAffichage_Tick;
            timerAffichage.Start();
            m_KeyboardHookManager = new KeyboardHookListener(new GlobalHooker());
            m_KeyboardHookManager.Enabled = true;
            m_KeyboardHookManager.KeyDown += HookManager_KeyDown;

        }



        private void TimerAffichage_Tick(object sender, EventArgs e)
        {
            byte[] message = Encoding.ASCII.GetBytes("Bonjour");
            /*TextBoxReception.Text += robot.ReceivedText;
            robot.ReceivedText = "";*/
            while (robot.byteListReceived.Count > 0)
            {
                DecodeMessage(robot.byteListReceived.Dequeue());
                //TextBoxReception.Text += robot.byteListReceived.Dequeue();
                //byte byteReceived = robot.byteListReceived.Dequeue();
                //string receivedText = "0x" + byteReceived.ToString("X2") + " " ;
                //char receivedText = Convert.ToChar(byteReceived);
                // TextBoxReception.Text += receivedText;
            }

        }

        private void SerialPort1_DataReceived(object sender, DataReceivedArgs e)
        {
            //robot.ReceivedText += Encoding.UTF8.GetString(e.Data, 0, e.Data.Length);
            for (int i = 0; i < e.Data.Length; i++)
            {
                DecodeMessage(e.Data[i]);
                //robot.byteListReceived.Enqueue(e.Data[i]);
            }
        }


        #region Encodage/Decodage

        StateReception rcvState = StateReception.Waiting;
        int msgDecodedFunction = 0;
        int msgDecodedPayloadLength = 0;
        byte[] msgDecodedPayload;
        int msgDecodedPayloadIndex = 0;

        public enum StateReception
        {
            Waiting,
            FunctionMSB,
            FunctionLSB,
            PayloadLengthMSB,
            PayloadLengthLSB,
            Payload,
            CheckSum
        }

        public enum Functions
        {
            LEDS = 0x0020,
            telemetres = 0x0030,
            moteurs = 0x0040,
            message = 0x0080,
            RobotState=0x0050,
            Clavier=0x0053,
            Position=0x0061,
            PID=0x0062
        }

        public enum StateRobot
        {
            STATE_ATTENTE=0,
            STATE_ATTENTE_EN_COURS=1,
            STATE_AVANCE=2,
            STATE_AVANCE_EN_COURS=3,
            STATE_TOURNE_GAUCHE=4,
            STATE_TOURNE_GAUCHE_EN_COURS=5,
            STATE_TOURNE_DROITE=6,
            STATE_TOURNE_DROITE_EN_COURS=7,
            STATE_TOURNE_SUR_PLACE_GAUCHE=8,
            STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS=9,
            STATE_TOURNE_SUR_PLACE_DROITE=10,
            STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS=11,
            STATE_ARRET=12,
            STATE_ARRET_EN_COURS=13,
            STATE_RECULE=14,
            STATE_RECULE_EN_COURS=15,
            STATE_TOURNE_GAUCHE_DOUX =16,
            STATE_TOURNE_GAUCHE_DOUX_EN_COURS= 17,
            STATE_TOURNE_DROITE_DOUX=19,
            STATE_TOURNE_DROITE_DOUX_EN_COURS=18 ,
            STATE_DROITE_TUNNEL =20,
            STATE_DROITE_TUNNEL_EN_COURS= 21,
            STATE_GAUCHE_TUNNEL= 22,
            STATE_GAUCHE_TUNNEL_EN_COURS= 23,
        }

        byte CalculateChecksum(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte checksum = 0;
            checksum ^= 0xFE;
            checksum ^= (byte)(msgFunction >> 8);
            checksum ^= (byte)(msgFunction >> 0);
            checksum ^= (byte)(msgPayloadLength >> 8);
            checksum ^= (byte)(msgPayloadLength >> 0);
            for (int i = 0; i < msgPayload.Length; i++)
            {
                checksum ^= msgPayload[i];
            }
            return checksum;
        }

        void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte[] trame = new byte[6 + msgPayload.Length];
            int pos = 0;
            int i;
            trame[pos++] = 0xFE;
            trame[pos++] = (byte)(msgFunction >> 8);
            trame[pos++] = (byte)(msgFunction >> 0);
            trame[pos++] = (byte)(msgPayloadLength >> 8);
            trame[pos++] = (byte)(msgPayloadLength >> 0);
            for (i = 0; i < msgPayload.Length; i++)
            {
                trame[pos++] = msgPayload[i];
            }
            trame[pos++] = CalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
            serialPort1.Write(trame, 0, pos);
        }

        private void DecodeMessage(byte c)
        {
            switch (rcvState)
            {
                case StateReception.Waiting:
                    if (c == 0xFE)
                    {
                        rcvState = StateReception.FunctionMSB;
                    }
                    msgDecodedFunction = 0;
                    msgDecodedPayloadIndex = 0;
                    msgDecodedPayloadLength = 0;
                    break;

                case StateReception.FunctionMSB:
                    msgDecodedFunction = (byte)(c << 8);
                    rcvState = StateReception.FunctionLSB;
                    break;

                case StateReception.FunctionLSB:
                    msgDecodedFunction += (byte)(c << 0);
                    rcvState = StateReception.PayloadLengthMSB;
                    break;

                case StateReception.PayloadLengthMSB:
                    msgDecodedPayloadLength = (byte)(c << 8);
                    rcvState = StateReception.PayloadLengthLSB;
                    break;

                case StateReception.PayloadLengthLSB:
                    msgDecodedPayloadLength += (byte)(c << 0);
                    if (msgDecodedPayloadLength != 0)
                    {
                        msgDecodedPayload = new byte[msgDecodedPayloadLength];
                        msgDecodedPayloadIndex = 0;
                        rcvState = StateReception.Payload;
                    }
                    else
                    {
                        rcvState = StateReception.CheckSum;
                    }
                    break;

                case StateReception.Payload:
                    msgDecodedPayload[msgDecodedPayloadIndex] = c;
                    msgDecodedPayloadIndex++;
                    if (msgDecodedPayloadIndex >= msgDecodedPayloadLength)
                    {
                        rcvState = StateReception.CheckSum;
                    }
                    break;

                case StateReception.CheckSum:
                    byte calculatedChecksum, receivedChecksum;
                    receivedChecksum = c;
                    calculatedChecksum = CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                    if (calculatedChecksum == receivedChecksum)
                    {
                        Dispatcher.Invoke(
                            delegate
                            {
                                ProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                            });
                    }
                    else
                    {
                        Dispatcher.Invoke(
                            delegate {TextBoxReception.Text += "\n\r/!\\ Attention, message corrompu. fonctions: " + "0x" + msgDecodedFunction.ToString("X4");}
                            );
                    }
                    rcvState = StateReception.Waiting;
                    break;

                default:
                    rcvState = StateReception.Waiting;
                    break;
            }
        }

        void ProcessDecodedMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            int numLed, etatLed, IRGauche, IRCentre, IRDroite, MG, MD;

            switch((Functions)msgFunction)
            {
                case Functions.LEDS:
                    numLed = msgPayload[0];
                    etatLed = msgPayload[1];
                    if (numLed == 1)
                    {
                        if (etatLed == 1)
                        {
                            CheckBoxLed1.IsChecked = true;
                        }
                        else
                        {
                            CheckBoxLed1.IsChecked = false;
                        }
                    }
                    else if (numLed == 2)
                    {
                        if (etatLed == 1)
                        {
                            CheckBoxLed2.IsChecked = true;
                        }
                        else
                        {
                            CheckBoxLed2.IsChecked = false;
                        }
                    }
                    else if (numLed == 3)
                    {
                        if (etatLed == 1)
                        {
                            CheckBoxLed3.IsChecked = true;
                        }
                        else
                        {
                            CheckBoxLed3.IsChecked = false;
                        }
                    }
                break;

                case Functions.telemetres:
                    IRDroite = msgPayload[0];
                    IRCentre = msgPayload[1];
                    IRGauche = msgPayload[2];
                    TextBoxTelemetres.Text = "IR Gauche:" + IRGauche.ToString() + "cm\n\r" + "IR Centre:" + IRCentre.ToString() + "cm\n\r" + "IR Droite:" + IRDroite.ToString() + "cm";
                break;

                case Functions.moteurs:
                    MG = msgPayload[0];
                    MD = msgPayload[1];
                    TextBoxMoteurs.Text = "Vitesse Gauche:" + MG.ToString() + "%\n\r" + "Vitesse Droite:" + MD.ToString() + "%\n\r";
                break;

                case Functions.message:
                    TextBoxReception.Text += Encoding.UTF8.GetString(msgPayload, 0, msgPayload.Length) + "\n";
                break;

                case Functions.RobotState:
                    int instant = (((int)msgPayload[1]) << 24) + (((int)msgPayload[2]) << 16) + (((int)msgPayload[3]) << 8) + (((int)msgPayload[4]));
                    RtbReception.Text = ((StateRobot)(msgPayload[0])).ToString() + "\n\rTemps: " + instant.ToString() + " ms";
                break;

                case Functions.Clavier:
                    textBoxPilotage.Text = "Instruction reçue: " + ((StateRobot)msgPayload[0]).ToString() ;
                break;

                case Functions.Position:
                    byte[] tab = msgPayload.GetRange(4, 4);
                    robot.positionXOdo = tab.GetFloat();
                    tab = msgPayload.GetRange(8, 4);
                    robot.positionYOdo = tab.GetFloat();
                    tab = msgPayload.GetRange(12, 4);
                    robot.AngleRadOdo = tab.GetFloat();
                    tab = msgPayload.GetRange(16, 4);
                    robot.vLinéaireOdo = tab.GetFloat();
                    tab = msgPayload.GetRange(20, 4);
                    robot.vAngulaireOdo = tab.GetFloat();
                    textBoxPosition.Text = "Pos X: " + (robot.positionXOdo).ToString("F4") + "\n\r";
                    textBoxPosition.Text += "Pos Y: " + (robot.positionYOdo).ToString("F4") + "\n\r";
                    textBoxPosition.Text += "Angle en ° : " + (robot.AngleRadOdo*(180d/Math.PI)).ToString("F4") + "\n\r";
                    textBoxPosition.Text += "Vitesse linéaire en m.s-1 : " + robot.vLinéaireOdo.ToString() + "\n\r";
                    textBoxPosition.Text += "Vitesse Angulaire en rad.s-1 : " + robot.vAngulaireOdo.ToString();
                    
                    break;

                case Functions.PID:
                    double vAngError;
                    double vAngCommand;
                    double KpAng;
                    double CorrectKpAng;
                    double KiAng;
                    double CorrectKiAng;
                    double KdAng;
                    double CorrectKdAng;
                    double vAngCons;
                    double CorrectKpAngMax;
                    double CorrectKiAngMax;
                    double CorrectKdAngMax;

                    double vLinError;
                    double vLinCommand;
                    double KpLin;
                    double CorrectKpLin;
                    double KiLin;
                    double CorrectKiLin;
                    double KdLin;
                    double CorrectKdLin;
                    double vLinCons;
                    double CorrectKpLinMax;
                    double CorrectKiLinMax;
                    double CorrectKdLinMax;


                    byte[] tabPID = msgPayload.GetRange(0, 4);
                    vAngError = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(4, 4);
                    vAngCommand = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(8, 4);
                    KpAng = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(12, 4);
                    CorrectKpAng = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(16, 4);
                    KiAng = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(20, 4);
                    CorrectKiAng = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(24, 4);
                    KdAng = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(28, 4);
                    CorrectKdAng = tabPID.GetFloat();

                    tabPID = msgPayload.GetRange(32, 4);
                    vLinError = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(36, 4);
                    vLinCommand = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(40, 4);
                    KpLin = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(44, 4);
                    CorrectKpLin = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(48, 4);
                    KiLin = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(52, 4);
                    CorrectKiLin = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(56, 4);
                    KdLin = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(60, 4);
                    CorrectKdLin = tabPID.GetFloat();

                    tabPID = msgPayload.GetRange(64, 4);
                    vAngCons = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(68, 4);
                    vLinCons = tabPID.GetFloat();

                    tabPID = msgPayload.GetRange(72, 4);
                    CorrectKpAngMax = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(76, 4);
                    CorrectKiAngMax = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(80, 4);
                    CorrectKdAngMax = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(84, 4);
                    CorrectKpLinMax = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(88, 4);
                    CorrectKiLinMax = tabPID.GetFloat();
                    tabPID = msgPayload.GetRange(92, 4);
                    CorrectKdLinMax = tabPID.GetFloat();

                    AsservDisplay.UpdatePolarSpeedConsigneValues(vLinCons, vAngCons);
                    AsservDisplay.UpdatePolarSpeedCommandValues(vLinCommand, vAngCommand);
                    AsservDisplay.UpdatePolarOdometrySpeed(robot.vLinéaireOdo, robot.vAngulaireOdo);
                    AsservDisplay.UpdatePolarSpeedErrorValues(vLinError, vAngError);
                    AsservDisplay.UpdatePolarSpeedCorrectionGains(KpLin, KpAng, KiLin, KiAng, KdLin, KdAng);
                    AsservDisplay.UpdatePolarSpeedCorrectionValues(CorrectKpLin, CorrectKpAng, CorrectKiLin, CorrectKiAng, CorrectKdLin, CorrectKdAng);
                    AsservDisplay.UpdatePolarSpeedCorrectionLimits(CorrectKpLinMax, CorrectKiLinMax, CorrectKdLinMax, CorrectKpAngMax, CorrectKiAngMax, CorrectKdAngMax);

                    AsservDisplay.UpdateDisplay();
                    break;
            }
        }

        bool autoControlActivated = false;
        private void HookManager_KeyDown(object sender, System.Windows.Forms.KeyEventArgs e)
        {
            var imageBrush = new ImageBrush();
            if (autoControlActivated == true)
            {
                switch (e.KeyCode)
                {
                    //case Keys.Left:
                    //    UartEncodeAndSendMessage(0x0051, 1, new byte[] { (byte)StateRobot.STATE_TOURNE_SUR_PLACE_GAUCHE });
                    //    //TextBoxReception.Text = "Left\n\r";
                    //    imageBrush.ImageSource = new BitmapImage(new Uri("keyboard_key_left.png", UriKind.Relative));
                    //    BoxClavier.Background = imageBrush;
                    //    break;

                    //case Keys.Right:
                    //    UartEncodeAndSendMessage(0x0051, 1, new byte[] { (byte)StateRobot.STATE_TOURNE_SUR_PLACE_DROITE });
                    //    //TextBoxReception.Text = "Right\n\r";
                    //    imageBrush.ImageSource = new BitmapImage(new Uri("keyboard_key_right.png", UriKind.Relative));
                    //    BoxClavier.Background = imageBrush;
                    //    break;

                    //case Keys.Up:
                    //    UartEncodeAndSendMessage(0x0051, 1, new byte[] { (byte)StateRobot.STATE_AVANCE });
                    //    //TextBoxReception.Text = "up\n\r";
                    //    imageBrush.ImageSource = new BitmapImage(new Uri("keyboard_key_up.png", UriKind.Relative));
                    //    BoxClavier.Background = imageBrush;
                    //    break;

                    //case Keys.Down:
                    //    UartEncodeAndSendMessage(0x0051, 1, new byte[] { (byte)StateRobot.STATE_RECULE });
                    //    //TextBoxReception.Text = "down\n\r";
                    //    imageBrush.ImageSource = new BitmapImage(new Uri("keyboard_key_down.png", UriKind.Relative));
                    //    BoxClavier.Background = imageBrush;
                    //    break;

                    //case Keys.PageDown:
                    //    UartEncodeAndSendMessage(0x0051, 1, new byte[] { (byte)StateRobot.STATE_ARRET });
                    //    //TextBoxReception.Text = "Pagedown\n\r";
                    //    imageBrush.ImageSource = new BitmapImage(new Uri("keyboard_key_page_down.png", UriKind.Relative));
                    //    BoxClavier.Background = imageBrush;
                    //    break;

                    
                    case Keys.Up:
                        UartEncodeAndSendMessage(0x0053, 2, new byte[] { Convert.ToByte(0.3), 0 });
                        break;

                    case Keys.Down:
                        UartEncodeAndSendMessage(0x0053, 2, new byte[] { Convert.ToByte(-0.3), 0 });
                        break;

                    case Keys.Left:
                        UartEncodeAndSendMessage(0x0053, 2, new byte[] { Convert.ToByte(0.2), Convert.ToByte(0.2) });
                        break;

                    case Keys.Right:
                        UartEncodeAndSendMessage(0x0053, 2, new byte[] { Convert.ToByte(0.2), Convert.ToByte(-0.2) });
                        break;
                }
            }
            }

        #endregion

        #region Boutons 
        private void textBox_TextChanged(object sender, TextChangedEventArgs e) 
        {

        }

        private void buttonEnvoyer_Click(object sender, RoutedEventArgs e)
        {
            robot.ReceivedText = null;
            TextBoxEmission.Text = "";
            TextBoxReception.Text = "";

        }

        private void sendMessage()
        {
            
            //TextBoxReception.Text = TextBoxReception.Text + "Message reçu: " + TextBoxEmission.Text + "\r\n";
            //TextBoxEmission.Text = "";
            serialPort1.Write(TextBoxEmission.Text);
            TextBoxEmission.Text = "";
        }

        private void buttonTest_Click(object sender, RoutedEventArgs e)
        {
            byte[] message = Encoding.ASCII.GetBytes("Bonjour");

           // ProcessDecodedMessage(0x0020, 2, new byte[] { 3,1 });
           UartEncodeAndSendMessage(0x0020, 2, new byte[] { 3, 1 });   //led 3 true
           UartEncodeAndSendMessage(0x0030, 3, new byte[] { 25, 30, 25 });  //IR 25cm 30cm 25cm
           UartEncodeAndSendMessage(0x0040, 2, new byte[] { 41, 38 });  // Moteur1 41% Moteur2 38%
           //UartEncodeAndSendMessage(0x0080, 7, message);
           UartEncodeAndSendMessage(0x0020, 2, new byte[] { 1, 1 });   //led 1 true
           UartEncodeAndSendMessage(0x0020, 2, new byte[] { 2, 1 });   //led 2 true        
           // UartEncodeAndSendMessage(0x0080, (UInt16)message.Length, message);
            UartEncodeAndSendMessage(0x0053, 1, new byte[] { 4 });
            UartEncodeAndSendMessage(0x0050, 5, new byte[] { 2, 0, 0, 0, 10 });
            UartEncodeAndSendMessage(0x0061, 5, new byte[] { 0, 0, 0, 0, 0 });
        }

        private void TextBoxEmission_KeyUp(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                //sendMessage();
                string message = TextBoxEmission.Text;
                byte[] array = Encoding.ASCII.GetBytes(message);
                UartEncodeAndSendMessage(0x0080, (UInt16)array.Length, array);
                TextBoxEmission.Text = "";
            }
        }

        bool cpt = true;
        private void buttonControl_Click(object sender, RoutedEventArgs e)
        {
            
            if (!cpt)
            {
                buttonControl.Background = Brushes.White;
                buttonControl.Content = "Automatique";
                autoControlActivated = false;
                cpt = true;
                BoxClavier.Visibility = Visibility.Hidden;
                BoxClavier.Background = Brushes.Black;
                UartEncodeAndSendMessage(0x0052, 1, new byte[] {1});
            }
            else
            {
                buttonControl.Background = Brushes.Red;
                buttonControl.Content = "Manuel";
                autoControlActivated = true;
                cpt = false;
                BoxClavier.Visibility = Visibility.Visible;
                UartEncodeAndSendMessage(0x0052, 1, new byte[] {0});
            }
        }
        private void buttonReset_Click(object sender, RoutedEventArgs e)
        {
            UartEncodeAndSendMessage(0x0062,1, new byte[] {0});
        }

        #endregion

    }
}

