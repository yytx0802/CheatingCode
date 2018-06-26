using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace lscm.driver.lidar.hokuyo.urg
{
    /// <summary>
    /// when send the lidar "SCIP2.0\n" will return "0Ee\r\n"
    /// 
    /// "SCIP2.0" --> ping command, return "0Ee\r\n"
    /// "VV" --> version information
    /// "II" --> detail started infomration
    /// "PP" --> current device setting, start angle, end angle...
    /// "QT" --> stop scan.
    /// "MD0044072501000" --> fully scan the lidar
    /// </summary>
    public class Urg04LX
    {

        private static int BUADRATE = 115200;
        private static int START = 44;
        private static int END = 725;
        #region properties
        private SerialPort urg = null;
        private Thread sThread = null;
        private bool startFlag = false;
        List<long> distances = new List<long>();
        #endregion

        public Urg04LX()
        {
            //urg = new SerialPort(comName, BUADRATE);
            //urg.NewLine = "\n\n";

            //urg.Open();

            //urg.Write(SCIP_Writer.SCIP2());
            //urg.ReadLine(); // ignore echo back
            //urg.Write(SCIP_Writer.MD(START, END));
            //urg.ReadLine(); // ignore echo back

            string comPortName = SearchComPort();
            Console.WriteLine("Lidar URG com port=" + comPortName);
            if (comPortName == null)
            {
                throw new Exception("No lidar found in the system.");
            }

            urg = new SerialPort(comPortName, BUADRATE)
            {
                NewLine = "\n\n"
            };
        }

        public Urg04LX(string port, int buadRate)
        {
            urg = new SerialPort(port, buadRate)
            {
                NewLine = "\n\n"
            };
        }

        private string SearchComPort()
        {
            string[] comNames = SerialPort.GetPortNames();
            if (comNames.Length == 0)
            {
                return null;
            }

            for (int i = 0; i < comNames.Length; i++)
            {
                try
                {
                    if (comNames[i].ToUpper().StartsWith("COM"))
                    {
                        using (var port = new SerialPort(comNames[i], BUADRATE))
                        {
                            port.NewLine = "\n\n";
                            port.WriteTimeout = 10;

                            port.Open();
                            port.Write(SCIP_Writer.SCIP2());
                            Thread.Sleep(50);
                            var bufferCount = port.BytesToRead;
                            if (bufferCount > 3)
                            {
                                string line = port.ReadExisting();
                                if (line.Contains("0Ee"))
                                {
                                    return comNames[i];
                                }
                            }

                            port.Close();
                        }
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }
            }
            return null;
        }

        public bool OpenConnection()
        {
            try
            {
                if (this.urg != null)
                {
                    if (!this.urg.IsOpen)
                    {
                        this.urg.Open();
                        return true;
                    }
                }
                return false;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex);
                return false;
            }
        }

        private void WorkerHandler()
        {
            urg.Write(SCIP_Writer.MD(START, END));
            urg.ReadLine();
            long time_stamp = 0;

            while (startFlag)
            {
                string receive_data = urg.ReadLine();
                if (!SCIP_Reader.MD(receive_data, ref time_stamp, ref distances))
                {
                    Console.WriteLine(receive_data);
                    break;
                }
                if (distances.Count == 0)
                {
                    Console.WriteLine(receive_data);
                    continue;
                }
                //TODO: process the data here


                // show distance data
                //for (int i = 0; i < 10; i++)
                //{
                //    //Console.WriteLine("time stamp: " + time_stamp.ToString() + " distance[384] : " + distances[384].ToString());
                //    Console.Write("{0} ", distances[i].ToString());
                //}
                //Console.WriteLine("");
                //Console.WriteLine("{0} ", distances[44].ToString());
                Thread.Sleep(10);
            }
        }

        public bool StartService()
        {
            try
            {
                if (!this.urg.IsOpen)
                {
                    this.urg.Open();
                }

                this.startFlag = true;
                this.sThread = new Thread(new ThreadStart(WorkerHandler));
                this.sThread.IsBackground = true;
                this.sThread.Start();
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex);
                return false;
            }
        }

        public bool StopService()
        {
            try
            {
                urg.Write(SCIP_Writer.QT());
                urg.ReadLine();

                this.startFlag = false;
                Thread.Sleep(10);
                this.sThread.Abort();
                this.sThread = null;
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex);
                return false;
            }
        }
        public event EventHandler<UrgMessageReceveidEventArgs> OnMessageRecevied;

        private void OnLidarReceivedMessage(DataPoint[] datas)
        {
            if (OnMessageRecevied != null)
            {
                OnMessageRecevied(this, new UrgMessageReceveidEventArgs(datas));
            }
        }

    }
}
