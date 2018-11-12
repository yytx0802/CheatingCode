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
        //DataPoint[] datapoint;

        public int[] DataArray { get; private set; } = new int[361];

        int sample_number = 682;
        float angle_range = 240;
        float angle_increase;
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

        public Urg04LX(string port, int buadRate)           //customed parameter port constructer
        {
            urg = new SerialPort(port, buadRate)
            {
                NewLine = "\n\n"
            };
        }

        private string SearchComPort()                      //check every valid port and return a valid port name
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
            urg.Write(SCIP_Writer.SCIP2());                 //set protocol
            //Thread.Sleep(10);
            string line1 = urg.ReadLine();
            Thread.Sleep(20);
            urg.Write(SCIP_Writer.MD(START, END));          //set sending message
            //Thread.Sleep(10);
            string line2 = urg.ReadLine();
            Thread.Sleep(20);

            long time_stamp = 0;
            //datapoint = new DataPoint[sample_number];
            angle_increase = (float)sample_number / angle_range;

            for(int i = 0; i <= 360 ; i++)      //from 1 -> 360 degrees. 0 is invalid data
            {
                DataArray[i] = -1;          //store the lidar data into this array, and i correspond to the angle
            }
            while (startFlag)
            {
                string receive_data = urg.ReadLine();
                if (!SCIP_Reader.MD(receive_data, ref time_stamp, ref distances))   //data stored in distances
                {
                    Console.WriteLine(receive_data);
                    continue;
                }
                if (distances.Count == 0)
                {
                    Console.WriteLine(receive_data);
                    continue;
                }

                //TODO: process the data here
                //for (int i = 0; i < datapoint.Length; i++)            
                //{
                //    datapoint[i].Degree = i * angle_increase ;
                //    datapoint[i].Distance = (int)distances[i] ;
                //}

                //  urg_angle --> distances[x]     x = 341/120 * urg_angle + 385          \-120  |0  /120   
                //  urg_angle --> lidar_angle      urg_angle = 180 - lidar_angle      /60    |0  \300

                int temp_x = 0;
                for (int lidar_angle = 61; lidar_angle < 299; lidar_angle++)       //61 and 299 for convenient
                {
                    int num = 0;
                    temp_x = 681 * (180 - lidar_angle) /240 + 341;     //convert the raw data distances[i] to DataArray[x]
                    if (distances[temp_x - 1] > 10) num++;
                    else distances[temp_x - 1] = 0;
                    if (distances[temp_x] > 10) num++;
                    else distances[temp_x] = 0;
                    if (distances[temp_x + 1] > 10) num++;
                    else distances[temp_x + 1] = 0;
                    if (num > 0) DataArray[lidar_angle] = ((int)distances[temp_x - 1] + (int)distances[temp_x] + (int)distances[temp_x + 1]) / num;
                    else DataArray[lidar_angle] = 0;
                    //DataArray[lidar_angle] = ((int)distances[temp_x - 1] + (int)distances[temp_x] + (int)distances[temp_x + 1]) / 3;
                    if (DataArray[lidar_angle] < 20) DataArray[lidar_angle] = 0;        //see small value as noise

                }

                Thread.Sleep(10);
            }
        }

        //public int getResultAt(float angle)     //   \-120  |0  /120 
        //{
        //    if (this.startFlag)
        //    {
        //        if (angle > -120 && angle < 120)
        //        {
        //            float data_angle = 120 - angle;          
        //            float temp_offset = data_angle / angle_increase;
        //            int offset = ((int)(temp_offset * 100) / 100); 
        //            return (datapoint[offset-1].Distance + datapoint[offset].Distance + datapoint[offset + 1].Distance) / 3;
        //        }
        //    }
        //    return -1;
        //}

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
                Console.WriteLine("lidar thread open");
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
