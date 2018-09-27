using lscm.driver.lidar.hokuyo.urg;
using lscm.driver.motorcontroller.lscm;
using lscm.driver.uwb.decawave;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using MathNet.Numerics;

namespace lscm.project.followerv2
{
    public class FollowerV2
    {
        private DW1000 uwb = null;
        private MotionController motorController = null;
        private Urg04LX lidar = null;

        private EFollowerMode mode;

        #region Thread-saft for the UWB two distance values
        private static readonly object stLockUwbDistance0 = new object();
        private int _lastD0;
        private int LastD0
        {
            get
            {
                lock (stLockUwbDistance0)
                {
                    return this._lastD0;
                }
            }
            set
            {
                lock (stLockUwbDistance0)
                {
                    this._lastD0 = value;
                }
            }
        }
        private static readonly object stLockUwbDistance1 = new object();
        private int _lastD1 = 0;
        public int LastD1
        {
            get
            {
                lock (stLockUwbDistance1)
                {
                    return this._lastD1;
                }
            }
            set
            {
                lock (stLockUwbDistance1)
                {
                    this._lastD1 = value;
                }
            }
        }
        #endregion

        #region Thread-saft for the Lidar data
        private static readonly int MAX_LIDAR_DATA_SIZE = 720;
        private static readonly object stLockLidar = new object();
        private DataPoint[] _lidarData = null;
        public DataPoint[] LidarData
        {
            get
            {
                lock (stLockLidar)
                {
                    return this._lidarData;
                }
            }
            set
            {
                if (value != null)
                {
                    lock (stLockLidar)
                    {
                        this._lidarData = value;
                    }
                }
            }
        }

        #endregion

        private bool _startFlag = false;
        //private bool _startFlag = true;           
        private void FollowerWorkHandler()
        {
            Console.WriteLine("enter follower handler");
            //Console.WriteLine("FollowerV2 main loopppppp.....");
            //TODO: uwb + lidar.... (input)
            //step 1 check uwb valid and keep lidar scanning when following
            //step 2 make decision: decide turning direction
            //step 3 make sure avoid the obstacle safely ?

            //threshold configs
            int car_width = 450;               // the distance between D1 and D2, to be checked 
            int uwb_detect_thresh = 1200;       //uwb starting to detect threshold (mm)   
            int turn_thresh = 75;              //threshold for deciding uwb turning
            int safe_distance = 800;        //safe_distance from obstacle to lidar. Note that the lidar is 500mm from car edge
            int release_distance = 650;
            int emergency_distance = 700;
            int Ddistance = 0;
            int D_offset = 100;              //offset that decrease the error  measure may 10cm smaller than real 

            //speed configs
            int basic_speed = 40;
            double speed_param = 1;
            double obs_paramL = 1, obs_paramR = 1;     //for recording avoid obstacle params
            double turn_paramL = 1, turn_paramR = 1;    //for 

            //angle configs
            int scan_angleL = 145;      //scan angle when following 
            int scan_angleR = 215;
            int scan_spe_angleL = 100;   //scan angle when large angle turning following
            int scan_spe_angleR = 260;
            int release_angleL = 90;    //scan angle when obstacle avoidance finish
            int release_angleR = 270;

            //other variables and flags
            int obstacle_angleL = 0, obstacle_angleR = 0;         //record the first obstacle angle and last obstacle angle
            bool obstacle_flag = false;
            int follow_num = 0;       //a number which represent the following status    -2 -1 0 1 2 

            //kalman filter params
            double kal_R = 0.000004d;
            double kal_Q = 0.000001d;
            double kal_P = 1.0d;
            double kal_K = 0;
            double kal_D0 = this.LastD0, kal_D1 = this.LastD1;
            //int pre_D0 = this.LastD0, now_D0 = 0, pre_D1 = this.LastD1, now_D1 = 0;
            //int D_diff = 0;
            //stop the car first
            this.motorController.SendMessage("z 0 0\r\n");
            while (this.lidar.DataArray[0] != -1) ;     //make sure to get the lidar data
            //------------------------------start looping------------------------//
            while (_startFlag)
            {
                
                #region step 1 execute following once
                Ddistance = (int)Math.Sqrt((double)(0.5 * (this.LastD0 * this.LastD0 +
                    this.LastD1 * this.LastD1 - 0.5 * car_width * car_width))) + D_offset;                //calculate the distance between the object and the middle of the uwb
                while (Ddistance < uwb_detect_thresh)
                {
                    Ddistance = (int)Math.Sqrt((double)(0.5 * (this.LastD0 * this.LastD0 +
                                this.LastD1 * this.LastD1 - 0.5 * car_width * car_width))) + D_offset;
                    //follower_flag = false;
                    this.motorController.SendMessage("z 0 0\r\n");
                    //Console.Write(Ddistance);
                    Console.WriteLine("Ddistance stop in following");
                    Thread.Sleep(200);
                    kal_D0 = this.LastD0;
                    kal_D1 = this.LastD1;
                }

                kal_K = kal_P / (kal_P + kal_R);
                kal_D0 = kal_D0 + kal_K * (this.LastD0 - kal_D0);
                kal_D1 = kal_D1 + kal_K * (this.LastD1 - kal_D1);
                kal_P = (1 - kal_K) * kal_P + kal_Q;

                if (Ddistance >= uwb_detect_thresh)
                {
                    follow_num = 0;
                    if ((this.LastD0 - this.LastD1) >= turn_thresh)
                    {
                        turn_paramL = 1.2;
                        turn_paramR = 0.8;
                        follow_num = 1;
                        if ((this.LastD0 - this.LastD1) >= 4 * turn_thresh)
                        {
                            turn_paramL += 0.2;
                            turn_paramR -= 0.2;
                            follow_num = 2;
                        }
                    }
                    else if ((this.LastD0 - this.LastD1) <= turn_thresh * (-1))
                    {
                        turn_paramL = 0.8;          //param needs tuning
                        turn_paramR = 1.2;
                        follow_num = -1;
                        if ((this.LastD0 - this.LastD1) <= turn_thresh * (-4))
                        {
                            turn_paramL -= 0.2;          //param needs tuning
                            turn_paramR += 0.2;
                            follow_num = -2;
                        }
                    }
                    string cmd = string.Format("z {0:0} {1:0}\r\n", basic_speed * speed_param * turn_paramL,
                        basic_speed * speed_param * turn_paramR);      //speed param is declared but not in use
                    this.motorController.SendMessage(cmd);
                    //speed_param = 1;        //reset params
                    turn_paramL = 1;
                    turn_paramR = 1;
                    Thread.Sleep(20);
                }
                #endregion

                #region step 2  check lidar scanning when keeps following and decide speed with obs angle

                if (!obstacle_flag)
                {
                    //reset the parameters
                    obstacle_angleL = 0;
                    obstacle_angleR = 0;
                    if (follow_num == -2 || follow_num == 2)        //when large angle following, increase scan range 
                    {
                        scan_angleL = scan_spe_angleL;
                        scan_angleR = scan_spe_angleR;
                    }
                    //scan for obstacles
                    for (int i = scan_angleL; i < scan_angleR; i++)          //   \90   270/  detect clockwise. Check object.smaller than 120 will not influence the following
                    {
                        if (i < 150 || i > 210) safe_distance = 650;
                        else safe_distance = 800;
                        if (this.lidar.DataArray[i] * this.lidar.DataArray[i + 1] * this.lidar.DataArray[i + 2] > 0         //clear invalid points
                            && this.lidar.DataArray[i] < safe_distance && this.lidar.DataArray[i + 1] < safe_distance && this.lidar.DataArray[i + 2] < safe_distance)
                        {
                            obstacle_flag = true;
                            obstacle_angleL = i;                //get the obstacle left edge
                            break;
                        }
                        //if no break ,means no obstacles         
                    }
                    //if obstacle detected, and left side of obstacle found, find the right side
                    if (obstacle_flag)
                    {
                        for (int j = 0; j < 30; j++)        //decide obstacle right edge, assume obstacle is smaller than 30 degrees
                        {
                            if (this.lidar.DataArray[obstacle_angleL + j] <= 0 || this.lidar.DataArray[obstacle_angleL + j] > safe_distance)
                            {
                                obstacle_angleR = obstacle_angleL + j;      //right edge of the obstacle
                                break;
                            }
                        }
                        if (obstacle_angleR == 0) obstacle_angleR = obstacle_angleL + 30;   //in case cannot find right side
                        if (obstacle_angleR > scan_angleR) obstacle_angleR = scan_angleR;                   //in case cannot find right side
                    }
                    safe_distance = 800; // resest parameter
                    Thread.Sleep(20);
                }
                #endregion

                # region step 3 when avoid mode, it will stay in this loop 
                int escape_counter = 0;
                while (obstacle_flag)           //follow flag is JUST FOR EMERGENCY STOP
                {
                    //Console.WriteLine("avoiding");
                    //////////////////////security adding////////////////////////////
                    Ddistance = (int)Math.Sqrt((double)(0.5 * (this.LastD0 * this.LastD0 +
                             this.LastD1 * this.LastD1 - 0.5 * car_width * car_width))) + D_offset;
                    while (Ddistance < uwb_detect_thresh)
                    {
                        Ddistance = (int)Math.Sqrt((double)(0.5 * (this.LastD0 * this.LastD0 +
                                    this.LastD1 * this.LastD1 - 0.5 * car_width * car_width))) + D_offset;
                        //follower_flag = false;
                        this.motorController.SendMessage("z 0 0\r\n");
                        Console.WriteLine("Ddistance stop in obs");
                        Thread.Sleep(50);
                    }
                    ///////////////////////////security ending////////////////////////
                    if ((obstacle_angleR > obstacle_angleL) && obstacle_angleL > 180)             //obstacle on the right, the angle may change from time to time
                    {
                        obs_paramL = 0.9;                  
                        obs_paramR = 1.3;
                        Console.Write(" turn left ");
                    }
                    else if ((obstacle_angleR > obstacle_angleL) && obstacle_angleL <= 180)        //obstacle on the left
                    {
                        obs_paramL = 1.3;
                        obs_paramR = 0.9;
                        Console.Write(" turn right ");
                    }

                    ////////////////////////add a emergency button
                    for (int i = 160; i < 200; i++) 
                    {
                        if (this.lidar.DataArray[i] * this.lidar.DataArray[i + 1] > 0         //clear invalid points
                            && this.lidar.DataArray[i] < safe_distance && this.lidar.DataArray[i + 1] < emergency_distance)    
                        {
                            obs_paramL = 0;
                            obs_paramR = 0;
                            Console.WriteLine("emergency stop in obs, front obstacle");
                            Thread.Sleep(100);
                            break;
                        }
                    }

                    string obs_cmd = string.Format("z {0:0} {1:0}\r\n", basic_speed * obs_paramL, basic_speed * obs_paramR);
                    this.motorController.SendMessage(obs_cmd);

                    for (int i = release_angleL; i <= release_angleR; i++)          //   \90   270/  detect clockwise.         //try to decrease a little?
                    {
                        if (i < 150 && this.lidar.DataArray[i] > 0 && this.lidar.DataArray[i] < release_distance)
                        {
                            Console.Write("zoneAAAA");
                            if (obstacle_angleL > 0 && obstacle_angleL < 180) break;//means turn right
                            //else if (obstacle_angleL >= 180)
                            //{
                            //    obs_paramL = 0.6;
                            //    obs_paramR = 0.6;
                            //    break;  //slow the turning speed  
                            //}
                        }
                        else if (i >= 150 && i < 180 && this.lidar.DataArray[i] > 0 && this.lidar.DataArray[i] < 1.1 * safe_distance)
                        {
                            Console.Write("zoneBBBB");
                            if (obstacle_angleL > 0 && obstacle_angleL < 180) break;//means turn right
                        }
                        else if ((i >= 180 && i < 210) && this.lidar.DataArray[i] > 0 && this.lidar.DataArray[i] < 1.1 * safe_distance)
                        {
                            Console.Write("zoneCCCC");
                            if (obstacle_angleL >= 180) break;//means turn right

                        }
                        else if (i >= 210 && this.lidar.DataArray[i] > 0 && this.lidar.DataArray[i] < release_distance)
                        {
                            Console.Write("zoneDDDD");
                            if (obstacle_angleL >= 180) break;//means turn right
                        }
                        else if (i >= release_angleR)  //pay attention to this edge
                        {
                            escape_counter += 1;
                            if (escape_counter >= 5)
                            {             
                                Console.WriteLine("obstacle cleared");
                                obs_paramL = 1;                  //reset params and jump out of the loop
                                obs_paramR = 1;
                                obstacle_flag = false;              //if no break ,means obstacle is cleared, jump out of the loop
                                kal_D0 = this.LastD0;
                                kal_D1 = this.LastD1;
                            }
                        }
                    }
                    Thread.Sleep(50);
                }
                #endregion
                Thread.Sleep(50);
            }
        }
        //note: turning(1.2 0.8)  following(1.4 0.8)
        public FollowerV2()
        {
            mode = EFollowerMode.FollowUwbMode;
            this.LastD0 = 0;
            this.LastD1 = 0;
            //this.LidarData = new DataPoint[MAX_LIDAR_DATA_SIZE];
            //for (int i = 0; i < MAX_LIDAR_DATA_SIZE; i++)
            //{
            //    this.LidarData[i] = new DataPoint()
            //    {
            //        Degree = 0.0f,
            //        Distance = 0,
            //    };
            //}
        }

        #region Init Stages
        public void Init()
        {
            try
            {
                InitUwb();
                Console.WriteLine("uwb init");
                InitMotorController();
                Console.WriteLine("motor init");
                InitLidar();
                Console.WriteLine("lidar init");
            }
            catch (Exception)
            {
                throw;
            }
        }

        private void InitUwb()
        {
            try
            {
                if (this.uwb == null)
                {
                    //TODO: assign the COM port
                    this.uwb = new DW1000("COM9", 115200);
                }
            }
            catch (Exception)
            {
                throw;
            }
        }

        private void InitMotorController()
        {
            try
            {
                if (this.motorController == null)
                {
                    //TODO: assign the COM port
                    this.motorController = new MotionController("COM10", 115200);
                }
            }
            catch (Exception)
            {
                throw;
            }
        }

        private void InitLidar()
        {
            try
            {
                //TODO: assign the COM port
                this.lidar = new Urg04LX("COM6", 115200);
            }
            catch (Exception)
            {
                throw;
            }
        }
        #endregion

        #region Start all services
        public void StartService()
        {
            try
            {
                StartUwb();
                StartMotorController();
                StartLidar();
                StartFollowerService();
            }
            catch (Exception)
            {
                throw;
            }
        }

        private void StartUwb()
        {
            try
            {
                this.uwb.OnReceivedMessage += Uwb_OnReceivedMessage;
                this.uwb.StartService();
            }
            catch (Exception)
            {
                throw;
            }
        }

        private void StartMotorController()
        {
            try
            {
                this.motorController.OnReceivedMessage += MotorController_OnReceivedMessage;
                this.motorController.StartService();
            }
            catch (Exception)
            {
                throw;
            }
        }

        private void StartLidar()
        {
            try
            {
                //this.lidar.OnMessageRecevied += Lidar_OnMessageRecevied;
                this.lidar.StartService();
            }
            catch (Exception)
            {
                throw;
            }
        }

        private void StartFollowerService()
        {
            try
            {
                this._startFlag = true;
                var wThread = new Thread(new ThreadStart(FollowerWorkHandler))
                {
                    IsBackground = true
                };
                wThread.Start();
            }
            catch (Exception)
            {
                throw;
            }
        }
        #endregion

        #region device service handler
        private void Uwb_OnReceivedMessage(object sender, DW1000MessageReceivedEventArgs e)
        {
            if (e != null)
            {
                this.LastD0 = e.D0;
                this.LastD1 = e.D1;
            }
        }

        private void MotorController_OnReceivedMessage(object sender, McuMessageReceivedEventArgs e)
        {
            if (e != null)
            {
                //Console.WriteLine("DEBUG>{0}", e.Message);
            }
        }

        //private void Lidar_OnMessageRecevied(object sender, UrgMessageReceveidEventArgs e)
        //{
        //    if (e != null)
        //    {
        //        this.LidarData = e.Data;
        //    }
        //}

        #endregion
    }
}
