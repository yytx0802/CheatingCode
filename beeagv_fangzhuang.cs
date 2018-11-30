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
        private static readonly object stLockUwbDistance3 = new object();
        private int _lastTick;
        private int LastTick
        {
            get
            {
                lock (stLockUwbDistance3)
                {
                    return this._lastTick;
                }
            }
            set
            {
                lock (stLockUwbDistance3)
                {
                    this._lastTick = value;
                }
            }
        }
        #endregion

        //#region Thread-saft for the Lidar data
        //private static readonly int MAX_LIDAR_DATA_SIZE = 720;
        //private static readonly object stLockLidar = new object();
        //private DataPoint[] _lidarData = null;
        //public DataPoint[] LidarData
        //{
        //    get
        //    {
        //        lock (stLockLidar)
        //        {
        //            return this._lidarData;
        //        }
        //    }
        //    set
        //    {
        //        if (value != null)
        //        {
        //            lock (stLockLidar)
        //            {
        //                this._lidarData = value;
        //            }
        //        }
        //    }
        //}

        //#endregion

        private bool _startFlag = false;


        private void FollowerWorkHandler()
        {
            Console.WriteLine("enter follower handler");
            //Console.WriteLine("FollowerV2 main loopppppp.....");
            //TODO: uwb + lidar.... (input)
            //step 1 check uwb valid a nd keep lidar scanning when following
            //step 2 make decision: decide turning direction
            //step 3 make sure avoid the obstacle safely ?



            //threshold configs
            int car_width = 450;               // the distance between D1 and D2, to be checked 
            int uwb_detect_thresh = 1000;       //uwb starting to detect threshold (mm)  1050 
            double turn_thresh = 78;              //threshold for deciding uwb turning
            int safe_distance = 1000;        //safe_distance from obstacle to lidar. Note that the lidar is 500mm from car edge 800
            int release_distance = 500;
            int emergency_distance = 900;   //750
            int Ddistance = 0;
            int D_offset = 100;              //offset that decrease the error  measure may 10cm smaller than real 

            //speed configs
            double basic_speed = 50;
            double speed_param = 1.2;
            double obs_paramL = 1, obs_paramR = 1;     //for recording avoid obstacle params
            double turn_paramL = 1, turn_paramR = 1;    //for 

            //angle configs
            int scan_angleL = 130;      //scan angle when following 
            int scan_angleR = 230;
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
            double kal_P = 2.5d;        //2.5 when in front
            double kal_K = 0.0d;        //0.4 when in front
            double kal_D0 = this.LastD0, kal_D1 = this.LastD1;
            //int pre_D0 = this.LastD0, now_D0 = 0, pre_D1 = this.LastD1, now_D1 = 0;
            //int D_diff = 0;

            double last_VL = 0, last_VR = 0;
            //stop the car first
            this.motorController.SendMessage("z 0 0\r\n");
            while (this.lidar.DataArray[0] != -1) ;     //make sure to get the lidar data
            //------------------------------start looping------------------------//
            while (_startFlag)
            {
                if (System.Environment.TickCount - this.LastTick >= 1000)       //check uwb valid every time
                {
                    this.LastD0 = 0;
                    this.LastD1 = 0;
                    Console.WriteLine("No current uwb signal!");
                }
                #region step 1 execute following once
                Ddistance = cal_distance(this.LastD0, this.LastD1, D_offset);                //calculate the distance between the object and the middle of the uwb
                while ((System.Environment.TickCount - this.LastTick >= 500) || Ddistance < uwb_detect_thresh)
                {
                    Ddistance = cal_distance(this.LastD0, this.LastD1, D_offset);
                    //follower_flag = false;
                    //this.motorController.SendMessage("z 0 0\r\n");
                    setspeedV2(0, 0, ref last_VL, ref last_VR);
                    //Console.Write(Ddistance);
                    Console.WriteLine("Ddistance stop in following");
                    Thread.Sleep(100);
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
                    speed_param = 0.1d + (double)Ddistance / 1800d;
                    if ((kal_D0 - kal_D1) >= turn_thresh)
                    {
                        turn_paramL = 1.2;
                        turn_paramR = 0.8;
                        if (Ddistance > 2000)
                        {
                            turn_paramL = 1.1;
                            turn_paramR = 0.9;
                        }
                        follow_num = 1;
                        if ((kal_D0 - kal_D1) >= 3 * turn_thresh)
                        {
                            turn_paramL = 1.5;
                            turn_paramR = 0.6;
                            follow_num = 2;
                        }
                    }
                    else if ((kal_D0 - kal_D1) <= turn_thresh * (-1))
                    {
                        turn_paramL = 0.8;          //param needs tuning
                        turn_paramR = 1.2;
                        follow_num = -1;
                        if (Ddistance > 2000)
                        {
                            turn_paramL = 0.9;
                            turn_paramR = 1.1;
                        }
                        if ((kal_D0 - kal_D1) <= turn_thresh * (-3))
                        {
                            turn_paramL = 0.6;          //param needs tuning
                            turn_paramR = 1.5;
                            follow_num = -2;
                        }
                    }

                    setspeedV2(basic_speed * speed_param * turn_paramL, basic_speed * speed_param * turn_paramR, ref last_VL, ref last_VR);
                    //string cmd = string.Format("z {0:0} {1:0}\r\n", last_VL, last_VR);      //speed param is declared but not in use
                    //this.motorController.SendMessage(cmd);
                    speed_param = 1;        //reset params
                    turn_paramL = 1;
                    turn_paramR = 1;
                    //Thread.Sleep(20);
                }
                #endregion

                #region step 2  check lidar scanning when keeps following and decide speed with obs angle

                if (!obstacle_flag)
                {
                    //reset the parameters
                    obstacle_angleL = 0;
                    obstacle_angleR = 0;
                    //if (follow_num == -2 || follow_num == 2)        //when large angle following, increase scan range 
                    if (follow_num != 0)        //when large angle following, increase scan range 
                    {
                        scan_angleL = scan_spe_angleL;
                        scan_angleR = scan_spe_angleR;
                    }
                    //scan for obstacles
                    for (int i = scan_angleL; i < scan_angleR; i+=2)          //   \90   270/  detect clockwise. Check object.smaller than 120 will not influence the following
                    {
                        if (i < 145 || i > 215) safe_distance = 650;
                        else safe_distance = 1000;
                        if (this.lidar.DataArray[i] * this.lidar.DataArray[i + 1] > 0         //clear invalid points
                            && this.lidar.DataArray[i] < safe_distance)
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
                    safe_distance = 700; // resest parameter
                    scan_angleL = 130;
                    scan_angleR = 230;
                    //Thread.Sleep(20);
                }
                #endregion

                # region step 3 when avoid mode, it will stay in this loop 
                int escape_counter = 0;
                while (obstacle_flag)           //follow flag is JUST FOR EMERGENCY STOP
                {
                    //Console.WriteLine("avoiding");
                    //////////////////////security adding////////////////////////////
                    if (System.Environment.TickCount - this.LastTick >= 1000)       //check uwb valid every time
                    {
                        this.LastD0 = 0;
                        this.LastD1 = 0;
                        Console.WriteLine("No current uwb signal!");
                    }
                    Ddistance = cal_distance(this.LastD0, this.LastD1, D_offset);
                    while ((System.Environment.TickCount - this.LastTick >= 1000) || Ddistance < uwb_detect_thresh)
                    {
                        Ddistance = cal_distance(this.LastD0, this.LastD1, D_offset);
                        //follower_flag = false;
                        setspeedV2(0, 0, ref last_VL, ref last_VR,5);
                        //this.motorController.SendMessage("z 0 0\r\n");
                        Console.WriteLine("Ddistance stop in obs");
                        Thread.Sleep(50);
                    }
                    ///////////////////////////security ending////////////////////////
                    if ((obstacle_angleR > obstacle_angleL) && obstacle_angleL > 180)             //obstacle on the right, the angle may change from time to time
                    {
                        obs_paramL = 0.0d;
                        obs_paramR = 0.0d;
                        Console.Write(" turn left ");
                    }
                    else if ((obstacle_angleR > obstacle_angleL) && obstacle_angleL <= 180)        //obstacle on the left
                    {
                        obs_paramL = 0.0d;
                        obs_paramR = 0.00d;
                        Console.Write(" turn right ");
                    }

                    ////////////////////////add a emergency button//////////////////////////////////////
                    for (int i = 150; i < 210; i++)
                    {
                        if (this.lidar.DataArray[i] * this.lidar.DataArray[i + 1] > 0         //clear invalid points
                            && (this.lidar.DataArray[i] < emergency_distance * (1 + last_VL / 400) || this.lidar.DataArray[i + 1] < emergency_distance * (1 + last_VL / 400)))
                        {
                            obs_paramL = 0;
                            obs_paramR = 0;
                            Console.WriteLine("emergency stop in obs, front obstacle");
                            this.motorController.SendMessage("z 0 0\r\n");
                            last_VL = 0;
                            last_VR = 0;
                            //setspeedV2(0, 0, ref last_VL, ref last_VR);
                            //Thread.Sleep(100);  
                            break;
                        }
                    }

                    setspeedV2(basic_speed * obs_paramL, basic_speed * obs_paramR, ref last_VL, ref last_VR,5);
                    //string obs_cmd = string.Format("z {0:0} {1:0}\r\n", basic_speed * obs_paramL, basic_speed * obs_paramR);
                    //this.motorController.SendMessage(obs_cmd);

                    for (int i = release_angleL; i <= release_angleR; i++)          //   \90   270/  detect clockwise.         //try to decrease a little?
                    {
                        if (i < 150 && this.lidar.DataArray[i] > 0 && this.lidar.DataArray[i] < release_distance)
                        {
                            Console.Write("zoneAAAA");
                            if (obstacle_angleL > 0 && obstacle_angleL < 180) break;//means turn right
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
                            if (escape_counter >= 2)
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
                    //Thread.Sleep(50);
                }
                #endregion
                Thread.Sleep(10);
            }
        }

        private void FollowerWorkHandler2()
        {
            Console.WriteLine("2 version controlling");
            //all params are defined in double
            //fixed params
            //double UWB_WIDTH = 450d;        //distance between D0 and D1
            //double CAR_WIDTH = 550d;
            int UWB_TIMEOUT = 500;
            int DECIDER_NUM = 3;
            //----------------frequently changes params-----------------//
            //speed settings

            double basic_speed = 40;
            double speed_param = 0;     //speed adjustment for both wheels
            double avoid_paramL = 0;     //speed adjustment for far obstacle avoid 
            double avoid_paramR = 0;
            double slow_paramL = 0;      //speed adjustment for near obstacle slow
            double slow_paramR = 0;
            double uwb_paramL = 0;       //speed adjustment for follow uwb
            double uwb_paramR = 0;
            double final_paramL = 0;
            double final_paramR = 0;

            //devide to 4 zones, side , slow , avoid ,dead
            double avoid_angL = 150;
            double avoid_angR = 210;

            //lidar detect thresh
            double uwb_detect_thresh = 1300;    //threshold for uwb stop (distance)
            double turn_thresh = 70;              //threshold for turning (D0 - D1)
            double side_thresh = 500;             //threshold for slowing down on both sides
            double stop_thresh = 700;             //threshold for stop in front (same with safe_distance)
            double slow_thresh = 1300;           //threshold for avoid in front (NOTICE: stop is prior than stop) 
            double avoid_thresh = 2000;           //threshold for avoid in front (NOTICE: stop is prior than stop) 

            //lidar data points counter     (counters will affect car behaviours)
            int side_counterL = 0;
            int side_counterR = 0;
            int avoid_counterL = 0;
            int avoid_counterR = 0;
            int slow_counterL = 0;
            int slow_counterR = 0;

            //uwb config
            double Ddistance = 0;               //initial distance from uwb to the mid of D0 D1. 
            double emergency_distance = 750;    //emergency stop distance for uwb
            double D_offset = 20;              //a small offset  
            double uwb_T0 = 0;
            double uwb_T1 = 0;
            double uwb_T2 = 0;
            //double safe_distance = 800;        //safe_distance from obstacle to lidar. Note that the lidar is 500mm from car edge
            //int release_distance = 500;  

            double last_VL = 0, last_VR = 0;        //record every speed changes 

            //kalman filter params
            double kal_R = 0.000004d;
            double kal_Q = 0.000001d;
            double kal_P = 2.5d;        //2.5 when in front
            double kal_K = 0.0d;        //0.4 when in front
            double kal_D0 = this.LastD0, kal_D1 = this.LastD1;

            int initial_counter = 0;
            int relife_counter = 0;
            int back_numberL = 0;
            int back_numberR = 0;
            int tick_now = System.Environment.TickCount;
            int dead_time = 0;
            bool dead_flag = false;

            //waiting for lidar
            while (this.lidar.DataArray[0] != -1) ;     //make sure to get the lidar data

            //waiting for uwb
            //while (initial_counter < 5)
            //{
            //    this.motorController.SendMessage("z 1 1\r\n");      //LED keep flashing
            //    Thread.Sleep(1000);
            //    this.motorController.SendMessage("z 0 0\r\n");
            //    Thread.Sleep(1000);
            //    initial_counter++;
            //}
            while (this.LastD0 * this.LastD0 == 0)
            {
                Console.WriteLine("waiting for uwb");
                Thread.Sleep(1000);
            }
            //------------------------start moving--------------------------//
            while (_startFlag)
            {
                //check uwb
                //if (System.Environment.TickCount - this.LastTick >= UWB_TIMEOUT)       //check uwb valid every time
                //{
                //    this.LastD0 = 0;
                //    this.LastD1 = 0;
                //    Console.WriteLine("No current uwb signal!");
                //    continue;
                //}

                Ddistance = cal_distance(this.LastD0, this.LastD1, D_offset);                //calculate the distance between the object and the middle of the uwb
                while ((System.Environment.TickCount - this.LastTick >= UWB_TIMEOUT) || Ddistance < uwb_detect_thresh)
                {
                    Ddistance = cal_distance(this.LastD0, this.LastD1, D_offset);
                    setspeedV2(0, 0, ref last_VL, ref last_VR);
                    //Console.Write(Ddistance);
                    Console.WriteLine("Ddistance stop");
                    Thread.Sleep(100);
                    uwb_T0 = this.LastD0 - this.LastD1;            //add a 0.5s * 2 delay for tracking people
                    uwb_T1 = this.LastD0 - this.LastD1;
                    uwb_T2 = this.LastD0 - this.LastD1;
                    kal_D0 = this.LastD0;
                    kal_D1 = this.LastD1;
                }
                //combine follow and avoid mode together
                //READ LIDAR DATA
                //deal with left side
                for (int i = 90; i < 180; i++)
                {
                    if (this.lidar.DataArray[i] <= 0 || this.lidar.DataArray[i+1] <= 0) continue;
                    if (i <= avoid_angL && this.lidar.DataArray[i] < side_thresh) side_counterL++;
                    if (i > avoid_angL && i <= 180)
                    {
                        if (this.lidar.DataArray[i] <= stop_thresh && this.lidar.DataArray[i + 1] <= stop_thresh)
                        {
                            this.motorController.SendMessage("z 0 0\r\n");
                            Console.WriteLine("lidarL stop");
                            last_VL = 0;
                            last_VR = 0;
                            speed_param = -10;
                            back_numberL++;
                            break;
                        }
                        if (this.lidar.DataArray[i] <= slow_thresh)
                        {
                            slow_counterL++;
                        }
                        else if (this.lidar.DataArray[i] <= avoid_thresh && this.lidar.DataArray[i] <= (Ddistance + 300))     //to be dicided
                        {
                            avoid_counterL++;
                        }
                    }
                }
                //deal with right side
                for (int i = 180; i <= 270; i++)
                {
                    if (this.lidar.DataArray[i] <= 0 || this.lidar.DataArray[i + 1] <= 0) continue;
                    if (i > avoid_angR && this.lidar.DataArray[i] < side_thresh) side_counterR++;
                    if (i <= avoid_angR && i >= 180)
                    {
                        if (this.lidar.DataArray[i] <= stop_thresh && this.lidar.DataArray[i + 1] <= stop_thresh)
                        {
                            this.motorController.SendMessage("z 0 0\r\n");      //set speed to zero
                            Console.WriteLine("lidarR stop");
                            last_VL = 0;
                            last_VR = 0;
                            speed_param = -10;
                            back_numberR++;
                            break;
                        }
                        if (this.lidar.DataArray[i] <= slow_thresh)
                        {
                            slow_counterR++;
                        }
                        else if (this.lidar.DataArray[i] <= avoid_thresh && avoid_thresh <= Ddistance)
                        {
                            avoid_counterR++;
                        }
                    }
                }

                //RELIFE ACTION
                if(relife_counter <= 3 && back_numberL > 25 && slow_counterR == 0)
                {
                    this.motorController.SendMessage("z -20 -20\r\n");
                    last_VL = -20;
                    last_VR = -20;
                    Thread.Sleep(1600);
                    back_numberL = 0;
                    relife_counter++;
                }
                else if (relife_counter <= 4 && back_numberR > 25 && slow_counterL == 0)
                {
                    this.motorController.SendMessage("z -20 -20\r\n");
                    last_VL = -20;
                    last_VR = -20;
                    Thread.Sleep(1600);
                    back_numberR = 0;
                    relife_counter++;
                }
                if(!dead_flag && relife_counter >= 4)
                {
                    dead_time = System.Environment.TickCount;
                    dead_flag = true;
                }
                if(dead_flag && (System.Environment.TickCount - dead_time) > 60000)
                {
                    relife_counter = 0;
                    dead_flag = false;
                }
                //DICISION MAKING
                Ddistance = cal_distance(this.LastD0, this.LastD1, D_offset);
                if ((System.Environment.TickCount - this.LastTick >= UWB_TIMEOUT) || Ddistance < uwb_detect_thresh) continue;

                kal_K = kal_P / (kal_P + kal_R);
                kal_D0 = kal_D0 + kal_K * (this.LastD0 - kal_D0);
                kal_D1 = kal_D1 + kal_K * (this.LastD1 - kal_D1);
                kal_P = (1 - kal_K) * kal_P + kal_Q;


                if ((System.Environment.TickCount - tick_now) > 100)            //500000
                {
                    uwb_T0 = uwb_T1;            //add a 0.5s * 2 delay for tracking people
                    uwb_T1 = uwb_T2;
                    uwb_T2 = kal_D0 - kal_D1;
                    tick_now = System.Environment.TickCount;
                }

                if ((uwb_T0) / turn_thresh > 1)
                {
                    uwb_paramL = 0.2;
                    uwb_paramR = -0.2;
                }
                if ((uwb_T0) / turn_thresh > 3)
                {
                    uwb_paramL = 0.4;
                    uwb_paramR = -0.3;
                }

                if ((uwb_T0) / turn_thresh < -1)
                {
                    uwb_paramL = -0.2;
                    uwb_paramR = 0.2;
                }
                if ((uwb_T0) / turn_thresh < -3)
                {
                    uwb_paramL = -0.3;
                    uwb_paramR = 0.4;
                }
                speed_param += Ddistance / 10000;        //1000 -- 0.2
                if (speed_param >= 0.2) Ddistance = 0.2;

                if (side_counterL >= DECIDER_NUM || side_counterR >= DECIDER_NUM)       //if obstacles exist on either sides, slow down
                {
                    speed_param = -0.3;
                    uwb_paramL = 0;
                    uwb_paramR = 0;
                    if (side_counterL > (side_counterR+ DECIDER_NUM)) uwb_paramL = 0.1;
                    if (side_counterR > (side_counterL + DECIDER_NUM)) uwb_paramR = 0.1;

                }
                if (slow_counterL >= DECIDER_NUM)
                {
                    avoid_counterL = 0;
                    if (slow_counterR == 0)
                    {
                        slow_paramL = (-0.02) * slow_counterL;
                        slow_paramR = -0.3;
                    }
                    else
                    {
                        slow_paramL = -0.3;
                        slow_paramR = -0.3;
                    }
                }
                if (avoid_counterL >= DECIDER_NUM)
                {
                    avoid_paramL = 0.01 * avoid_counterL;
                    avoid_paramR = (-0.01) * avoid_counterL;
                }

                if (slow_counterR >= DECIDER_NUM && slow_counterL == 0)
                {
                    avoid_counterR = 0;
                    if (slow_counterL == 0)
                    {
                        slow_paramR = (-0.02) * slow_counterR;
                        slow_paramL = -0.3;
                    }
                    else
                    {
                        slow_paramL = -0.3;
                        slow_paramR = -0.3;
                    }
                }
                if (avoid_counterL >= DECIDER_NUM)
                {
                    avoid_paramR = 0.01 * avoid_counterR;
                    avoid_paramL = (-0.01) * avoid_counterR;
                }


                final_paramL = 1 + speed_param + avoid_paramL + slow_paramL + uwb_paramL;
                final_paramR = 1 + speed_param + avoid_paramR + slow_paramR + uwb_paramR;
                if (final_paramL < 0) final_paramL = 0;
                if (final_paramR < 0) final_paramR = 0;
                if (side_counterL >= DECIDER_NUM && final_paramL < final_paramR)
                {
                    final_paramR = final_paramL;
                }
                if (side_counterR >= DECIDER_NUM && final_paramR < final_paramL)
                {
                    final_paramL = final_paramR;
                }
                //Console.Write("side L : " + side_counterL);
                //Console.WriteLine(" and side R : " + side_counterR);
                //Console.Write("avoid L : " + avoid_counterL);
                Console.WriteLine(" and avoid R : " + avoid_counterR);
                Console.Write("slow L : " + slow_counterL);
                //Console.WriteLine(" and slow R : " + slow_counterR);
                //Console.Write("uwbL : " + uwb_paramL);
                //Console.WriteLine(" and uwbR : " + uwb_paramR);
                Console.Write("L : " + basic_speed * final_paramL);
                Console.WriteLine(" and R : " + basic_speed * final_paramR);
                //string cmd = string.Format("z {0:0} {1:0}\r\n", basic_speed * final_paramL, basic_speed * final_paramR);      //speed param is declared but not in use
                //this.motorController.SendMessage(cmd);
                setspeed(basic_speed * final_paramL, basic_speed * final_paramR, ref last_VL, ref last_VR);
                //setspeedV2(basic_speed * final_paramL, basic_speed * final_paramR, ref last_VL, ref last_VR);

                speed_param = 0;     //speed adjustment for both wheels
                avoid_paramL = 0;     //speed adjustment for far obstacle avoid 
                avoid_paramR = 0;
                slow_paramL = 0;      //speed adjustment for near obstacle slow
                slow_paramR = 0;
                uwb_paramL = 0;       //speed adjustment for follow uwb
                uwb_paramR = 0;
                final_paramL = 0;
                final_paramR = 0;

                side_counterL = 0;
                side_counterR = 0;
                avoid_counterL = 0;
                avoid_counterR = 0;
                slow_counterL = 0;
                slow_counterR = 0;

                Thread.Sleep(100);

            }
        }
        private int cal_distance(double d1, double d2, double D_offset = 50)        //won't affect d0 -d1
        {
            if (d1 * d2 == 0) return 0;
            return (int)(Math.Sqrt((0.5 * (this.LastD0 * this.LastD0 + this.LastD1 * this.LastD1 - 0.5 * 450 * 450))) + D_offset);
        }
        #region out-dated control
        private void setspeed(double set_VL, double set_VR, ref double last_VL, ref double last_VR)
        {
            //control the left wheel speed and right speed seperatedly
            double step_size = 8d;
            //set acc depending on different velocities
            if ((set_VL >= (last_VL + 20)) || (set_VR >= (last_VR + 20))) step_size = 15d;
            //else if (last_VL >= 20) step_size = 4;
            if ((set_VL - last_VL) >= 0)
            {
                if ((last_VL += step_size) > set_VL) last_VL = set_VL;
            }
            else if ((set_VL - last_VL) < 0)
            {
                if ((last_VL -= step_size) < set_VL) last_VL = set_VL;
            }

            if ((set_VR - last_VR) >= 0)
            {
                if ((last_VR += step_size) > set_VR) last_VR = set_VR;
            }
            else if ((set_VR - last_VR) < 0)
            {
                if ((last_VR -= step_size) < set_VR) last_VR = set_VR;
            }

            string cmd = string.Format("z {0:0} {1:0}\r\n", last_VL, last_VR);      //speed param is declared but not in use
            this.motorController.SendMessage(cmd);
            Thread.Sleep(10);
        }
        #endregion
        private void setspeedV2(double set_VL, double set_VR, ref double last_VL, ref double last_VR, double step_size = 1.8)
        {
            // set left wheel speed
            if (set_VL > 0 && last_VL == 0) last_VL = 10;
            if (set_VR > 0 && last_VR == 0) last_VR = 10;
            if (set_VL > last_VL)
            {
                last_VL *= step_size;
                if (last_VL >= set_VL) last_VL = set_VL;
            }
            else if (set_VL < last_VL)
            {
                last_VL /= step_size;
                if (last_VL <= set_VL) last_VL = set_VL;
            }

            // set right wheel speed
            if (set_VR > last_VR)
            {
                last_VR *= step_size;
                if (last_VR >= set_VR) last_VR = set_VR;
            }
            else if (set_VR < last_VR)
            {
                last_VR /= step_size;
                if (last_VR <= set_VR) last_VR = set_VR;
            }
            //set stop threshold
            if (set_VL * set_VR == 0)
            {
                if (last_VL <= 10) last_VL = 0;
                if (last_VR <= 10) last_VR = 0;
            }

            string cmd = string.Format("z {0:0} {1:0}\r\n", last_VL, last_VR);      //speed param is declared but not in use
            this.motorController.SendMessage(cmd);
            Thread.Sleep(80);
        }

        private void check_breakpoint()
        {
            double delta_fi = 1;
            double lambda = 10;       //determined by user
            double Dmax = 0;
            double measurment_err = 30;
            bool flag = false;
            for (int i = 60; i < 240; i++)
            {
                Dmax = this.lidar.DataArray[i] * Math.Sin(delta_fi * Math.PI / 180) / Math.Sin((lambda - delta_fi) * Math.PI / 180) + 3 * measurment_err;
                if (Math.Abs(this.lidar.DataArray[i + 1] - this.lidar.DataArray[i]) > Dmax) flag = true;        // I made the calculation simpler
            }
        }

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
                    this.uwb = new DW1000("COM8", 115200);
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
                    this.motorController = new MotionController("COM19", 115200);
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
                //var wThread = new Thread(new ThreadStart(FollowerWorkHandler2))
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
                this.LastTick = System.Environment.TickCount;
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
