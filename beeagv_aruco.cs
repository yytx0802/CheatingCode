using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Emgu.CV;
using Emgu.CV.Aruco;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System.Drawing;
using System.Threading;

namespace lscm.driver.Aruco
{
    public class ArucoModule
    {
        public ArucoModule() {
            _detectorParameters = DetectorParameters.GetDefault();
            _capture = new VideoCapture(camId);
            if (estimatePose)
            {
                bool readOk = readCameraParameters(_filename, ref _cameraMatrix, ref _distCoeffs);
                if (!readOk)
                {
                    Console.WriteLine("cannot find parameter files");
                    return;
                }
            }
        }

        private string _filename = "C:\\Users\\bjliu\\Documents\\QtProjects\\fuck.xml";
        private VideoCapture _capture = null;
        int camId = 0;
        public float markersX = 0;
        public float markersY = 0;
        int markersLength = 12;
        //int markersSeparation = 30;

        private Dictionary _dict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict6X6_100);
        private Dictionary ArucoDictionary
        {
            get
            {
                if (_dict == null)
                    _dict = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_100);
                return _dict;
            }

        }
        bool estimatePose = true;

        Mat _frame = new Mat();
        Mat _frameCopy = new Mat();
        Mat _output = new Mat();
        Mat _cameraMatrix = new Mat();
        Mat _distCoeffs = new Mat();
        Mat rvecs = new Mat();
        Mat tvecs = new Mat();
        Mat temp = new Mat();
        private VectorOfInt _markerCounterPerFrame = new VectorOfInt();

        private DetectorParameters _detectorParameters;

        private void ProcessFrame()
        {
            while (_capture != null && _capture.Ptr != IntPtr.Zero)
            {
                _capture.Retrieve(_frame, 0);
                _frame.CopyTo(_frameCopy);

                using (VectorOfInt ids = new VectorOfInt())
                using (VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF())
                using (VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF())
                {
                    //DetectorParameters p = DetectorParameters.GetDefault();
                    ArucoInvoke.DetectMarkers(_frameCopy, ArucoDictionary, corners, ids, _detectorParameters, rejected);

                    if (ids.Size > 0)
                    {
                        //ArucoInvoke.RefineDetectedMarkers(_frameCopy, ArucoBoard, corners, ids, rejected, null, null, 10, 3, true, null, _detectorParameters);
                        //cameraButton.Text = "Calibrate camera";
                        ArucoInvoke.DrawDetectedMarkers(_frameCopy, corners, ids, new MCvScalar(0, 255, 0));
                        if (!_cameraMatrix.IsEmpty && !_distCoeffs.IsEmpty)
                        {
                            ArucoInvoke.EstimatePoseSingleMarkers(corners, markersLength, _cameraMatrix, _distCoeffs, rvecs, tvecs);
                            for (int i = 0; i < ids.Size; i++)
                            {
                                using (Mat rvecMat = rvecs.Row(i))
                                using (Mat tvecMat = tvecs.Row(i))
                                using (VectorOfDouble rvec = new VectorOfDouble())
                                using (VectorOfDouble tvec = new VectorOfDouble())
                                {
                                    double[] values = new double[3];
                                    rvecMat.CopyTo(values);
                                    rvec.Push(values);
                                    tvecMat.CopyTo(values);
                                    tvec.Push(values);
                                    ArucoInvoke.DrawAxis(_frameCopy, _cameraMatrix, _distCoeffs, rvec, tvec,
                                       markersLength * 0.5f);
                                }
                            }
                        }
                        float counterX = 0, counterY = 0;
                        int count = corners.Size;
                        for (int i = 0; i < count; ++i)
                        {
                            using (VectorOfPointF corner = corners[i])
                            {
                                PointF[] cor = corner.ToArray();
                                for (int j = 0; j < cor.Length; j++)
                                {
                                    //Console.WriteLine(cor[j].X);
                                    counterX += cor[j].X;
                                    counterY += cor[j].Y;
                                }
                                markersX = counterX / 4;
                                markersY = counterY / 4;
                            }
                        }
                    }
                }
                CvInvoke.Undistort(_frameCopy, _output, _cameraMatrix, _distCoeffs);
                CvInvoke.Imshow("out", _output);
                CvInvoke.WaitKey(10);
                //Console.WriteLine("markersX is " + markersX);
               // Console.WriteLine("markersY is " + markersY);
            }
            //else
            //{
            Console.WriteLine("VideoCapture was not created");
            //}
        }
        private bool readCameraParameters(string filename, ref Mat camMatrix, ref Mat distCoeffs)
        {
            FileStorage fs = new FileStorage(filename, FileStorage.Mode.Read);
            if (!fs.IsOpened) return false;
            //camMatrix = fs.GetNode("camMatrix");
            if (!fs.IsOpened) Console.WriteLine("can't find files");
            Console.WriteLine(fs.GetNode("image_width").ReadInt());
            fs.GetNode("camera_matrix").ReadMat(camMatrix);
            fs.GetNode("distortion_coefficients").ReadMat(distCoeffs);
            return true;
        }
        public void StartService()
        {
            try
            {
                var wThread = new Thread(new ThreadStart(ProcessFrame))
                {
                    IsBackground = true
                };
                wThread.Start();
            }
            catch (Exception)
            {
                throw;
            }

            //while (true)
            //{
            //    ProcessFrame();
            //    Console.WriteLine("markersX is " + markersX);
            //    Console.WriteLine("markersY is " + markersY);
            //}
        }


    }


}
