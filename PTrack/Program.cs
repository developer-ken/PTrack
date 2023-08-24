using DFBSharp;
using OpenCvSharp;
using System.Drawing;
using System.IO.Ports;
using static System.Net.Mime.MediaTypeNames;
using Point = OpenCvSharp.Point;
using Size = OpenCvSharp.Size;
using System.Device.Gpio;
using System;

namespace PTrack
{
    internal class Program
    {
        static FreeRollingCamera cam;// = new FreeRollingCamera(0);
        static Calibration2d calib = new Calibration2d();
        static SerialPort com;// = new SerialPort("COM8", 115200);
        //static SerialPort commotor;
        static FrameBuffer fb;
        static int currentX = 0, currentY = 0;
        public static bool IsBtnPressed = false;
        public static bool IsBtn2Pressed = false;

        static void MoveCommand(int x, int y, uint interval)
        {
            if (x > 500) x = 500;
            if (x < -500) x = -500;
            if (y > 500) y = 500;
            if (y < -500) y = -500;
            byte[] buffer = new byte[12];
            Buffer.BlockCopy(BitConverter.GetBytes(-x), 0, buffer, 0, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(-y), 0, buffer, 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(interval), 0, buffer, 8, 4);
            com.Write(buffer, 0, 12);
            Thread.Sleep(10);
            currentX += x;
            currentY += y;
            Console.WriteLine($"MoveCommand: {x}, {y}, {interval}");
        }

        static void WaitBtn()
        {
            IsBtnPressed = false;
            while (!IsBtnPressed) Thread.Sleep(1);
            Thread.Sleep(500);
            IsBtnPressed = false;
        }

        static void MoveAbs(int x, int y, uint interval)
        {
            MoveCommand(x - currentX, y - currentY, interval);
        }

        static void Calibrate(int gridsize = 20, int total = 500)
        {
            int lx = 0;
            for (int y = 0; y < total / gridsize; y++)
            {
                {
                    var point = ShotForGreenSpot();
                    calib.AddCalibrationPoint((point.X, point.Y), (lx * gridsize, y * gridsize));
                }
                for (int x = 0; x < total / gridsize; x++)
                {
                    if (y % 2 == 0)
                        MoveCommand(gridsize, 0, 300);
                    else
                        MoveCommand(-gridsize, 0, 300);
                    Thread.Sleep(300);
                    var point = ShotForGreenSpot();
                    calib.AddCalibrationPoint((point.X, point.Y), (x * gridsize, y * gridsize));
                    lx = x;
                }
                MoveCommand(0, gridsize, 500);
                Thread.Sleep(300);
            }
            MoveAbs(0, 0, 1000);
        }

        static void MoveToTargetPoint(Point p)
        {
            const double Kp = 0.5, Ki = 0, Kd = 0; // PID constants
            double integral = 0, error_prev = 0;
            const int interval = 500; // MoveCommand interval
            var spot = ShotForRedSpot();
            while (Math.Abs(spot.X - p.X) > 2 || Math.Abs(spot.Y - p.Y) > 2) // threshold for error
            {
                double error = Math.Sqrt(Math.Pow(p.X - spot.X, 2) + Math.Pow(p.Y - spot.Y, 2));
                integral += error;
                if (integral * Ki > 100)
                {
                    integral = 100 / Ki;
                }
                double derivative = error - error_prev;
                double output = Kp * error + Ki * integral + Kd * derivative;
                int x_move = (int)Math.Round(output * Math.Cos(Math.Atan2(p.Y - spot.Y, p.X - spot.X)));
                int y_move = (int)Math.Round(output * Math.Sin(Math.Atan2(p.Y - spot.Y, p.X - spot.X)));
                MoveCommand(x_move, y_move, interval);
                error_prev = error;
                spot = ShotForRedSpot();
            }
        }

        static bool GreenGoesToRed()
        {
            const double Kp = 3, Ki = 0.05, Kd = 1; // PID constants
            double integral = 0, error_prev = 0;
            const int interval = 500; // MoveCommand interval
            var p2pp = Shot4DuoColorPoints();
            var spot = p2pp.Item1;
            var p = p2pp.Item2;
            while (Math.Abs(spot.X - p.X) > 15 || Math.Abs(spot.Y - p.Y) > 15) // threshold for error
            {
                try
                {
                    double error = Math.Sqrt(Math.Pow(p.X - spot.X, 2) + Math.Pow(p.Y - spot.Y, 2));
                    integral += error;
                    if (integral * Ki > 100)
                    {
                        integral = 100 / Ki;
                    }
                    double derivative = error - error_prev;
                    double output = Kp * error + Ki * integral + Kd * derivative;
                    int x_move = (int)Math.Round(output * Math.Cos(Math.Atan2(p.Y - spot.Y, p.X - spot.X)));
                    int y_move = (int)Math.Round(output * Math.Sin(Math.Atan2(p.Y - spot.Y, p.X - spot.X)));
                    MoveCommand(x_move, y_move, interval);
                    error_prev = error;
                    p2pp = Shot4DuoColorPoints();
                    spot = p2pp.Item1;
                    p = p2pp.Item2;
                }
                catch (Exception err)
                {
                    Console.WriteLine(err.Message);
                }
                if (IsBtnPressed)
                {
                    BeepStop();
                    Thread.Sleep(1000);
                    IsBtnPressed = false;
                    IsBtn2Pressed = false;
                    while (true)
                    {
                        if (IsBtnPressed)
                        {
                            break;
                        }
                        if (IsBtn2Pressed)
                        {
                            return true;
                        }
                    }
                    Thread.Sleep(1000);
                    IsBtnPressed = false;
                }
            }
            return false;
        }

        /// <summary>
        /// Track both green and red spot
        /// </summary>
        /// <returns></returns>
        /// <exception cref="Exception"></exception>
        static (Point, Point) Shot4DuoColorPoints((Point, Point) lastp)
        {
            Point p1 = new Point(), p2 = new Point();
            using (var f = cam.GetFrame(-5))
            using (var g = CVUtil.GreenMinusRG(f))
            using (var r = CVUtil.RedMinusBG(f))
            using (Mat point = new Mat())
            {
                {
                    Cv2.MinMaxLoc(g, out double minvalg, out double maxvalg, out _, out Point maxpg);
                    int thresholdg = (int)((maxvalg + minvalg) / 2);
                    Cv2.Threshold(g, g, thresholdg, 255, ThresholdTypes.Binary);
                    Cv2.FindContours(g, out Point[][] c, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);
                    if (c.Length == 0) throw new Exception("No green spot found");
                    Point target = new Point();
                    double score = double.MinValue;
                    foreach (var cc in c)
                    {
                        var rect = Cv2.BoundingRect(cc);
                        if()
                        Mat roi = f.SubMat(rect);
                        Scalar avg = Cv2.Mean(roi);
                        var brightness = (avg.Val0 + avg.Val1 + avg.Val2 + avg.Val3) / 4;
                        if (brightness > score)
                        {
                            score = brightness;
                            target = new Point((rect.Left + rect.Right) / 2, (rect.Bottom + rect.Top) / 2);
                        }
                    }
                    Console.WriteLine("Red score:" + score);
                    if (score < 50) throw new Exception("No green spot found");
                    p1 = target;
                }
                {
                    Cv2.MinMaxLoc(r, out double minvalr, out double maxvalr, out _, out Point maxpr);
                    int thresholdr = (int)((maxvalr + minvalr) / 2);
                    Cv2.Threshold(r, r, thresholdr, 255, ThresholdTypes.Binary);
                    Cv2.FindContours(r, out Point[][] c, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);
                    if (c.Length == 0) throw new Exception("No red spot found");
                    Point target = new Point();
                    double score = double.MinValue;
                    foreach (var cc in c)
                    {
                        var rect = Cv2.BoundingRect(cc);
                        Mat roi = f.SubMat(rect);
                        Scalar avg = Cv2.Mean(roi);
                        var brightness = (avg.Val0 + avg.Val1 + avg.Val2 + avg.Val3) / 4;
                        if (brightness > score)
                        {
                            score = brightness;
                            target = new Point((rect.Left + rect.Right) / 2, (rect.Bottom + rect.Top) / 2);
                        }
                    }
                    Console.WriteLine("Green score:" + score);
                    if (score < 50) throw new Exception("No green spot found");
                    p2 = target;
                }
                Cv2.DrawMarker(f, p1, Scalar.Green);
                Cv2.DrawMarker(f, p2, Scalar.Red);
                Visualize(f);
            }
            return (p1, p2);
        }

        static Point Center(Rect rect)
        {
            return new Point((rect.Left + rect.Right) / 2, (rect.Bottom + rect.Top) / 2);
        }

        static Point ShotForGreenSpot()
        {
            using (var f = cam.GetFrame(-5))
            using (var g = CVUtil.GreenMinusRG(f))
            using (Mat point = new Mat())
            {
                Cv2.MinMaxLoc(g, out double minval, out double maxval, out _, out Point maxp);
                int threshold = (int)((maxval + minval) / 2);
                Cv2.Threshold(g, g, threshold, 255, ThresholdTypes.Binary);
                Cv2.FindContours(g, out Point[][] c, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);
                if (c.Length == 0) throw new Exception("No green spot found");
                Point target = new Point();
                double score = double.MinValue;
                foreach (var cc in c)
                {
                    var rect = Cv2.BoundingRect(cc);
                    Mat roi = f.SubMat(rect);
                    Scalar avg = Cv2.Mean(roi);
                    var brightness = (avg.Val0 + avg.Val1 + avg.Val2 + avg.Val3) / 4;
                    if (brightness > score)
                    {
                        score = brightness;
                        target = new Point((rect.Left + rect.Right) / 2, (rect.Bottom + rect.Top) / 2);
                    }
                }
                f.DrawMarker(target, Scalar.Red);
                return target;
            }
        }

        static Point ShotForRedSpot()
        {
            using (var f = cam.GetFrame(-5))
            using (var r = CVUtil.RedMinusBG(f))
            using (Mat point = new Mat())
            {
                Cv2.MinMaxLoc(r, out double minval, out double maxval, out _, out Point maxp);
                int threshold = (int)((maxval + minval) / 2);
                Cv2.Threshold(r, r, threshold, 255, ThresholdTypes.Binary);
                Cv2.FindContours(r, out Point[][] c, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);
                if (c.Length == 0) throw new Exception("No green spot found");
                Point target = new Point();
                double score = double.MinValue;
                foreach (var cc in c)
                {
                    var rect = Cv2.BoundingRect(cc);
                    Mat roi = f.SubMat(rect);
                    Scalar avg = Cv2.Mean(roi);
                    var brightness = (avg.Val0 + avg.Val1 + avg.Val2 + avg.Val3) / 4;
                    if (brightness > score)
                    {
                        score = brightness;
                        target = new Point((rect.Left + rect.Right) / 2, (rect.Bottom + rect.Top) / 2);
                    }
                }
                return target;
            }
        }

        static void BeepStart()
        {
            byte[] buffer = new byte[12];
            Buffer.BlockCopy(BitConverter.GetBytes(0), 0, buffer, 0, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(0), 0, buffer, 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(0), 0, buffer, 8, 4);
            com.Write(buffer, 0, 12);
            Thread.Sleep(10);
        }

        static void BeepStop()
        {
            byte[] buffer = new byte[12];
            Buffer.BlockCopy(BitConverter.GetBytes(0), 0, buffer, 0, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(0), 0, buffer, 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(1), 0, buffer, 8, 4);
            com.Write(buffer, 0, 12);
            Thread.Sleep(10);
        }

        static void Main(string[] args)
        {
            if (OperatingSystem.IsWindows())
            {
                cam = new FreeRollingCamera(0);
                com = new SerialPort("COM8", 115200);
                //commotor = new SerialPort("COMx", 9600);
            }
            else
            {
                Console.WriteLine("检测到Linux，使用/dev/videoUSBx和/dev/ttyUSB0");
                cam = new FreeRollingCamera("/dev/videoUSB0");
                com = new SerialPort("/dev/ttyUSBSTM32", 115200);
                //commotor = new SerialPort("/dev/ttyUSBSTEPPER", 9600);
            }
            cam.BeginRoll();
            com.Open();
            Thread th = new Thread(new ThreadStart(() =>
            {
                while (true)
                {
                    var b = com.ReadByte();
                    if (b == 0xfa)
                    {
                        Console.WriteLine("pressed");
                        IsBtnPressed = true;
                    }
                    if (b == 0xfb)
                    {
                        Console.WriteLine("btn2 pressed");
                        IsBtn2Pressed = true;
                    }
                }
            }));
            th.Start();
            //commotor.Open();
            if (OperatingSystem.IsWindows())
            {
                Console.WriteLine("检查点1");
                Console.ReadLine();
            }
            else
            {
                Console.WriteLine("检查点1");
                WaitBtn();
                Thread.Sleep(1000);
            }

            Point center = new Point(), lu = new Point(),
                ru = new Point(), rb = new Point(), lb = new Point();

            IsBtnPressed = false;
            while (!IsBtnPressed)
            {
                using (var f = cam.GetFrame(-5))
                {
                    Cv2.PutText(f, "Tracker ready", new Point(10, 30), HersheyFonts.HersheyPlain, 2, Scalar.Red);
                    Visualize(f);
                }
            }
            GC.Collect();
            Thread.Sleep(500);
            IsBtnPressed = false;
            while (true)
            {
                try
                {
                    if (GreenGoesToRed())
                    {// goes into preview
                        IsBtnPressed = false;
                        //cam.Exposure();
                        while (!IsBtnPressed)
                        {
                            using (var f = cam.GetFrame())
                            {
                                Cv2.PutText(f, "Tracker ready", new Point(10, 30), HersheyFonts.HersheyPlain, 2, Scalar.Red);
                                Visualize(f);
                            }
                        }
                        //cam.Exposure(false, -10);
                        continue;
                    }
                    byte[] buffer = new byte[12];
                    BeepStart();
                    if (IsBtnPressed)
                    {
                        BeepStop();
                        Thread.Sleep(1000);
                        IsBtnPressed = false;
                        while (!IsBtnPressed) Thread.Sleep(1);
                        Thread.Sleep(1000);
                        IsBtnPressed = false;
                    }
                }
                catch (Exception err)
                {
                    Console.WriteLine(err.Message);
                }
            }
        }

        static void LockUnlockMotor(byte id = 0x00, bool _lock = true)
        {
            //commotor.Write(new byte[] { id, 0xf3, _lock ? (byte)0x01 : (byte)0x00, 0x6b }, 0, 4);
        }

        private static void FollowCtr()
        {
            {
            Refind:
                Console.WriteLine("抓帧...");
                using (var f = cam.GetFrame(-5))
                {
                    Visualize(f);
                    try
                    {
                        Console.WriteLine("计算...");
                        var rectp = RectTrack.DetectSkewRect(f);
                        Console.WriteLine("轮廓检出");
                        foreach (var p in rectp.ContourCW)
                        {
                            using (Mat fdraw = f.Clone())
                            {
                                Cv2.DrawMarker(fdraw, p, Scalar.Blue, MarkerTypes.Star, thickness: 2);
                                Visualize(fdraw);
                            }
                            Console.WriteLine($"新目标点：{p.X},{p.Y}");
                            MoveToTargetPoint(p);
                        }
                        Console.WriteLine($"回到第一个角点");
                        using (Mat fdraw = f.Clone())
                        {
                            Cv2.DrawMarker(fdraw, rectp.ContourCW.First(), Scalar.Blue, MarkerTypes.Star, thickness: 2);
                            Visualize(fdraw);
                        }
                        MoveToTargetPoint(rectp.ContourCW.First());
                    }
                    catch (Exception err)
                    {
                        Console.WriteLine(err.Message);
                        Console.WriteLine("无轮廓检出");
                        //Visualize(f);
                        goto Refind;
                    }
                }
            }
        }

        static Mat rpic = new Mat();
        static Mat ppic = new Mat();
        static void Visualize(Mat pic)
        {
            if (OperatingSystem.IsWindows())
            {
                Cv2.ImShow("Visualize", pic);
                Cv2.WaitKey(1);
            }
            else
            {
                fb ??= new FrameBuffer("/dev/fb0");
                Cv2.Resize(pic, rpic, new Size(fb.Width, fb.Height));
                Cv2.CvtColor(rpic, ppic, ColorConversionCodes.BGR2BGRA);
                fb.DrawBitmap(0, 0, ppic.Width, ppic.Height, ppic.DataStart);
            }
        }
    }
}