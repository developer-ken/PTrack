using OpenCvSharp;
using System.Drawing;
using System.IO.Ports;
using static System.Net.Mime.MediaTypeNames;
using Point = OpenCvSharp.Point;

namespace PTrack
{
    internal class Program
    {
        static FreeRollingCamera cam = new FreeRollingCamera(0);
        static Calibration2d calib = new Calibration2d();
        static SerialPort com = new SerialPort("COM8", 115200);
        static int currentX = 0, currentY = 0;

        static void MoveCommand(int x, int y, uint interval)
        {
            byte[] buffer = new byte[12];
            Buffer.BlockCopy(BitConverter.GetBytes(x), 0, buffer, 0, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(-y), 0, buffer, 4, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(interval), 0, buffer, 8, 4);
            com.Write(buffer, 0, 12);
            Thread.Sleep(25);
            currentX += x;
            currentY += y;
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
            const double Kp = 0.05, Ki = 0.001, Kd = 0; // PID constants
            double integral = 0, error_prev = 0;
            const int interval = 500; // MoveCommand interval
            var spot = ShotForGreenSpot();
            while (Math.Abs(spot.X - p.X) > 8 || Math.Abs(spot.Y - p.Y) > 8) // threshold for error
            {
                double error = Math.Sqrt(Math.Pow(p.X - spot.X, 2) + Math.Pow(p.Y - spot.Y, 2));
                integral += error;
                double derivative = error - error_prev;
                double output = Kp * error + Ki * integral + Kd * derivative;
                int x_move = (int)Math.Round(output * Math.Cos(Math.Atan2(p.Y - spot.Y, p.X - spot.X)));
                int y_move = (int)Math.Round(output * Math.Sin(Math.Atan2(p.Y - spot.Y, p.X - spot.X)));
                MoveCommand(x_move, y_move, interval);
                error_prev = error;
                spot = ShotForGreenSpot();
            }
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
                Cv2.ImShow("spot", f);
                Cv2.WaitKey(1);
                return target;
            }
        }

        static void Main(string[] args)
        {
            cam.BeginRoll();
            com.Open();
            while (true)
            {
                using (var f = cam.GetFrame(-5))
                {
                    Cv2.ImShow("g", f);
                    var result = Cv2.WaitKey(1);
                    if (result > -1) break;
                }
            }
            while (true)
            {
                using (var f = cam.GetFrame(-5))
                {
                Refind:
                    try
                    {
                        var rectp = RectTrack.DetectSkewRect(f);
                        cam.Exposure(-5);
                        Thread.Sleep(500);
                        foreach (var p in rectp.ContourCW)
                        {
                            Cv2.DrawMarker(f, p, Scalar.Red);
                            Cv2.ImShow("g", f);
                            Cv2.WaitKey(1);
                            MoveToTargetPoint(p);
                        }
                        MoveToTargetPoint(rectp.ContourCW.First());
                    }
                    catch
                    {
                        goto Refind;
                    }
                }
            }
        }
    }
}