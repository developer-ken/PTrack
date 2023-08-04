using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PTrack
{
    public class FreeRollingCamera : IDisposable
    {
        VideoCapture vcap;
        ManualResetEvent onCap = new ManualResetEvent(false);
        Mat latestframe;
        bool run = true;
        bool count_fps;
        public double RollingRate { get; private set; }
        private int frames;

        public FreeRollingCamera(int dev, bool fpscounter = false)
        {
            count_fps = fpscounter;
            vcap = new VideoCapture(dev);
            vcap.AutoExposure = 0.25;
            latestframe = new Mat();
        }
        public FreeRollingCamera(string dev, bool fpscounter = false)
        {
            count_fps = fpscounter;
            vcap = new VideoCapture(dev);
            vcap.AutoExposure = 0.25;
            latestframe = new Mat();
        }

        public void BeginRoll()
        {
            Task.Run(() =>
            {
                if (count_fps)
                {
                    frames = 0;
                    while (run)
                    {
                        vcap.Read(latestframe);
                        if (latestframe.Empty())
                        {
                            continue;
                        }
                        onCap.Set();
                        frames++;
                    }
                }
                else
                    while (run)
                    {
                        vcap.Read(latestframe);
                        if (latestframe.Empty())
                        {
                            continue;
                        }
                        onCap.Set();
                    }
            });
            if (count_fps)
            {
                Task.Run(() =>
                {
                    DateTime time = DateTime.Now;
                    while (run)
                    {
                        Thread.Sleep(500);
                        RollingRate = frames / (DateTime.Now - time).TotalSeconds;
                        time = DateTime.Now;
                        frames = 0;
                    }
                });
            }
        }

        public void Dispose()
        {
            run = false;
            latestframe.Dispose();
        }

        public void Exposure(double exposure = 0)
        {
            //vcap.Exposure = exposure;
        }

        public MatInPool GetFrame(double exposure = 0)
        {
            //vcap.Exposure = exposure;

            onCap.WaitOne();
            onCap.Reset();
            //Cv2.Flip(latestframe, latestframe, FlipMode.XY);
            int size = Math.Min(latestframe.Height, latestframe.Width);
            Point center = new Point(latestframe.Width / 2, latestframe.Height / 2);
            Rect roi = new Rect(center.X - size / 2, center.Y - size / 2, size, size);
            var cropped = latestframe[roi];

            return MatPool.Default.EnPool(cropped);
        }
    }
}
