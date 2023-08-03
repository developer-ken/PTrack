using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Data.Common;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PTrack
{
    public class RectTrack
    {
        public static Rect4P DetectSkewRect(Mat image)
        {
            using (Mat gray = GrayIsh(image))
            using (Mat imgbin = new Mat())
            using (Mat edges = new Mat())
            {
                Cv2.EqualizeHist(gray, gray);
                Cv2.Threshold(gray, imgbin, 30, 255, ThresholdTypes.BinaryInv); // 二值化
                Cv2.Erode(imgbin, imgbin, new Mat());
                Cv2.Erode(imgbin, imgbin, new Mat());
                Cv2.ImShow("bin", imgbin);
                Cv2.WaitKey(1);
                Cv2.FindContours(imgbin, out Point[][] contours, out HierarchyIndex[] hierarchy, RetrievalModes.Tree, ContourApproximationModes.ApproxSimple);
                Cv2.DrawContours(image, contours, -1, Scalar.Yellow, 1);
                double maxScore = 0;
                double mastersize = 0;
                bool t = false;
                Rect4P bx = new Rect4P();
                int maxbxid = -1;
                int ijj = -1;
                foreach (var contour in contours)
                {
                    ijj++;
                    double area = Cv2.ContourArea(contour); // 面积
                    RotatedRect rect = Cv2.MinAreaRect(contour); // 最小外接矩形
                    double dist = Distance((Point)rect.Center - new Point(image.Width / 2, image.Height / 2));
                    double score = area + dist * 5;

                    if (hierarchy[ijj].Child < 0) continue; //没有内框
                    if (hierarchy[hierarchy[ijj].Child].Next > 0) continue; //太多内框
                    if (hierarchy[hierarchy[ijj].Child].Previous > 0) continue; //太少内框

                    var bbx = Cv2.BoxPoints(rect);
                    if (score > maxScore)
                    {
                        maxbxid = ijj;
                        mastersize = area;
                        bx.LeftUp = (Point)bbx[3];
                        bx.LeftDown = (Point)bbx[2];
                        bx.RightDown = (Point)bbx[1];
                        bx.RightUp = (Point)bbx[0];
                        t = true;
                        maxScore = score; // 过滤最高分矩形
                    }
                }
                if (t)
                {
                    Cv2.DrawContours(image, new Point[][] { bx.ContourCW }, -1, Scalar.Green, 1);
                    var child = contours[hierarchy[maxbxid].Child];
                    RotatedRect rect = Cv2.MinAreaRect(child); // 最小外接矩形
                    var bbx = Cv2.BoxPoints(rect);
                    Rect4P aa = new Rect4P();
                    aa.LeftUp = (Point)bbx[3];
                    aa.LeftDown = (Point)bbx[2];
                    aa.RightDown = (Point)bbx[1];
                    aa.RightUp = (Point)bbx[0];
                    PointEnhance(aa, child);
                    Cv2.DrawContours(image, new Point[][] { aa.ContourCW }, -1, Scalar.Red, 1);

                    bx.LeftUp = Devide((bx.LeftUp + aa.LeftUp), 2);
                    bx.LeftDown = Devide((bx.LeftDown + aa.LeftDown), 2);
                    bx.RightDown = Devide((bx.RightDown + aa.RightDown), 2);
                    bx.RightUp = Devide((bx.RightUp + aa.RightUp), 2);
                    return bx;
                }
                else
                {
                    throw new Exception("No Target Found");
                }
            }
        }

        public static Point Devide(Point p, int i)
        {
            return new Point(p.X / i, p.Y / i);
        }

        public static MatInPool GrayIsh(Mat input)
        {
            MatInPool result = MatPool.Default.Borrow(input.Size(), MatType.CV_8UC1);//new Mat(input.Size(), MatType.CV_8UC1);
            for (int y = 0; y < input.Rows; y++)
            {
                for (int x = 0; x < input.Cols; x++)
                {
                    var bgr = input.Get<Vec3b>(y, x);
                    byte max = Math.Max(bgr.Item0, Math.Max(bgr.Item1, bgr.Item2));
                    result.Set(y, x, new Vec3b(max, max, max));
                }
            }
            return result;
        }

        public static Rect4P PointEnhance(Rect4P input, Point[] contour)
        {
            Rect4P result = new Rect4P();
            Cv2.ApproxPolyDP(contour, Distance(input.LeftUp - input.RightDown) / 50f, false);
            result.LeftDown = ClosestTo(input.LeftDown, contour);
            result.LeftUp = ClosestTo(input.LeftUp, contour);
            result.RightUp = ClosestTo(input.RightUp, contour);
            result.RightDown = ClosestTo(input.RightDown, contour);
            return result;
        }

        public static Point ClosestTo(Point ref_, Point[] points)
        {
            double mindist = double.MaxValue;
            Point result = points[0];
            foreach(Point p in points)
            {
                var dist = Distance(ref_ - p);
                if(mindist > dist)
                {
                    mindist = dist;
                    result = p;
                }
            }
            return result;
        }

        public static MatInPool RedMinusBG(Mat input)
        {
            var mmap = input.Split();
            var r = mmap[2];
            var g = mmap[1];
            var b = mmap[0];
            Cv2.Multiply(b, 0.5, b);
            Cv2.Multiply(g, 0.5, g);
            Cv2.Subtract(r, b, r);
            Cv2.Subtract(r, g, r);
            var chr = MatPool.Default.EnPool(r);
            r.Dispose();
            g.Dispose();
            b.Dispose();
            return chr;
        }

        public static float Distance(Point p)
        {
            return (float)Math.Sqrt(p.X * p.X + p.Y * p.Y);
        }

        public struct Rect4P
        {
            public Point LeftUp, RightUp, LeftDown, RightDown;
            public Point[] ContourCW => new Point[] { LeftUp, RightUp, RightDown, LeftDown };
            public Point[] ContourCCW => new Point[] { LeftUp, LeftDown, RightDown, RightUp };
        }
    }
}
