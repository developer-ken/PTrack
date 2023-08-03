using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PTrack
{
    public static class CVUtil
    {
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

        public static MatInPool GreenMinusRG(Mat input)
        {
            var mmap = input.Split();
            var r = mmap[2];
            var g = mmap[1];
            var b = mmap[0];
            Cv2.Multiply(b, 0.5, b);
            Cv2.Multiply(r, 0.5, r);
            Cv2.Subtract(g, b, g);
            Cv2.Subtract(g, r, g);
            var chr = MatPool.Default.EnPool(g);
            r.Dispose();
            g.Dispose();
            b.Dispose();
            return chr;
        }
    }
}
