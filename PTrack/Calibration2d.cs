using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;

namespace PTrack
{
    public class Calibration2d
    {
        private Dictionary<(double, double), (double, double)> _calibrationData;

        public Calibration2d(Dictionary<(double, double), (double, double)> calibrationData)
        {
            _calibrationData = calibrationData;
        }

        public Calibration2d()
        {
            _calibrationData = new Dictionary<(double, double), (double, double)>();
        }

        public string Serialize()
        {
            StringBuilder sb = new StringBuilder();
            foreach (KeyValuePair<(double, double), (double, double)> pair in _calibrationData)
            {
                sb.Append($"{pair.Key.Item1},{pair.Key.Item2}>{pair.Value.Item1},{pair.Value.Item2};");
            }
            return sb.ToString();
        }

        public void Deserialize(string serializedData)
        {
            var items = serializedData.Split(';');
            foreach (var item in items)
            {
                if (item.Length < 2) continue;
                var points = item.Split('>');
                var detectedPoint = points[0].Split(',');
                var realPoint = points[1].Split(',');
                AddCalibrationPoint((double.Parse(detectedPoint[0]), double.Parse(detectedPoint[1])), (double.Parse(realPoint[0]), double.Parse(realPoint[1])));
            }
        }

        public void AddCalibrationPoint((double, double) detectedPoint, (double, double) realPoint)
        {
            try
            {
                _calibrationData.Add(detectedPoint, realPoint);
            }
            catch { }
        }

        public (double, double) MapCoordinates((double, double) detectedPoint)
        {
            var nearestPoints = _calibrationData.OrderBy(p => Distance(p.Key, detectedPoint)).Take(4);

            double sumX = 0, sumY = 0, sumWeight = 0;

            foreach (var point in nearestPoints)
            {
                var dist = Distance(point.Key, detectedPoint);

                var weight = 1 / (1 + dist);

                sumX += point.Value.Item1 * weight;
                sumY += point.Value.Item2 * weight;
                sumWeight += weight;
            }

            var resultX = sumX / sumWeight;
            var resultY = sumY / sumWeight;

            return (resultX, resultY);
        }

        private static double Distance((double, double) p1, (double, double) p2)
        {
            var dx = p1.Item1 - p2.Item1;
            var dy = p1.Item2 - p2.Item2;

            return dx * dx + dy * dy;
        }
    }
}