using System;
using System.Collections.Generic;
using AutomaticWaterMasking;

namespace AutomaticWaterMaskingTest
{
    class Program
    {
        static void Main(string[] args)
        {
            DownloadArea d = new DownloadArea(-80.1238138888889, -80.1195222222222, 26.1076277777778, 26.1075555555556);
            List<Way<Point>> polygons = WaterMasking.GetPolygons(d, Environment.GetCommandLineArgs()[1]);
        }
    }
}
