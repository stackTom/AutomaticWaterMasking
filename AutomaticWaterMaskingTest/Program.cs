using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using AutomaticWaterMasking;

namespace AutomaticWaterMaskingTest
{
    class Program
    {
        static void Main(string[] args)
        {
            string outPath = Environment.GetCommandLineArgs()[1];
            string coastXML = File.ReadAllText(outPath + @"\coast.xml");
            string waterXML = File.ReadAllText(outPath + @"\water.xml");
            List<Way<AutomaticWaterMasking.Point>> polygons = WaterMasking.CreatePolygons(coastXML, waterXML);
            Bitmap bmp = WaterMasking.GetMask(outPath, 4096, 4096, new AutomaticWaterMasking.Point(26.1075555555556, -80.1195222222222), new AutomaticWaterMasking.Point(26.1076277777778, -80.1238138888889), polygons);
            bmp.Save(outPath + @"\img.bmp");
        }
    }
}
