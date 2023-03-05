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
            string coastXML = File.ReadAllText(outPath + @"\coast.osm");
            string waterXML = File.ReadAllText(outPath + @"\water.osm");
            DownloadArea d = new DownloadArea(-80.1238138888889m, -80.1195222222222m, 26.1076277777778m, 26.1075555555556m);
            Way<AutomaticWaterMasking.Point> viewPort = new Way<AutomaticWaterMasking.Point>();
            viewPort.Add(new AutomaticWaterMasking.Point(d.startLon, d.startLat));
            viewPort.Add(new AutomaticWaterMasking.Point(d.endLon, d.startLat));
            viewPort.Add(new AutomaticWaterMasking.Point(d.endLon, d.endLat));
            viewPort.Add(new AutomaticWaterMasking.Point(d.startLon, d.endLat));
            viewPort.Add(new AutomaticWaterMasking.Point(d.startLon, d.startLat));
            string xml = viewPort.ToOSMXML();
            File.WriteAllText(@"C:\users\fery2\Desktop\VIEWPORT.osm", xml);
            List<Way<AutomaticWaterMasking.Point>> polygons = WaterMasking.CreatePolygons(coastXML, waterXML, viewPort);
            return;
            Bitmap bmp = WaterMasking.GetMask(outPath, 4096, 4096, new AutomaticWaterMasking.Point(26.1075555555556m, -80.1195222222222m), new AutomaticWaterMasking.Point(26.1076277777778m, -80.1238138888889m), polygons);
            bmp.Save(outPath + @"\img.bmp");
        }
    }
}
