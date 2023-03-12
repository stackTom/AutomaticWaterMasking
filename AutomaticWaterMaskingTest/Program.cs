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
            string viewPortXML = File.ReadAllText(outPath + @"\viewport.osm");
            Way<AutomaticWaterMasking.Point> viewPort = new List<Way<AutomaticWaterMasking.Point>>(AreaKMLFromOSMDataCreator.GetWays(viewPortXML, true).Values)[0];

            List<Way<AutomaticWaterMasking.Point>> coastWaterPolygons = new List<Way<AutomaticWaterMasking.Point>>();
            List<Way<AutomaticWaterMasking.Point>> inlandPolygons = new List<Way<AutomaticWaterMasking.Point>>();
            List<Way<AutomaticWaterMasking.Point>> inlandWater = new List<Way<AutomaticWaterMasking.Point>>();
            WaterMasking.CreatePolygons(coastWaterPolygons, inlandPolygons, inlandWater, coastXML, waterXML, new Way<AutomaticWaterMasking.Point>(viewPort));
            Bitmap bmp = WaterMasking.GetMask(outPath, 4096, 4096, viewPort[0], viewPort[2], coastWaterPolygons, inlandPolygons, inlandWater);
            bmp.Save(outPath + @"\img.bmp", System.Drawing.Imaging.ImageFormat.Bmp);
        }
    }
}
