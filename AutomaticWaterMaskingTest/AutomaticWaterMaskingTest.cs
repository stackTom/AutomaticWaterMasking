using AutomaticWaterMasking;
using System;
using System.IO;
using System.Collections.Generic;

public class AutomaticWaterMaskingTest
{
    public static void Main(string[] args)
    {
        string[] cmdArgs = Environment.GetCommandLineArgs();
        string outPath = cmdArgs[3];
        int startAt = Int32.Parse(cmdArgs[1]);
        for (int startLat = Int32.Parse(cmdArgs[2]); startLat < 90; startLat++)
        {
            for (int startLon = startAt; startLon < 180; startLon++)
            {
                List<Way<AutomaticWaterMasking.Point>> coastWaterPolygons = new List<Way<AutomaticWaterMasking.Point>>();
                List<Way<AutomaticWaterMasking.Point>> inlandPolygons = new List<Way<Point>>();
                List<Way<Point>> islands = new List<Way<Point>>();

                Console.WriteLine("Creating " + startLat.ToString() + ", " + startLon.ToString());
                DownloadArea d = new DownloadArea(startLon, startLon + 1, startLat + 1, startLat);
                Way<AutomaticWaterMasking.Point> viewPort = new Way<AutomaticWaterMasking.Point>();
                viewPort.Add(new AutomaticWaterMasking.Point((decimal)startLon, (decimal)(startLat + 1)));
                viewPort.Add(new AutomaticWaterMasking.Point((decimal)startLon + 1, (decimal)(startLat + 1)));
                viewPort.Add(new AutomaticWaterMasking.Point((decimal)(startLon + 1), (decimal)startLat));
                viewPort.Add(new AutomaticWaterMasking.Point((decimal)startLon, (decimal)startLat));
                viewPort.Add(new AutomaticWaterMasking.Point((decimal)startLon, (decimal)(startLat + 1)));

                string coastXML = WaterMasking.DownloadOsmCoastData(d, outPath + Path.DirectorySeparatorChar + "coast.osm");
                try
                {
                    WaterMasking.CreatePolygons(coastWaterPolygons, islands, inlandPolygons, coastXML, null, viewPort);
                    string coastWater = OSMXMLParser.WaysToOSMXML(coastWaterPolygons);
                    File.WriteAllText(outPath + Path.DirectorySeparatorChar + "coastWaterPolys.osm", coastWater);
                }
                catch (Exception e)
                {
                    Console.WriteLine("failed " + startLat.ToString() + ", " + startLon.ToString());
                    Console.WriteLine(e.ToString());
                    return;
                }
            }
            startAt = -180;
        }
    }
}
