using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Xml;

namespace AutomaticWaterMasking
{
    public class XYPair
    {
        public decimal X;
        public decimal Y;

        public XYPair(decimal x, decimal y)
        {
            this.X = x;
            this.Y = y;
        }

        public override string ToString()
        {
            return this.Y + ", " + this.X;
        }
    }

    public class Point : XYPair
    {
        public Point(decimal x, decimal y) : base(x, y) { }
    }

    public class Way<T> : System.Collections.Generic.List<T> where T : Point
    {
        public string relation;
        public string type;
        public string wayID;

        public Way() : base()
        {
        }
        public Way(Way<T> way) : base(way)
        {
        }

        public Way<T> MergePointToPoint(Way<T> way)
        {
            // closed Ways should never be merged point to point
            if (this.IsClosedWay() || way.IsClosedWay())
            {
                return null;
            }

            Point w1p1 = this[0];
            Point w1p2 = this[this.Count - 1];
            Point w2p1 = way[0];
            Point w2p2 = way[way.Count - 1];
            Way<T> newWay = new Way<T>(this);
            bool merged = false;

            if (w1p1.Equals(w2p1))
            {
                newWay.Reverse();
                for (int w2 = 1; w2 < way.Count; w2++)
                {
                    newWay.Add(way[w2]);
                }
                merged = true;
            }
            else if (w1p2.Equals(w2p2))
            {
                for (int w2 = way.Count - 2; w2 >= 0; w2--)
                {
                    newWay.Add(way[w2]);
                }
                merged = true;
            }
            else if (w1p2.Equals(w2p1))
            {
                for (int w2 = 1; w2 < way.Count; w2++)
                {
                    newWay.Add(way[w2]);
                }
                merged = true;
            }
            else if (w1p1.Equals(w2p2))
            {
                newWay.Reverse();
                for (int w2 = way.Count - 2; w2 >= 0; w2--)
                {
                    newWay.Add(way[w2]);
                }
                merged = true;
            }

            if (merged)
            {
                newWay.SetRelationAfterMerge(way);

                return newWay;
            }

            return null;
        }

        public void SetRelationAfterMerge(Way<T> way)
        {
            if (way.relation == "outer" || way.relation == "inner")
            {
                this.relation = way.relation;
            }
        }

        public bool IsClosedWay()
        {
            Point firstCoord = this[0];
            Point lastCoord = this[this.Count - 1];

            return firstCoord.X == lastCoord.X && firstCoord.Y == lastCoord.Y;
        }

        public override bool Equals(object obj)
        {
            Way<T> cmp = (Way<T>)obj;

            return this.wayID == cmp.wayID;
        }

        public override int GetHashCode()
        {
            return this.wayID.GetHashCode();
        }

        private class Edge
        {
            public Point p1;
            public Point p2;
            public decimal A;
            public decimal B;
            public decimal C;

            public Edge(Point p1, Point p2)
            {
                this.p1 = p1;
                this.p2 = p2;

                this.A = p2.Y - p1.Y;
                this.B = p1.X - p2.X;
                this.C= this.A * p1.X + this.B * p1.Y;
            }

            private decimal EPSILON = 0.000000000000000001m;
            public bool SafeLessThan(decimal d1, decimal d2)
            {
                return (d1 - d2) < -EPSILON;
            }

            public bool SafeGreaterThan(decimal d1, decimal d2)
            {
                return (d1 - d2) > EPSILON;
            }

            public bool PointInEdge(Point p)
            {
                decimal minX = Math.Min(this.p1.X, this.p2.X);
                decimal maxX = Math.Max(this.p1.X, this.p2.X);
                decimal minY = Math.Min(this.p1.Y, this.p2.Y);
                decimal maxY = Math.Max(this.p1.Y, this.p2.Y);

                if (SafeLessThan(p.X, minX) || SafeGreaterThan(p.X, maxX) || SafeLessThan(p.Y, minY) || SafeGreaterThan(p.Y, maxY))
                {
                    return false;
                }

                return true;
            }

            // returns point of intersection, null if no intersection
            public Point IntersectsWith(Edge e)
            {
                decimal delta = this.A * e.B - e.A * this.B;

                if (delta == 0)
                {
                    return null;
                }

                decimal x = (e.B * this.C - this.B * e.C) / delta;
                decimal y = (this.A * e.C - e.A * this.C) / delta;
                Point possibleIntersection = new Point(x, y);

                if (this.PointInEdge(possibleIntersection) && e.PointInEdge(possibleIntersection))
                {
                    return possibleIntersection;
                }

                return null;
            }

            public override string ToString()
            {
                return "(" + this.p1.ToString() + " " + this.p2.ToString() + ")";
            }
        }

        // returns point of intersection, null if no intersection
        public List<Point> IntersectsWith(Way<Point> check)
        {
            List<Point> intersections = new List<Point>();
            // TODO: optimize
            for (int i = 0; i < this.Count - 1; i++)
            {
                Edge thisEdge = new Edge(this[i], this[i + 1]);
                Edge checkEdge = null;

                for (int j = 0; j < check.Count - 1; j++)
                {
                    checkEdge = new Edge(check[j], check[j + 1]);
                    Point intersection = thisEdge.IntersectsWith(checkEdge);
                    if (intersection != null)
                    {
                        intersections.Add(intersection);
                    }
                }
            }

            return intersections.Count > 0 ? intersections : null;
        }
    }

    public class AreaKMLFromOSMDataCreator
    {
        private static Dictionary<string, Point> GetNodeIDsToCoords(XmlDocument d)
        {
            Dictionary<string, Point> nodeIDsToCoords = new Dictionary<string, Point>();
            XmlNodeList nodeTags = d.GetElementsByTagName("node");
            foreach (XmlElement node in nodeTags)
            {
                decimal lat = Convert.ToDecimal(node.GetAttribute("lat"));
                decimal lon = Convert.ToDecimal(node.GetAttribute("lon"));
                string id = node.GetAttribute("id");
                Point coords = new Point(lon, lat);
                nodeIDsToCoords.Add(id, coords);
            }

            return nodeIDsToCoords;
        }

        private static Dictionary<string, List<string>> GetWayIDsToWayNodes(XmlDocument d)
        {
            Dictionary<string, List<string>> wayIDsToWayNodes = new Dictionary<string, List<string>>();
            XmlNodeList wayTags = d.GetElementsByTagName("way");
            foreach (XmlElement way in wayTags)
            {
                string id = way.GetAttribute("id");

                List<string> nodes = new List<string>();
                XmlNodeList ndTags = way.GetElementsByTagName("nd");
                foreach (XmlElement nd in ndTags)
                {
                    string ndId = nd.GetAttribute("ref");
                    nodes.Add(ndId);
                }
                wayIDsToWayNodes.Add(id, nodes);
            }

            return wayIDsToWayNodes;
        }

        private static Dictionary<string, Way<Point>> GetWayIDsToWays(XmlDocument d, Dictionary<string, List<string>> wayIDsToWayNodes, Dictionary<string, Point> nodeIDsToCoords)
        {
            Dictionary<string, Way<Point>> wayIDsToways = new Dictionary<string, Way<Point>>();

            foreach (KeyValuePair<string, List<string>> kv in wayIDsToWayNodes)
            {
                string wayID = kv.Key;
                List<string> nodIDs = kv.Value;
                Way<Point> way = new Way<Point>();
                way.wayID = wayID;
                foreach (string id in nodIDs)
                {
                    Point coords = nodeIDsToCoords[id];
                    way.Add(coords);
                }
                wayIDsToways.Add(wayID, way);
            }

            return wayIDsToways;
        }

        private static List<string> GetWaysInThisMultipolygonAndUpdateRelations(XmlElement rel, Dictionary<string, string> wayIDsToRelation, Dictionary<string, string> wayIDsToType)
        {
            List<string> waysInThisMultipolygon = new List<string>();
            string type = null;
            foreach (XmlElement tag in rel.GetElementsByTagName("tag"))
            {
                if (tag.GetAttribute("k") == "water")
                {
                    type = tag.GetAttribute("v");
                }
            }

            foreach (XmlElement tag in rel.GetElementsByTagName("tag"))
            {
                if (tag.GetAttribute("v") == "multipolygon")
                {
                    foreach (XmlElement member in rel.GetElementsByTagName("member"))
                    {
                        string wayID = member.GetAttribute("ref");
                        waysInThisMultipolygon.Add(wayID);
                        string role = member.GetAttribute("role");
                        string curRole = null;
                        wayIDsToRelation.TryGetValue(wayID, out curRole);
                        wayIDsToType[wayID] = type;
                        // Bug in OSM data?
                        if (role == "" || role == " ")
                        {
                            continue;
                        }

                        if (curRole == null)
                        {
                            wayIDsToRelation.Add(wayID, role);
                        }
                        else if (curRole != role && role == "inner")
                        {
                            // set it to inner whether it's previous role was inner or outer. if it's inner, just treat it as land.
                            // TODO: what about water that is inner in another land... need to fix
                            wayIDsToRelation[wayID] = role;
                        }
                    }
                }
            }

            return waysInThisMultipolygon;
        }

        public static void MergeMultipolygonWays(List<string> waysInThisMultipolygon, Dictionary<string, Way<Point>> wayIDsToWays, string relationID, HashSet<Way<Point>> toDelete, HashSet<Way<Point>> toAdd)
        {
            Dictionary<string, Way<Point>> waysCopy = new Dictionary<string, Way<Point>>(wayIDsToWays);
            HashSet<Way<Point>> merged = new HashSet<Way<Point>>();
            bool mergeFound = false;
            do
            {
                mergeFound = false;
                for (int i = 0; i < waysInThisMultipolygon.Count; i++)
                {
                    for (int j = 0; j < waysInThisMultipolygon.Count; j++)
                    {
                        // i != j makes sure not comparing to the same way
                        if (i != j)
                        {
                            string way1id = waysInThisMultipolygon[i];
                            string way2id = waysInThisMultipolygon[j];
                            // make sure the way hasn't been removed due to being combined previously...
                            if (!waysCopy.ContainsKey(way1id) || !waysCopy.ContainsKey(way2id))
                            {
                                continue;
                            }
                            Way<Point> way1 = waysCopy[way1id];
                            Way<Point> way2 = waysCopy[way2id];

                            Way<Point> mergedWay = way1.MergePointToPoint(way2);
                            if (mergedWay != null)
                            {
                                mergedWay.wayID = way1id;
                                waysCopy[way1id] = mergedWay;
                                waysCopy.Remove(way2id);
                                if (!merged.Contains(mergedWay))
                                {
                                    merged.Add(mergedWay);
                                }
                                else
                                {
                                    merged.Remove(mergedWay);
                                    merged.Add(mergedWay);
                                }

                                if (!toDelete.Contains(way1))
                                {
                                    toDelete.Add(way1);
                                }

                                if (!toDelete.Contains(way2))
                                {
                                    toDelete.Add(way2);
                                }
                                mergeFound = true;
                                break;
                            }
                        }
                    }
                }
            } while (mergeFound);

            foreach (Way<Point> w in merged)
            {
                if (!toAdd.Contains(w))
                {
                    toAdd.Add(w);
                }
            }
        }

        public static Dictionary<string, Way<Point>> GetWays(string OSMKML, bool mergeMultipolygons)
        {
            XmlDocument d = new XmlDocument();
            d.LoadXml(OSMKML);

            Dictionary<string, List<string>> wayIDsToWayNodes = GetWayIDsToWayNodes(d);
            Dictionary<string, Point> nodeIDsToCoords = GetNodeIDsToCoords(d);
            Dictionary<string, Way<Point>> wayIDsToWays = GetWayIDsToWays(d, wayIDsToWayNodes, nodeIDsToCoords);
            // POSSIBLE BUG: can a way every be both inner and outer in a relationship?
            Dictionary<string, string> wayIDsToRelation = new Dictionary<string, string>();
            Dictionary<string, string> wayIDsToType = new Dictionary<string, string>();

            XmlNodeList relationTags = d.GetElementsByTagName("relation");
            HashSet<Way<Point>> toAdd = new HashSet<Way<Point>>();
            HashSet<Way<Point>> toDelete = new HashSet<Way<Point>>();
            foreach (XmlElement rel in relationTags)
            {
                // unite multipolygon pieces into one big linestring, since fset just looks at line. no point in doing a kml polygon
                // since fset seems to not understand outer vs inner relation. we need this because singular way lines of inner water are problematic
                // it is hard to determine direction the water is in relative to the way because I believe OSM only requires direction
                // for coastal ways. but if we make them a full polygon, then it is easy to determine that the water is inside the polygon
                // here we compare every way to every other way.
                List<string> waysInThisMultipolygon = GetWaysInThisMultipolygonAndUpdateRelations(rel, wayIDsToRelation, wayIDsToType);
                string relationID = rel.GetAttribute("id");
                // update relations
                foreach (KeyValuePair<string, Way<Point>> kv in wayIDsToWays)
                {
                    string wayID = kv.Key;
                    Way<Point> way = kv.Value;
                    way.relation = null;
                    way.wayID = wayID;
                    if (wayIDsToRelation.ContainsKey(wayID))
                    {
                        way.relation = wayIDsToRelation[wayID];
                        way.type = wayIDsToType[wayID];
                    }
                }

                if (mergeMultipolygons)
                {
                    MergeMultipolygonWays(waysInThisMultipolygon, wayIDsToWays, relationID, toDelete, toAdd);
                }
            }

            foreach (Way<Point> way in toDelete)
            {
                wayIDsToWays.Remove(way.wayID);
            }

            foreach (Way<Point> way in toAdd)
            {
                wayIDsToWays.Add(way.wayID, way);
            }

            return wayIDsToWays;
        }

    }
    public struct DownloadArea
    {
        public decimal startLon;
        public decimal endLon;
        public decimal startLat;
        public decimal endLat;

        public DownloadArea(decimal startLon, decimal endLon, decimal startLat, decimal endLat)
        {
            this.startLon = startLon;
            this.endLon = endLon;
            this.startLat = startLat;
            this.endLat = endLat;
        }

        public void addPadding(decimal padding)
        {
            this.startLat += padding;
            this.endLat -= padding;
            this.startLon -= padding;
            this.endLon += padding;
        }
    }

    public class WaterMasking
    {
        private static string[] overPassServers = {
            "http://overpass-api.de/api/interpreter",
            "http://api.openstreetmap.fr/oapi/interpreter",
            "https://overpass.kumi.systems/api/interpreter",
            "http://overpass.osm.rambler.ru/cgi/interpreter",
        };

        private static string DownloadOSM(string queryParams)
        {
            bool keepTrying = false;
            string contents = null;
            int sleepTime = 1;
            do
            {
                foreach (string server in overPassServers)
                {
                    using (var wc = new System.Net.WebClient())
                    {
                        try
                        {
                            // make sure to kill any zombie queries...
                            wc.DownloadString("http://overpass-api.de/api/kill_my_queries");

                            Console.WriteLine("Downloading OSM data using server: " + server + ". This might take a while. Please wait...");
                            Console.WriteLine(queryParams);
                            contents = wc.DownloadString(server + queryParams);
                            keepTrying = false;
                            break;
                        }
                        catch (System.Net.WebException)
                        {
                            Console.WriteLine("Download failed using " + server + "... trying new overpass server in " + sleepTime + " seconds");
                            keepTrying = true;
                            System.Threading.Thread.Sleep(sleepTime);
                        }
                    }
                }
                if (sleepTime < 32)
                {
                    sleepTime *= 2;
                }
            } while (keepTrying);

            return contents;
        }

        private static string DownloadOsmWaterData(DownloadArea d, string saveLoc)
        {
            string[] waterQueries = { "rel[\"natural\"=\"water\"]", "rel[\"waterway\"=\"riverbank\"]", "way[\"natural\"=\"water\"]", "way[\"waterway\"=\"riverbank\"]", "way[\"waterway\"=\"dock\"]" };
            string waterOSM = null;
            string queryParams = "?data=(";
            string bbox = "(" + d.endLat + ", " + d.startLon + ", " + d.startLat + ", " + d.endLon + ")";
            foreach (string query in waterQueries)
            {
                queryParams += query + bbox + ";";
            }
            queryParams = queryParams.Remove(queryParams.Length - 1, 1);
            queryParams += ";);(._;>>;);out body;";

            waterOSM = DownloadOSM(queryParams);
            File.WriteAllText(saveLoc, waterOSM);

            return waterOSM;
        }
        // http://overpass-api.de/api/interpreter?data=(way["natural"="coastline"](23, -83, 24, -82););(._;>>;);out meta;
        private static string DownloadOsmCoastData(DownloadArea d, string saveLoc)
        {
            string[] coastQueries = { "way[\"natural\"=\"coastline\"]" };
            string coastOSM = null;
            string queryParams = "?data=(";
            string bbox = "(" + d.endLat + ", " + d.startLon + ", " + d.startLat + ", " + d.endLon + ")";
            foreach (string query in coastQueries)
            {
                queryParams += query + bbox + ";";
            }
            queryParams = queryParams.Remove(queryParams.Length - 1, 1);
            queryParams += ";);(._;>>;);out body;";

            coastOSM = DownloadOSM(queryParams);
            File.WriteAllText(saveLoc, coastOSM);

            return coastOSM;
        }

        // TODO: implement
        private static List<Way<Point>> CoastWaysToPolygon(Dictionary<string, Way<Point>> coastWays)
        {
            List<Way<Point>> polygons = new List<Way<Point>>();

            return polygons;
        }

        public static List<Way<Point>> CreatePolygons(string coastXML, string waterXML)
        {
            Dictionary<string, Way<Point>> coastWays = AreaKMLFromOSMDataCreator.GetWays(coastXML, true);
            Dictionary<string, Way<Point>> waterWays = AreaKMLFromOSMDataCreator.GetWays(waterXML, true);
            MergeCoastLines(coastWays);
            List<Way<Point>> polygons = new List<Way<Point>>();

            //foreach (KeyValuePair<string, Way<Point>> kv in waterWays)
            foreach (KeyValuePair<string, Way<Point>> kv in coastWays)
            {
                polygons.Add(kv.Value);
            }

            for (int i = 0; i < polygons.Count; i++)
            {
                Way<Point> p = polygons[i];
                List<Point> intersection = p.IntersectsWith(box);
            }
            List<Way<Point>> coastPolygons = CoastWaysToPolygon(coastWays);
            foreach (Way<Point> way in coastPolygons)
            {
                polygons.Add(way);
            }


            return polygons;
        }

        // TODO: the way I merge lines that aren't part of a multipolygon is a hack. FIXME
        private static void MergeCoastLines(Dictionary<string, Way<Point>> coastWays)
        {
            List<string> mergeMe = new List<string>(coastWays.Keys);
            HashSet<Way<Point>> toAdd = new HashSet<Way<Point>>();
            HashSet<Way<Point>> toDelete = new HashSet<Way<Point>>();
            AreaKMLFromOSMDataCreator.MergeMultipolygonWays(mergeMe, coastWays, null, toDelete, toAdd);

            foreach (Way<Point> way in toDelete)
            {
                coastWays.Remove(way.wayID);
            }

            foreach (Way<Point> way in toAdd)
            {
                coastWays.Add(way.wayID, way);
            }
        }

        public static List<Way<Point>> GetPolygons(DownloadArea d, string saveLoc)
        {
            string coastXML = DownloadOsmCoastData(d, saveLoc + @"\coast.xml");
            string waterXML = DownloadOsmWaterData(d, saveLoc + @"\water.xml");

            List<Way<Point>> polygons = CreatePolygons(coastXML, waterXML);


            return polygons;
        }

        public struct tXYCoord
        {
            public decimal mX;
            public decimal mY;
        }

        public static tXYCoord ConvertXYLatLongToPixel(tXYCoord iXYCoord, decimal startLat, decimal startLong, decimal vPixelPerLongitude, decimal vPixelPerLatitude)
        {
            tXYCoord vPixelXYCoord;

            vPixelXYCoord.mX = vPixelPerLongitude * (iXYCoord.mX - startLong);
            vPixelXYCoord.mY = -vPixelPerLatitude * (startLat - iXYCoord.mY);

            return vPixelXYCoord;
        }
        public static tXYCoord CoordToPixel(decimal lat, decimal longi, decimal mAreaNWCornerLatitude, decimal mAreaNWCornerLongitude, decimal vPixelPerLongitude,
                                    decimal vPixelPerLatitude)
        {
            tXYCoord tempCoord;
            tempCoord.mX = longi;
            tempCoord.mY = lat;
            tXYCoord pixel = ConvertXYLatLongToPixel(tempCoord, mAreaNWCornerLatitude, mAreaNWCornerLongitude, vPixelPerLongitude, vPixelPerLatitude);
            pixel.mX -= 0.5m;
            pixel.mY -= 0.5m;

            return pixel;
        }

        public static Bitmap GetMask(string outPath, int width, int height, Point NW, Point SE, List<Way<Point>> polygons)
        {
            Bitmap bmp = new Bitmap(width, height, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            decimal pixelsPerLon = -Convert.ToDecimal(width) / (NW.X - SE.X);
            decimal pixelsPerLat = Convert.ToDecimal(height) / (NW.Y - SE.Y);

            using (Graphics g = Graphics.FromImage(bmp))
            {
                SolidBrush b = new SolidBrush(Color.White);
                g.FillRectangle(Brushes.Black, 0, 0, bmp.Width, bmp.Height);
                foreach (Way<Point> way in polygons)
                {

                    List<PointF> l = new List<PointF>();
                    foreach (Point p in way)
                    {
                        tXYCoord pixel = CoordToPixel(p.Y, p.X, NW.Y, NW.X, pixelsPerLon, pixelsPerLat);

                        l.Add(new PointF((float)pixel.mX, (float)pixel.mY));
                    }
                    PointF[] pf = l.ToArray();
                    g.FillPolygon(b, pf);

                }
            }

            return bmp;
        }
    }
}
