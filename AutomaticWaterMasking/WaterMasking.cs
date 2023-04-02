using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Xml;
using System.Linq;

namespace AutomaticWaterMasking
{
    public static class SafeCompare
    {
        private static decimal EPSILON = 0.00000000000001m;
        public static bool SafeLessThan(decimal d1, decimal d2)
        {
            return (d1 - d2) < -EPSILON;
        }

        public static bool SafeGreaterThan(decimal d1, decimal d2)
        {
            return (d1 - d2) > EPSILON;
        }

        public static bool SafeEquals(decimal d1, decimal d2)
        {
            return Math.Abs(d1 - d2) < EPSILON;
        }

    }

    public class XYPair
    {
        // we use decimals's because using double's leads to floating point errors (sometimes even architecture dependent - ie works on M1 mac but not on x64 windows...)
        public decimal X;
        public decimal Y;
        private const int ROUND_TO_DIGITS = 14;

        public XYPair(decimal x, decimal y)
        {
            this.X = x;
            this.Y = y;
        }

        public override string ToString()
        {
            // G29 basically trims trailing 0's
            return Decimal.Round(this.Y, ROUND_TO_DIGITS).ToString("G29") + ", " + Decimal.Round(this.X, ROUND_TO_DIGITS).ToString("G29");
        }

        public override bool Equals(object obj)
        {
            return obj is XYPair pair &&
                   X == pair.X &&
                   Y == pair.Y;
        }

        public override int GetHashCode()
        {
            return this.ToString().GetHashCode();
        }
    }

    public class Point : XYPair
    {
        public Point(decimal x, decimal y) : base(x, y) { }
        public override int GetHashCode()
        {
            return base.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            return obj is Point pair &&
                   SafeCompare.SafeEquals(X, pair.X) &&
                   SafeCompare.SafeEquals(Y, pair.Y);
        }

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
                for (int w2 = way.Count - 2; w2 >= 0; w2--)
                {
                    newWay.Insert(0, way[w2]);
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
            if (this.Count < 4)
            {
                return false;
            }

            Point firstCoord = this[0];
            Point lastCoord = this[this.Count - 1];

            return firstCoord.Equals(lastCoord);
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

        public void InsertPointAtIndex(Point p, int idx)
        {
            if (idx <= 0)
            {
                return;
            }

            Edge curEdge = new Edge(this[idx - 1], this[idx]);
            Edge newEdge = new Edge(p, this[idx]);

            if (curEdge.GetDirection() == newEdge.GetDirection())
            {
                this.Insert(idx, (T)p);
            }
            else
            {
                // add it one point ahead
                this.Insert(idx + 1, (T)p);
            }
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
                this.C = this.A * p1.X + this.B * p1.Y;
            }


            public bool PointInEdge(Point p)
            {
                decimal minX = Math.Min(this.p1.X, this.p2.X);
                decimal maxX = Math.Max(this.p1.X, this.p2.X);
                decimal minY = Math.Min(this.p1.Y, this.p2.Y);
                decimal maxY = Math.Max(this.p1.Y, this.p2.Y);

                if (SafeCompare.SafeLessThan(p.X, minX) || SafeCompare.SafeGreaterThan(p.X, maxX) || SafeCompare.SafeLessThan(p.Y, minY) || SafeCompare.SafeGreaterThan(p.Y, maxY))
                {
                    return false;
                }

                return true;
            }

            // returns point of intersection, null if no intersection
            // didn't want to do high school math, so thanks to
            // https://stackoverflow.com/questions/4543506/algorithm-for-intersection-of-2-lines
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

            public enum Direction
            {
                NorthToSouth,
                SouthToNorth,
                EastToWest,
                WestToEast,
                SWToNE,
                NEToSW,
                SEToNW,
                NWToSE
            }

            public Direction GetDirection()
            {
                // equal lon (x)
                if (SafeCompare.SafeEquals(this.p1.X, this.p2.X))
                {
                    // going from north to south, water is on west
                    if (SafeCompare.SafeGreaterThan(this.p1.Y, this.p2.Y))
                    {
                        return Direction.NorthToSouth;
                    }
                    // going from south to north, water is on east
                    return Direction.SouthToNorth;
                }
                // equal lat (y)
                if (SafeCompare.SafeEquals(this.p1.Y, this.p2.Y))
                {
                    // going from east to west, water is on north
                    if (SafeCompare.SafeGreaterThan(this.p1.X, this.p2.X))
                    {
                        return Direction.EastToWest;
                    }
                    // going from west to east, water is on south
                    return Direction.WestToEast;
                }
                // neither lat nor lon are equal, look at the slope
                decimal dx = this.p2.X - this.p1.X;
                decimal dy = this.p2.Y - this.p1.Y;
                if (SafeCompare.SafeGreaterThan((dy / dx), 0))
                {
                    if (SafeCompare.SafeGreaterThan(dy, 0))
                    {
                        // going from southwest to northeast, water is on southeast
                        return Direction.SWToNE;
                    }
                    // going from northeast to southwest, water is on northwest
                    return Direction.NEToSW;
                }
                // at this point slope < 0. should never be 0 because lat's can't equal by this point
                if (SafeCompare.SafeGreaterThan(dy, 0))
                {
                    // going from southeast to northwest, water is on northeast
                    return Direction.SEToNW;
                }

                // going from northwest to southeast, water is on southwest
                return Direction.NWToSE;
            }
        }

        // returns point of intersection, null if no intersection
        public List<Point> IntersectsWith(Way<Point> check, bool insertIntersectionIntoWay, bool insertIntersectionIntoComparisonWay)
        {
            List<Point> intersections = new List<Point>();
            // TODO: optimize
            for (int i = 0; i < this.Count - 1; i++)
            {
                Edge thisEdge = new Edge(this[i], this[i + 1]);
                Edge checkEdge = null;

                int intersectionsAddedToThis = 0;
                for (int j = 0; j < check.Count - 1; j++)
                {
                    checkEdge = new Edge(check[j], check[j + 1]);
                    Point intersection = thisEdge.IntersectsWith(checkEdge);
                    if (intersection != null)
                    {
                        if (!intersections.Contains(intersection))
                        {
                            intersections.Add(intersection);
                        }
                        if (insertIntersectionIntoWay && !this.Contains(intersection))
                        {
                            this.InsertPointAtIndex(intersection, i + 1);
                            intersectionsAddedToThis++;
                        }
                        if (insertIntersectionIntoComparisonWay && !check.Contains(intersection))
                        {
                            check.InsertPointAtIndex(intersection, j + 1);
                            j++;
                        }
                    }
                }
                if (intersectionsAddedToThis > 0)
                {
                    i += intersectionsAddedToThis;
                }
            }

            return intersections.Count > 0 ? intersections : null;
        }

        public static void AddMissingOSMAttributes(XmlElement nodeEle)
        {
            nodeEle.SetAttribute("uid", "1");
            nodeEle.SetAttribute("changeset", "1");
            nodeEle.SetAttribute("version", "1");
            nodeEle.SetAttribute("timestamp", "2000-01-01T00:00:00Z");
            nodeEle.SetAttribute("user", "AutomaticWaterMasker");
        }

        public XmlElement CreateOSMXMLWayNodeAndNDEles(XmlDocument d, XmlElement osmEle, string wayID, int startIdx)
        {
            XmlElement wayEle = d.CreateElement(string.Empty, "way", string.Empty);
            wayEle.SetAttribute("id", wayID);
            AddMissingOSMAttributes(wayEle);

            foreach (Point p in this)
            {
                XmlElement nodeEle = d.CreateElement(string.Empty, "node", string.Empty);
                nodeEle.SetAttribute("id", startIdx.ToString());
                nodeEle.SetAttribute("lat", p.Y.ToString());
                nodeEle.SetAttribute("lon", p.X.ToString());
                AddMissingOSMAttributes(nodeEle);
                osmEle.AppendChild(nodeEle);

                XmlElement ndEle = d.CreateElement(string.Empty, "nd", string.Empty);
                ndEle.SetAttribute("ref", startIdx.ToString());
                wayEle.AppendChild(ndEle);
                startIdx++;
            }

            return wayEle;
        }


        public string ToOSMXML()
        {
            XmlDocument d = new XmlDocument();
            XmlDeclaration xmlDeclaration = d.CreateXmlDeclaration("1.0", "UTF-8", null);
            XmlElement root = d.DocumentElement;
            d.InsertBefore(xmlDeclaration, root);
            XmlElement osmEle = d.CreateElement(string.Empty, "osm", string.Empty);
            d.AppendChild(osmEle);
            osmEle.SetAttribute("version", "0.6");
            osmEle.SetAttribute("generator", "AutomaticWaterMasking");

            XmlElement metaEle = d.CreateElement(string.Empty, "meta", string.Empty);
            osmEle.AppendChild(metaEle);
            int i = 1; // must start at 1 or get error in JOSM...
            XmlElement wayEle = CreateOSMXMLWayNodeAndNDEles(d, osmEle, this.wayID != null ? this.wayID : "1", i);

            osmEle.AppendChild(wayEle);

            return d.OuterXml;
        }
    }

    public class OSMXMLParser
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
                string lastNodeID = "";
                foreach (string id in nodIDs)
                {
                    // OSM sometimes has ways which have the same point repeated multiple times, right after each other, unnecessarily
                    // This breaks the coast water polys generation logic for obvious reasons. This if statement prevents these repetitive points
                    // from being incorporated into the ways... only 1 instance of the point will be incorporated
                    if (id != lastNodeID)
                    {
                        Point coords = nodeIDsToCoords[id];
                        way.Add(coords);
                    }
                    lastNodeID = id;
                }
                wayIDsToways.Add(wayID, way);
            }

            return wayIDsToways;
        }

        private static List<Way<Point>> GetWaysInThisMultipolygonAndUpdateRelations(Dictionary<string, Way<Point>> wayIDsToWays, XmlElement rel, Dictionary<string, string> wayIDsToRelation, Dictionary<string, string> wayIDsToType)
        {
            List<Way<Point>> waysInThisMultipolygon = new List<Way<Point>>();
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
                        Way<Point> way = null;
                        if (wayIDsToWays.TryGetValue(wayID, out way))
                        {
                            waysInThisMultipolygon.Add(way);
                        }
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
                            wayIDsToRelation[wayID] = role;
                        }
                    }
                }
            }

            return waysInThisMultipolygon;
        }

        public static void MergeMultipolygonWays(List<Way<Point>> waysInThisMultipolygon)
        {
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
                            Way<Point> way1 = waysInThisMultipolygon[i];
                            Way<Point> way2 = waysInThisMultipolygon[j];

                            Way<Point> mergedWay = way1.MergePointToPoint(way2);
                            if (mergedWay != null)
                            {
                                mergedWay.wayID = way1.wayID;
                                waysInThisMultipolygon.Add(mergedWay);
                                waysInThisMultipolygon.Remove(way1);
                                waysInThisMultipolygon.Remove(way2);
                                mergeFound = true;
                                break;
                            }
                        }
                    }
                }
            } while (mergeFound);
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
                // unite multipolygon pieces into one big linestring
                // we need this because singular way lines of inner water are problematic
                // it is hard to determine direction the water is in relative to the way because I believe OSM only requires direction
                // for coastal ways. but if we make them a full polygon, then it is easy to determine that the water is inside the polygon
                // here we compare every way to every other way.
                List<Way<Point>> waysInThisMultipolygon = GetWaysInThisMultipolygonAndUpdateRelations(wayIDsToWays, rel, wayIDsToRelation, wayIDsToType);
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
                    // remove all the ones in the multipolygon and only re add them after they have been merged
                    foreach (Way<Point> way in waysInThisMultipolygon)
                    {
                        wayIDsToWays.Remove(way.wayID);
                    }
                    MergeMultipolygonWays(waysInThisMultipolygon);
                    foreach (Way<Point> way in waysInThisMultipolygon)
                    {
                        wayIDsToWays.Add(way.wayID, way);
                    }
                }
            }

            return wayIDsToWays;
        }

        public static string WaysToOSMXML(List<Way<Point>> ways)
        {
            XmlDocument d = new XmlDocument();
            XmlDeclaration xmlDeclaration = d.CreateXmlDeclaration("1.0", "UTF-8", null);
            XmlElement root = d.DocumentElement;
            d.InsertBefore(xmlDeclaration, root);
            XmlElement osmEle = d.CreateElement(string.Empty, "osm", string.Empty);
            d.AppendChild(osmEle);
            osmEle.SetAttribute("version", "0.6");
            osmEle.SetAttribute("generator", "AutomaticWaterMasking");

            XmlElement metaEle = d.CreateElement(string.Empty, "meta", string.Empty);
            osmEle.AppendChild(metaEle);
            int i = 1; // must start at 1 or get error in JOSM...
            foreach (Way<Point> way in ways)
            {
                XmlElement wayEle = way.CreateOSMXMLWayNodeAndNDEles(d, osmEle, i.ToString(), i);
                osmEle.AppendChild(wayEle);
                i += way.Count;
            }

            return d.OuterXml;
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

        // sets missing values from OSM data to make a valid OSM file that JOSM can load
        private static string FixOSM(string OSM)
        {
            XmlDocument d = new XmlDocument();
            d.LoadXml(OSM);
            XmlNodeList nodeTags = d.GetElementsByTagName("node");
            foreach (XmlElement node in nodeTags)
            {
                Way<Point>.AddMissingOSMAttributes(node);
            }
            nodeTags = d.GetElementsByTagName("way");
            foreach (XmlElement node in nodeTags)
            {
                Way<Point>.AddMissingOSMAttributes(node);
            }

            return d.OuterXml;
        }

        private static string DownloadOSM(string queryParams)
        {
            bool keepTrying = false;
            string contents = null;
            int sleepTime = 1; // in seconds
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
                            System.Threading.Thread.Sleep(sleepTime * 1000);
                        }
                    }
                }
                if (sleepTime < 32)
                {
                    sleepTime *= 2;
                }
            } while (keepTrying);

            return FixOSM(contents);
        }

        public static string DownloadOsmWaterData(DownloadArea d, string saveLoc)
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
        public static string DownloadOsmCoastData(DownloadArea d, string saveLoc)
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

        private static void PopulatePointToWaysDict(Dictionary<Point, List<Way<Point>>> dict, Way<Point> way)
        {
            foreach (Point p in way)
            {
                if (dict.ContainsKey(p))
                {
                    dict[p].Add(way);
                }
                else
                {
                    List<Way<Point>> l = new List<Way<Point>>();
                    l.Add(way);
                    dict.Add(p, l);
                }
            }
        }

        private static bool PointInViewport(Point p, Way<Point> viewPort)
        {
            Way<Point> temp = new Way<Point>(viewPort);
            temp.Sort(delegate(Point p1, Point p2)
            {
                if (p1.X - p2.X > 0)
                {
                    return 1;
                }
                if (p1.X - p2.X < 0)
                {
                    return -1;
                }

                return 0;
            });
            decimal minX = temp[0].X;
            decimal maxX = temp[temp.Count - 2].X;

            temp.Sort(delegate(Point p1, Point p2)
            {
                if (p1.Y - p2.Y > 0)
                {
                    return 1;
                }
                if (p1.Y - p2.Y < 0)
                {
                    return -1;
                }

                return 0;
            });
            decimal minY = temp[0].Y;
            decimal maxY = temp[temp.Count - 2].Y;

            if (SafeCompare.SafeLessThan(p.X, minX) || SafeCompare.SafeGreaterThan(p.X, maxX) || SafeCompare.SafeLessThan(p.Y, minY) || SafeCompare.SafeGreaterThan(p.Y, maxY))
            {
                return false;
            }

            return true;
        }

        private static int BACK_TRACK_RETRIES = 10000;

        private static bool PointTouchesViewPortOutside(Way<Point> way, Point point, Way<Point> viewPort)
        {
            int idx = way.IndexOf(point);

            int nextIdx = (idx + 1) % way.Count;
            Point nextPoint = way[nextIdx];
            // way.Count - 2 because first and last points are equal...
            int prevIdx = idx - 1 >= 0 ? idx - 1 : way.Count - 2;
            Point prevPoint = way[prevIdx];

            if (!viewPort.Contains(point))
            {
                return false;
            }
            if (!PointInViewport(nextPoint, viewPort) && !PointInViewport(prevPoint, viewPort))
            {
                return true;
            }

            return false;
        }

        private static bool PointTouchesViewPortInside(Way<Point> way, Point point, Way<Point> viewPort)
        {
            int idx = way.IndexOf(point);

            int nextIdx = (idx + 1) % way.Count;
            Point nextPoint = way[nextIdx];
            // way.Count - 2 because first and last points are equal...
            int prevIdx = idx - 1 >= 0 ? idx - 1 : way.Count - 2;
            Point prevPoint = way[prevIdx];

            if (!viewPort.Contains(point))
            {
                return false;
            }
            if (PointInViewport(nextPoint, viewPort) && PointInViewport(prevPoint, viewPort))
            {
                return true;
            }

            return false;
        }

        private static bool PointTouchesButDoesntIntersectViewPort(Way<Point> way, Point point, Way<Point> viewPort)
        {
            if (way.Equals(viewPort))
            {
                return false;
            }

            return PointTouchesViewPortInside(way, point, viewPort) || PointTouchesViewPortOutside(way, point, viewPort);
        }

        private static bool PointOnViewPortEdge(Way<Point> viewPort, Point p)
        {
            for (int i = 0; i < viewPort.Count; i++)
            {
                Point v1 = viewPort[i];
                Decimal dx = Math.Abs(v1.X - p.X);
                Decimal dy = Math.Abs(v1.Y - p.Y);

                if (dx == 0 || dy == 0)
                {
                    return true;
                }
            }

            return false;
        }

        private static void CleanExtraNonIntersectionPointsOnViewPort(Way<Point> way, Way<Point> viewPort, List<Point> intersections)
        {
            for (int i = 0; i < way.Count; i++)
            {
                Point p = way[i];
                if (PointOnViewPortEdge(viewPort, p) && !intersections.Contains(p))
                {
                    way.RemoveAt(i);
                }
            }
        }

        private static bool WayOutsideViewPort(Way<Point> way, Way<Point> viewPort, List<Point> intersections)
        {
            foreach (Point p in way)
            {
                if (PointInViewport(p, viewPort) && !intersections.Contains(p) && !PointOnViewPortEdge(viewPort, p))
                {
                    return false;
                }
            }

            return true;
        }

        private static bool TryToBuildPolygons(List<Way<Point>> polygons, Dictionary<Point, List<Way<Point>>> pointToWays, ref Way<Point> startingWay, ref Way<Point> viewPort, Way<Point> origViewPort, ref int startingIdx, ref bool followViewPort, ref bool backtracked, List<Point> intersections)
        {
            Way<Point> polygon = null;
            int idx = startingIdx;
            polygon = new Way<Point>();
            Way<Point> curWay = startingWay;
            Point curPoint = null;
            // if a polygon has more points comprising it than all the points available, we have a problem
            int CLOSE_WAY_RETRIES = pointToWays.Count;
            while (intersections.Count > 1)
            {
                List<Point> intersectionsRemoved = new List<Point>();
                List<Point> intersectionsNotToRemove = new List<Point>();
                while (!polygon.IsClosedWay())
                {
                    if (polygon.Count > CLOSE_WAY_RETRIES)
                    {
                        throw new Exception("Endless loop trying to close polygon; there is probably something wrong with the data");
                    }
                    curPoint = curWay[idx];
                    if (!PointInViewport(curPoint, origViewPort)) // origViewPort because otherwise, get weird, concave shapes as points removed from viewPort
                    {
                        // backtrack
                        startingIdx = idx - 1;
                        if (startingIdx == -1)
                        {
                            startingIdx = curWay.Count - 1;
                        }
                        startingWay = curWay;
                        followViewPort = true;

                        // reset, as we might need them since we didn't form a valid polygon
                        foreach (Point p in intersectionsRemoved)
                        {
                            intersections.Add(p);
                        }
                        return false;
                    }
                    // handle consecutive points on the viewport when it and the other way are opposite each other
                    // this would ordinarily cause a backtrack, but because there are consecutive points
                    // on the viewport, we just get an endless loop between the viewport and the other
                    // way. we fix this by seeing if we have removed the current intersection in question
                    // previously. in that case, we have seen it already
                    // in that case, we are in a loop with the viewPort and the other way, which
                    // has consecutive points on the viewport, but these aren't cleaned up because
                    // this way goes opposite the viewport (aka would force a backtrack if it wasn't
                    // for these consecutive points). The way we deal with this is forcing a backtrack
                    // by setting inLoopForceBacktrack to true, which will make the code follow the other
                    // way instead of the viewport once the point is reached.This will force a backtrack, and we will essentially
                    // remove these points on the viewport by successive backtracks because they won't
                    // be in the intersectionsRemoved list
                    bool inLoopForceBacktrack = false;
                    if (intersectionsRemoved.Contains(curPoint))
                    {
                        intersectionsRemoved.Remove(curPoint);
                        inLoopForceBacktrack = true;
                    }
                    if (intersections.Contains(curPoint))
                    {
                        intersections.Remove(curPoint);
                        intersectionsRemoved.Add(curPoint);

                        int nextIdx = (idx + 1) % curWay.Count;
                        Point nextPoint = curWay[nextIdx];
                        // way's which have multiple consecutive points on the viewport are problematic and break this algorithm;
                        // remove these points from the intersection array as these ways will either be ignored (if the viewport
                        // and way are opposite direction), or they will be incorporated into the polygon eventually (if they are the
                        // same direction)
                        while (!curWay.Equals(viewPort) && intersections.Contains(nextPoint))
                        {
                            intersections.Remove(nextPoint);
                            intersectionsRemoved.Add(nextPoint);
                            nextIdx = (nextIdx + 1) % curWay.Count;
                            nextPoint = curWay[nextIdx];
                        }
                    }

                    polygon.Add(curPoint);
                    List<Way<Point>> waysContainingPoint = pointToWays[curPoint];
                    if (waysContainingPoint.Count == 0)
                    {
                        throw new Exception("Something went wrong trying to create coast polygons");
                    }
                    else if (waysContainingPoint.Count == 1)
                    {
                        curWay = waysContainingPoint[0];
                    }
                    else if (waysContainingPoint.Contains(viewPort))
                    {
                        bool firstOrLastPoint = curPoint.Equals(curWay[0]) || curPoint.Equals(curWay[curWay.Count - 1]);
                        bool pointTouchesButDoesntIntersectViewPort = PointTouchesButDoesntIntersectViewPort(curWay, curPoint, origViewPort);
                        if (followViewPort && !inLoopForceBacktrack && (!pointTouchesButDoesntIntersectViewPort || firstOrLastPoint))
                        {
                            curWay = viewPort;
                        }
                        else
                        {
                            // choose the other way that is not the viewPort
                            foreach (Way<Point> w in waysContainingPoint)
                            {
                                if (!w.Equals(viewPort))
                                {
                                    curWay = w;
                                    break;
                                }
                            }
                        }
                        if (pointTouchesButDoesntIntersectViewPort)
                        {
                            // will be reset to true right below :) so we can follow the viewport at the next intersection
                            // if the next intersection isn't another point that just touches of course...
                            followViewPort = false;
                        }
                        followViewPort = !followViewPort;
                    }
                    else
                    {
                        // choose the only way that we can
                        curWay = waysContainingPoint[0];
                    }
                    idx = curWay.IndexOf(curPoint);

                    idx = (idx + 1) % curWay.Count;
                    if (backtracked && PointTouchesButDoesntIntersectViewPort(curWay, curPoint, origViewPort))
                    {
                        intersections.Add(curPoint);
                        intersectionsRemoved.Remove(curPoint);
                        intersectionsNotToRemove.Add(curPoint);
                    }
                }
                polygons.Add(polygon);

                foreach (Point p in polygon)
                {
                    if (!intersectionsNotToRemove.Contains(p))
                    {
                        viewPort.Remove(p);
                    }
                }

                if (intersections.Count > 0)
                {
                    // reset to make sure all remaining intersections are built into polygons
                    curWay = viewPort;
                    idx = 0;
                    // start at 0, or the first intersection, if there is one. Always starting at 0 can lead to cycles forming (endless loop)
                    for (int i = 0; i < viewPort.Count; i++)
                    {
                        if (intersections.Contains(viewPort[i]))
                        {
                            idx = i;
                            break;
                        }
                    }
                    polygon = new Way<Point>();
                    followViewPort = true;
                    backtracked = false;
                }
            }

            return true;
        }

        // returns true if a way is circular, and only touches the viewport at singular points (rather than cutting it).
        private static bool CircularWayOnlyTouchesViewPort(Way<Point> way, List<Point> intersections, Way<Point> viewPort)
        {
            if (!way.IsClosedWay())
            {
                throw new Exception("Way must be circular inside CircularWayOnlyTouchesViewPort.");
            }

            foreach (Point p in intersections)
            {
                if (!PointTouchesViewPortInside(way, p, viewPort))
                {
                    return false;
                }
            }

            // all of the intersections only touch the viewport inside, not fully cut it
            return true;
        }

        private static List<Way<Point>> CoastWaysToPolygon(List<Way<Point>> coastWays, Way<Point> viewPort, List<Way<Point>>[] inlandPolygons)
        {
            List<Way<Point>> polygons = new List<Way<Point>>();
            List<Point> allIntersections = new List<Point>();
            Dictionary<Point, List<Way<Point>>> pointToWays = new Dictionary<Point, List<Way<Point>>>();
            List<Way<Point>> waysShouldBeLandPolygons = new List<Way<Point>>();

            foreach (Way<Point> way in coastWays)
            {
                List<Point> intersections = way.IntersectsWith(viewPort, true, true);
                if (intersections == null)
                {
                    continue;
                }
                CleanExtraNonIntersectionPointsOnViewPort(way, viewPort, intersections);
                if (WayOutsideViewPort(way, viewPort, intersections))
                {
                    foreach (Point p in intersections)
                    {
                        viewPort.Remove(p);
                        way.Remove(p);
                    }
                    // do this to remove the way from the coastWays list.
                    // won't be actually made into one of the other polygons if not closed
                    // even if it is closed but it's outside the viewport, it won't matter
                    waysShouldBeLandPolygons.Add(way);
                }
                else if (way.IsClosedWay())
                {
                    // if the way is circular and only touches the viewport at singular points (rather than cutting through it)
                    // treat these was as land polygons
                    if (!CircularWayOnlyTouchesViewPort(way, intersections, viewPort))
                    {
                        foreach (Point p in intersections)
                        {
                            allIntersections.Add(p);
                        }
                        PopulatePointToWaysDict(pointToWays, way);
                    }
                    else
                    {
                        waysShouldBeLandPolygons.Add(way);
                    }
                }
                else
                {
                    foreach (Point p in intersections)
                    {
                        allIntersections.Add(p);
                    }
                    PopulatePointToWaysDict(pointToWays, way);
                }
            }
            // now, remove all those ways whose intersections only touch the viewport at a single point, but do not fully traverse it
            // as these should be treated as land polygons. Otherwise, breaks our sea polygon creation logic and get endless loops
            foreach (Way<Point> way in waysShouldBeLandPolygons)
            {
                coastWays.Remove(way);
                if (way.IsClosedWay())
                {
                    inlandPolygons[1].Add(way);
                }
            }

            Way<Point> viewPortWithoutLastPoint = new Way<Point>(viewPort);
            viewPortWithoutLastPoint.RemoveAt(viewPort.Count - 1);
            PopulatePointToWaysDict(pointToWays, viewPortWithoutLastPoint);
            bool keepTrying = true;
            int startingIdx = 0;
            Way<Point> startingWay = viewPort;
            bool followViewPort = false;
            bool backtracked = false;
            Way<Point> origViewPort = new Way<Point>(viewPort);

            int numRetries = 0;
            while (keepTrying)
            {
                keepTrying = false;
                bool newPolys = false;
                try
                {
                    newPolys = TryToBuildPolygons(polygons, pointToWays, ref startingWay, ref viewPort, origViewPort, ref startingIdx, ref followViewPort, ref backtracked, allIntersections); // copy of intersections in case we need to start again
                }
                catch (Exception e)
                {
                    throw e;
                }
                if (!newPolys)
                {
                    numRetries++;
                    keepTrying = true;
                    backtracked = true;
                }

                if (numRetries > BACK_TRACK_RETRIES)
                {
                    throw new Exception("Failed building polygons; there is likely something wrong with the data");
                }
            }
            // reset for next round
            keepTrying = true;
            startingWay = viewPort;
            startingIdx = 0;

            return polygons;
        }

        // TODO: the array of List<Way<Point>> is ugly. Find another way to represent the different layers representing alternating land and water.
        public static void CreatePolygons(List<Way<Point>> coastWaterPolygons, List<Way<Point>>[] inlandPolygons, string coastXML, string waterXML, Way<Point> viewPort)
        {
            Dictionary<string, Way<Point>> coastWays = OSMXMLParser.GetWays(coastXML, true);
            Dictionary<string, Way<Point>> waterWays = OSMXMLParser.GetWays(waterXML, true);
            // remove any that have length 0 (can get when edit with JOSM)
            foreach (string wayID in coastWays.Keys.ToArray())
            {
                Way<Point> way = coastWays[wayID];
                if (way.Count == 0)
                {
                    coastWays.Remove(wayID);
                }
            }

            foreach (string wayID in waterWays.Keys.ToArray())
            {
                Way<Point> way = waterWays[wayID];
                if (way.Count == 0)
                {
                    waterWays.Remove(wayID);
                }
                // non coast water ways should always be closed polygons
                // TODO: is this always true? Lake erie, for example, is not closed for some reason.
                // Not sure if this is bug with your code trying to form water multipolygons, or just something
                // with the OSM data
                if (!way.IsClosedWay())
                {
                    way.Add(way[way.Count - 1]);
                }
                if (way.relation == "outer")
                {
                    if (!inlandPolygons[0].Contains(way))
                    {
                        inlandPolygons[0].Add(way);
                    }
                }
                else if (way.relation == "inner")
                {
                    if (!inlandPolygons[1].Contains(way))
                    {
                        inlandPolygons[1].Add(way);
                    }
                }
                else if (way.relation == null)
                {
                    if (!inlandPolygons[2].Contains(way))
                    {
                        inlandPolygons[2].Add(way);
                    }
                }
            }

            List<Way<Point>> mergedCoasts = MergeCoastLines(coastWays);

            List<Way<Point>> mergedWaterPolys = CoastWaysToPolygon(mergedCoasts, viewPort, inlandPolygons);
            // add the water polygons of coast ways which intersect with the view port
            foreach (Way<Point> way in mergedWaterPolys)
            {
                coastWaterPolygons.Add(way);
            }
            // now add the circular coasts, which either were already full polygons in OSM data, or became one during merging
            foreach (Way<Point> way in mergedCoasts)
            {
                if (way.IsClosedWay())
                {
                    inlandPolygons[1].Add(way);
                }
            }
        }

        private static List<Way<Point>> MergeCoastLines(Dictionary<string, Way<Point>> coastWays)
        {
            List<Way<Point>> toMerge = new List<Way<Point>>(coastWays.Values.ToArray());
            OSMXMLParser.MergeMultipolygonWays(toMerge);

            return toMerge;
        }

        public static void GetPolygons(List<Way<Point>> coastWaterPolygons, List<Way<Point>>[] inlandPolygons, DownloadArea d, Way<Point> viewPort, string saveLoc)
        {
            string coastXML = DownloadOsmCoastData(d, saveLoc + Path.DirectorySeparatorChar + "coast.osm");
            string waterXML = DownloadOsmWaterData(d, saveLoc + Path.DirectorySeparatorChar + "water.osm");

            CreatePolygons(coastWaterPolygons, inlandPolygons, coastXML, waterXML, viewPort);
        }

        // these lat long to pixel, and vice versa, formulas, are from FSEarthtiles
        public static Point LatLongToPixel(Point latLong, decimal startLat, decimal startLong, decimal pixelsPerLongitude, decimal pixelsPerLatitude)
        {
            Point vPixelXYCoord = new Point(pixelsPerLongitude * (latLong.X - startLong), pixelsPerLatitude * (startLat - latLong.Y));

            return vPixelXYCoord;
        }


        public static Point CoordToPixel(decimal lat, decimal lon, decimal NWLat, decimal NWLon, decimal pixelsPerLongitude,
                                    decimal pixelsPerLatitude)
        {
            Point tempCoord = new Point(lon, lat);
            Point pixel = LatLongToPixel(tempCoord, NWLat, NWLon, pixelsPerLongitude, pixelsPerLatitude);
            pixel.X -= 0.5m;
            pixel.Y -= 0.5m;

            return pixel;
        }

        private static void DrawPolygons(Bitmap bmp, Graphics g, SolidBrush b, decimal pixelsPerLon, decimal pixelsPerLat, Point NW, List<Way<Point>> polygons)
        {
            foreach (Way<Point> way in polygons)
            {

                List<PointF> l = new List<PointF>();
                for (int i = 0; i < way.Count - 1; i++) // FillPolygon polygons don't need last point, hence - 1
                {
                    Point p = way[i];
                    Point pixel = CoordToPixel(p.Y, p.X, NW.Y, NW.X, pixelsPerLon, pixelsPerLat);

                    l.Add(new PointF((float)pixel.X, (float)pixel.Y));
                }
                PointF[] pf = l.ToArray();
                g.FillPolygon(b, pf);

            }
        }

        public static Bitmap GetMask(string outPath, int width, int height, Point NW, Point SE, List<Way<Point>> waterPolygons, List<Way<Point>>[] inlandPolygons)
        {
            Bitmap bmp = new Bitmap(width, height, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            decimal pixelsPerLon = Convert.ToDecimal(width) / (SE.X - NW.X);
            decimal pixelsPerLat = Convert.ToDecimal(height) / (NW.Y - SE.Y);

            using (Graphics g = Graphics.FromImage(bmp))
            {
                g.FillRectangle(Brushes.White, 0, 0, bmp.Width, bmp.Height);
                SolidBrush b = new SolidBrush(Color.Black);
                DrawPolygons(bmp, g, b, pixelsPerLon, pixelsPerLat, NW, waterPolygons);
                for (int i = 0; i < inlandPolygons.Length; i++)
                {
                    if (i % 2 == 0)
                    {
                        b = new SolidBrush(Color.Black);
                        DrawPolygons(bmp, g, b, pixelsPerLon, pixelsPerLat, NW, inlandPolygons[i]);
                    }
                    else
                    {
                        b = new SolidBrush(Color.White);
                        DrawPolygons(bmp, g, b, pixelsPerLon, pixelsPerLat, NW, inlandPolygons[i]);
                    }
                }
            }

            return bmp;
        }
    }
}
