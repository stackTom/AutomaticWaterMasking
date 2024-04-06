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

    public class Edge
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

        private decimal GetDelta(Edge e)
        {
            return this.A * e.B - e.A * this.B;
        }

        public bool ParallelTo(Edge e)
        {
            return SafeCompare.SafeEquals(GetDelta(e), 0m);
        }

        // returns point of intersection, null if no intersection
        // didn't want to do high school math, so thanks to
        // https://stackoverflow.com/questions/4543506/algorithm-for-intersection-of-2-lines
        public Point IntersectsWith(Edge e)
        {
            decimal delta = GetDelta(e);

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


    public class Way<T> : System.Collections.Generic.List<T> where T : Point
    {
        public string relation;
        public string type;
        public string wayID;
        public List<int> intersectionIDXs;

        public Way() : base()
        {
            intersectionIDXs = new List<int>();
        }
        public Way(Way<T> way) : base(way)
        {
            intersectionIDXs = way.intersectionIDXs;
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

        // Compare to other way by looking at contents of ways, not the id's
        // Orientation matters...
        public bool DeepEquals(object obj)
        {
            Way<T> cmp = (Way<T>)obj;
            if (this.Count != cmp.Count)
            {
                return false;
            }

            for (int i = 0; i < this.Count; i++)
            {
                if (!this[i].Equals(cmp[i]))
                {
                    return false;
                }
            }

            return true;
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

        public void RemoveUpdatingIntersectionIDXs(T toRemove)
        {
            int idxInToRemove = this.IndexOf(toRemove);
            int idxInIntersectionIDXs = this.intersectionIDXs.IndexOf(idxInToRemove);
            if (idxInIntersectionIDXs != -1)
            {
                this.intersectionIDXs.RemoveAt(idxInIntersectionIDXs);
                for (int i = idxInIntersectionIDXs; i < this.intersectionIDXs.Count; i++)
                {
                    this.intersectionIDXs[i]--;
                }
            }
            else
            {
                for (int i = 0; i < this.intersectionIDXs.Count; i++)
                {
                    if (this.intersectionIDXs[i] > idxInToRemove)
                    {
                        this.intersectionIDXs[i]--;
                    }
                }
            }
            this.Remove(toRemove);
        }

        public Point GetPointAtOffsetFromPoint(Point p, int offset)
        {
            if (offset == 0)
            {
                return p;
            }

            int idx = this.IndexOf((T)p);
            if (offset > 0)
            {
                int i = 0;
                Point nextPoint = null;
                while (i < offset)
                {
                    idx = (idx + 1) % this.Count;
                    if (this.IsClosedWay() && idx == 0)
                    {
                        // first and last points are equal
                        idx = 1;
                    }
                    nextPoint = this[idx];
                    i++;
                }

                return nextPoint;
            }
            // idx < 0
            int j = 0;
            Point prevPoint = null;
            while (j > offset)
            {
                if ((idx - 1) >= 0)
                {
                    idx--;
                }
                else
                {
                    if (this.IsClosedWay())
                    {
                        // way.Count - 2 because first and last points are equal...
                        idx = this.Count - 2;
                    }
                    else
                    {
                        idx = this.Count - 1;
                    }
                }
                prevPoint = this[idx];
                j--;
            }

            return prevPoint;
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
                            // if the way contains the intersection already, can be at i (j) or i + 1 (j + 1)
                            if (!this.Contains(intersection))
                            {
                                // going to be added down below at i + 1
                                this.intersectionIDXs.Add(i + 1);
                            }
                            else
                            {
                                int idx = intersection.Equals(this[i]) ? i : i + 1;
                                this.intersectionIDXs.Add(idx);
                            }
                            if (!check.Contains(intersection))
                            {
                                // going to be added down below at j + 1
                                check.intersectionIDXs.Add(j + 1);
                            }
                            else
                            {
                                int idx = intersection.Equals(check[j]) ? j : j + 1;
                                check.intersectionIDXs.Add(idx);
                            }
                        }
                        // TODO: this can probably be put into above if, since you never really want to insert
                        // if it's not in the intersection array
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

        public decimal Area()
        {
            // credit: https://stackoverflow.com/questions/2034540/calculating-area-of-irregular-polygon-in-c-sharp
            decimal area = Math.Abs(this.Take(this.Count - 1)
                   .Select((p, i) => (this[i + 1].X - p.X) * (this[i + 1].Y + p.Y))
                   .Sum() / 2);

            return area;
        }

        public static void AddMissingOSMAttributes(XmlElement nodeEle)
        {
            nodeEle.SetAttribute("uid", "1");
            nodeEle.SetAttribute("changeset", "1");
            nodeEle.SetAttribute("version", "1");
            nodeEle.SetAttribute("timestamp", "2000-01-01T00:00:00Z");
            nodeEle.SetAttribute("user", "AutomaticWaterMasker");
        }

        public XmlElement CreateOSMXMLWayNodeAndNDEles(XmlDocument d, XmlElement osmEle, string wayID, string relation, int startIdx)
        {
            XmlElement wayEle = d.CreateElement(string.Empty, "way", string.Empty);
            wayEle.SetAttribute("id", wayID);
            if (relation != null)
            {
                wayEle.SetAttribute("relation", relation);
            }
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
            XmlElement wayEle = CreateOSMXMLWayNodeAndNDEles(d, osmEle, this.wayID != null ? this.wayID : "1", this.relation, i);

            osmEle.AppendChild(wayEle);

            return d.OuterXml;
        }
    }

    public class OSMXMLParser
    {
        public Dictionary<string, List<string>> wayIDsToWayNodes;
        public Dictionary<string, Point> nodeIDsToCoords;
        public Dictionary<string, Way<Point>> wayIDsToWays; // TODO: do we really need a Dictionary, can probably be a List
        // POSSIBLE BUG: can a way every be both inner and outer in a relationship?
        public Dictionary<string, string> wayIDsToRelation;
        public Dictionary<string, string> wayIDsToType;
        private XmlDocument d;

        public OSMXMLParser(string xmlString)
        {
            this.d = new XmlDocument();
            this.d.LoadXml(xmlString);
            this.Parse();
        }

        private Dictionary<string, Point> GetNodeIDsToCoords()
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

        private Dictionary<string, List<string>> GetWayIDsToWayNodes()
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

        private Dictionary<string, Way<Point>> GetWayIDsToWays(Dictionary<string, List<string>> wayIDsToWayNodes, Dictionary<string, Point> nodeIDsToCoords)
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

        public void Parse()
        {
            this.wayIDsToWayNodes = GetWayIDsToWayNodes();
            this.nodeIDsToCoords = GetNodeIDsToCoords();
            this.wayIDsToWays = GetWayIDsToWays(wayIDsToWayNodes, nodeIDsToCoords);
            // POSSIBLE BUG: can a way every be both inner and outer in a relationship?
            this.wayIDsToRelation = GetWayIDsToRelation(wayIDsToWayNodes, nodeIDsToCoords);
            this.wayIDsToType = new Dictionary<string, string>();
        }

        private List<Way<Point>> GetWaysInThisMultipolygonAndUpdateRelations(Dictionary<string, Way<Point>> wayIDsToWays, XmlElement rel, Dictionary<string, string> wayIDsToRelation, Dictionary<string, string> wayIDsToType)
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
                                mergedWay.wayID = way1.wayID + "m";
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

        private Dictionary<string, string> GetWayIDsToRelation(Dictionary<string, List<string>> wayIDsToWayNodes, Dictionary<string, Point> nodeIDsToCoords)
        {
            Dictionary<string, string> wayIDsToRelations = new Dictionary<string, string>();

            XmlNodeList wayTags = this.d.GetElementsByTagName("way");
            foreach (XmlElement way in wayTags)
            {
                string relation = way.GetAttribute("relation");
                string id = way.GetAttribute("id");

                if (relation != null && relation != "")
                {
                    wayIDsToRelations.Add(id, relation);
                }
            }

            return wayIDsToRelations;
        }

        public List<Way<Point>> GetWays(bool mergeMultipolygons)
        {
            List<Way<Point>> toRemove = new List<Way<Point>>();
            List<Way<Point>> toAdd = new List<Way<Point>>();

            foreach (KeyValuePair<string, Way<Point>> kv in this.wayIDsToWays)
            {
                string wayID = kv.Key;
                Way<Point> way = kv.Value;
                way.relation = null;
                way.wayID = wayID;
                if (this.wayIDsToRelation.ContainsKey(wayID))
                {
                    way.relation = this.wayIDsToRelation[wayID];
                }
            }

            XmlNodeList relationTags = this.d.GetElementsByTagName("relation");
            foreach (XmlElement rel in relationTags)
            {
                // unite multipolygon pieces into one big linestring
                // we need this because singular way lines of inner water are problematic
                // it is hard to determine direction the water is in relative to the way because I believe OSM only requires direction
                // for coastal ways. but if we make them a full polygon, then it is easy to determine that the water is inside the polygon
                // here we compare every way to every other way.
                List<Way<Point>> waysInThisMultipolygon = GetWaysInThisMultipolygonAndUpdateRelations(this.wayIDsToWays, rel, this.wayIDsToRelation, this.wayIDsToType);
                // update relations
                foreach (KeyValuePair<string, Way<Point>> kv in this.wayIDsToWays)
                {
                    string wayID = kv.Key;
                    Way<Point> way = kv.Value;
                    way.relation = null;
                    way.wayID = wayID;
                    if (this.wayIDsToRelation.ContainsKey(wayID))
                    {
                        way.relation = this.wayIDsToRelation[wayID];
                        way.type = this.wayIDsToType[wayID];
                    }
                }

                if (mergeMultipolygons)
                {
                    // mark all the ones that are in this multipolygon for deletion eventually
                    // don't delete them yet as sometimes, multiple multipolygons can use one way...
                    foreach (Way<Point> way in waysInThisMultipolygon)
                    {
                        if (!toRemove.Contains(way))
                        {
                            toRemove.Add(way);
                        }
                    }
                    MergeMultipolygonWays(waysInThisMultipolygon);

                    // add new merged ways to eventually be added to wayIDsToWays
                    // don't just add them now, otherwise, can have their relation set to null above...
                    foreach (Way<Point> way in waysInThisMultipolygon)
                    {
                        if (!toAdd.Contains(way))
                        {
                            toAdd.Add(way);
                        }
                    }
                }
            }

            foreach (Way<Point> way in toRemove)
            {
                this.wayIDsToWays.Remove(way.wayID);
            }
            foreach (Way<Point> way in toAdd)
            {
                if (way.Count > 0)
                {
                    this.wayIDsToWays.Add(way.wayID, way);
                }
            }

            return this.wayIDsToWays.Values.ToList();
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
                XmlElement wayEle = way.CreateOSMXMLWayNodeAndNDEles(d, osmEle, i.ToString(), way.relation, i);
                osmEle.AppendChild(wayEle);
                i += way.Count;
            }

            return d.OuterXml;
        }

        public void ForceFreeMemory()
        {
            this.wayIDsToWayNodes = null;
            this.nodeIDsToCoords = null;
            this.wayIDsToWays = null;
            this.wayIDsToRelation = null;
            this.wayIDsToType = null;
            this.d = null;
            this.wayIDsToWays = null;

            GC.Collect();
            GC.WaitForPendingFinalizers();
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

    public class MaskingPolys
    {
        public List<Way<AutomaticWaterMasking.Point>> coastWaterPolygons;
        public List<Way<AutomaticWaterMasking.Point>> islands;
        public List<Way<AutomaticWaterMasking.Point>> inlandPolygons;
        public string tileName;

        public override string ToString()
        {
            return tileName;
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

        // for offsetting CoordToPixel calculation
        public static decimal xOffset = 0.0m;
        public static decimal yOffset = 0.0m;
        private static int totalPoints = 0;

        // sets missing values from OSM data to make a valid OSM file that JOSM can load
        private static string FixOSM(string OSM)
        {
            XmlDocument d = new XmlDocument();
            d.LoadXml(OSM);
            XmlNodeList tagsToFix = d.GetElementsByTagName("node");
            foreach (XmlElement node in tagsToFix)
            {
                Way<Point>.AddMissingOSMAttributes(node);
            }
            tagsToFix = d.GetElementsByTagName("way");
            foreach (XmlElement node in tagsToFix)
            {
                Way<Point>.AddMissingOSMAttributes(node);
            }
            tagsToFix = d.GetElementsByTagName("relation");
            foreach (XmlElement node in tagsToFix)
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

        private static void PopulatePointToIntersectingWayDict(Dictionary<Point, Way<Point>> dict, Way<Point> way, List<Point> intersections)
        {
            foreach (Point p in intersections)
            {
                dict.Add(p, way);
            }
        }

        private static Point[] MinMaxOfViewPort(Way<Point> viewPort)
        {
            Way<Point> temp = new Way<Point>(viewPort);
            temp.Sort(delegate (Point p1, Point p2)
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

            temp.Sort(delegate (Point p1, Point p2)
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

            return new Point[] { new Point(minX, minY), new Point(maxX, maxY) };
        }

        private static bool PointOnViewPortEdge(Point p, Way<Point> viewPort)
        {
            Point[] minMax = MinMaxOfViewPort(viewPort);
            decimal minX = minMax[0].X;
            decimal minY = minMax[0].Y;
            decimal maxX = minMax[1].X;
            decimal maxY = minMax[1].Y;

            if (SafeCompare.SafeEquals(p.X, minX) || SafeCompare.SafeEquals(p.X, maxX))
            {
                if (SafeCompare.SafeEquals(p.Y, minY) || SafeCompare.SafeEquals(p.Y, maxY))
                {
                    return true;
                }
                if (SafeCompare.SafeGreaterThan(p.Y, minY) && SafeCompare.SafeLessThan(p.Y, maxY))
                {
                    return true;
                }

                return false;
            }
            if (SafeCompare.SafeEquals(p.Y, minY) || SafeCompare.SafeEquals(p.Y, maxY))
            {
                if (SafeCompare.SafeEquals(p.X, minX) || SafeCompare.SafeEquals(p.X, maxX))
                {
                    return true;
                }
                if (SafeCompare.SafeGreaterThan(p.X, minX) && SafeCompare.SafeLessThan(p.X, maxX))
                {
                    return true;
                }

                return false;
            }

            return false;
        }

        private static bool PointInViewport(Point p, Way<Point> viewPort)
        {
            Point[] minMax = MinMaxOfViewPort(viewPort);
            decimal minX = minMax[0].X;
            decimal minY = minMax[0].Y;
            decimal maxX = minMax[1].X;
            decimal maxY = minMax[1].Y;

            // outside viewport
            if (SafeCompare.SafeLessThan(p.X, minX) || SafeCompare.SafeGreaterThan(p.X, maxX) || SafeCompare.SafeLessThan(p.Y, minY) || SafeCompare.SafeGreaterThan(p.Y, maxY))
            {
                return false;
            }
            // not outside, but exactly on the viewport
            if (PointOnViewPortEdge(p, viewPort))
            {
                return false;
            }

            return true;
        }

        private static bool PointOutsideViewPort(Point p, Way<Point> wayContainingPoint, Way<Point> viewPort)
        {
            return !PointInViewport(p, viewPort) && !PointOnViewPortEdge(p, viewPort);
        }

        private static bool PointTouchesViewPortOutside(Way<Point> way, Point point, Way<Point> viewPort)
        {
            Point nextPoint = way.GetPointAtOffsetFromPoint(point, 1);
            Point prevPoint = way.GetPointAtOffsetFromPoint(point, -1);

            if (!viewPort.Contains(point) || (!way.IsClosedWay() && (point.Equals(way[0]) || point.Equals(way[way.Count - 1]))))
            {
                return false;
            }
            bool prevPointInViewPort = PointInViewport(prevPoint, viewPort) || (PointOnViewPortEdge(prevPoint, viewPort) && !PointOnViewPortSegment(viewPort, way, prevPoint));
            bool nextPointInViewPort = PointInViewport(nextPoint, viewPort) || (PointOnViewPortEdge(nextPoint, viewPort) && !PointOnViewPortSegment(viewPort, way, nextPoint));
            if (!nextPointInViewPort && !prevPointInViewPort)
            {
                return true;
            }

            return false;
        }

        private static bool PointTouchesViewPortInside(Way<Point> way, Point point, Way<Point> viewPort)
        {
            Point nextPoint = way.GetPointAtOffsetFromPoint(point, 1);
            Point prevPoint = way.GetPointAtOffsetFromPoint(point, -1);

            if (!viewPort.Contains(point) || (!way.IsClosedWay() && (point.Equals(way[0]) || point.Equals(way[way.Count - 1]))))
            {
                return false;
            }
            bool prevPointInViewPort = PointInViewport(prevPoint, viewPort) || (PointOnViewPortEdge(prevPoint, viewPort) && !PointOnViewPortSegment(viewPort, way, prevPoint));
            bool nextPointInViewPort = PointInViewport(nextPoint, viewPort) || (PointOnViewPortEdge(nextPoint, viewPort) && !PointOnViewPortSegment(viewPort, way, nextPoint));
            if (nextPointInViewPort && prevPointInViewPort)
            {
                return true;
            }

            return false;
        }

        private static bool PointTouchesButDoesntIntersectViewPort(Way<Point> way, Point point, Way<Point> viewPort)
        {
            return PointTouchesViewPortInside(way, point, viewPort) || PointTouchesViewPortOutside(way, point, viewPort);
        }

        private static bool PointOnViewPortSegment(Way<Point> viewPort, Way<Point> wayContainingPoint, Point p)
        {
            Edge checkEdgeForward = new Edge(p, wayContainingPoint.GetPointAtOffsetFromPoint(p, 1));
            Edge checkEdgeBackward = new Edge(p, wayContainingPoint.GetPointAtOffsetFromPoint(p, -1));

            for (int i = 0; i < viewPort.Count; i++)
            {
                Point cur = viewPort[i];
                Point next = viewPort.GetPointAtOffsetFromPoint(cur, 1);
                Decimal dx = Math.Abs(cur.X - p.X);
                Decimal dy = Math.Abs(cur.Y - p.Y);

                if (!PointInViewport(p, viewPort) && (SafeCompare.SafeEquals(dx, 0m) || SafeCompare.SafeEquals(dy, 0m)))
                {
                    Edge viewPortEdge = new Edge(cur, next);

                    if (checkEdgeForward.ParallelTo(viewPortEdge))
                    {
                        return true;
                    }
                    if (checkEdgeBackward.ParallelTo(viewPortEdge))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        private static bool ShouldFollowViewport(Way<Point> viewPort, Way<Point> origViewPort, Way<Point> curWay, Way<Point> otherWay, Way<Point> polygon, List<Point> intersections, Point curPoint)
        {
            bool followViewPort = false;
            Point next = otherWay.GetPointAtOffsetFromPoint(curPoint, 1);
            if (curWay.Equals(viewPort))
            {
                // wrong orientation of vectors
                if (PointOutsideViewPort(next, otherWay, origViewPort))
                {
                    throw new Exception("These vectors don't form valid polygons. Check the input data, and try again");
                }
            }

            if (!otherWay.IsClosedWay() && curPoint.Equals(otherWay[0]))
            {
                followViewPort = false;
            }
            else if (!otherWay.IsClosedWay() && curPoint.Equals(otherWay[otherWay.Count - 1]))
            {
                followViewPort = true;
            }
            else if (PointTouchesViewPortInside(otherWay, curPoint, origViewPort) && intersections.Contains(curPoint) && intersections.Count > 0)
            {
                // basically, if the point touches but doesn't transect the viewport, but the way goes backwards on the viewport...
                // then follow the viewport, as it can't possibly form this current polygon if it's going backwards
                // on the viewport. unless the point in question is the first point of the polygon, in which case
                // a successful polygon can be formed
                int viewPortIdx = origViewPort.IndexOf(curPoint);
                // get next intersection of the other way and the viewport
                Point otherWayPoint = otherWay.GetPointAtOffsetFromPoint(curPoint, 1);
                int timesAroundLoop = 0;
                while (!intersections.Contains(otherWayPoint))
                {
                    if (timesAroundLoop > otherWay.Count)
                    {
                        throw new Exception("Something went wrong with a point touching, but not transecting the viewport. There is probably something wrong with the data.");
                    }
                    otherWayPoint = otherWay.GetPointAtOffsetFromPoint(otherWayPoint, 1);
                    timesAroundLoop++;
                }
                int otherWayIdx = origViewPort.IndexOf(otherWayPoint);
                if (otherWayIdx < viewPortIdx && !curPoint.Equals(polygon[0]))
                {
                    followViewPort = true;
                }
                else
                {
                    followViewPort = false;
                }
            }
            else if (PointOutsideViewPort(next, otherWay, origViewPort) ||
                (!curWay.Equals(viewPort) && (PointOnViewPortSegment(origViewPort, otherWay, next))))
            {
                followViewPort = true;
            }
            else
            {
                followViewPort = false;
            }

            return followViewPort;
        }

        private static bool TryToBuildPolygons(List<Way<Point>> polygons, Dictionary<Point, Way<Point>> pointsToIntersectingWays, Way<Point> startingWay, Way<Point> viewPort, Way<Point> origViewPort, int startingIdx, bool followViewPort, List<Point> intersections)
        {
            Way<Point> polygon = null;
            int idx = startingIdx;
            polygon = new Way<Point>();
            Way<Point> curWay = startingWay;
            HashSet<Point> origIntersections = new HashSet<Point>(intersections);
            Point curPoint = null;
            // contains points that touch but not transect viewport and we should follow the viewport after
            HashSet<Point> touchingButNotTransectingFollowViewPort = new HashSet<Point>();
            // contains points that touch but not transect viewport and we should not follow the viewport after
            HashSet<Point> touchingButNotTransectingDontFollowViewPort = new HashSet<Point>();
            while (intersections.Count > 0)
            {
                while (!polygon.IsClosedWay())
                {
                    // if a polygon has more points comprising it than all the points available, we have a problem
                    if (polygon.Count > WaterMasking.totalPoints)
                    {
                        throw new Exception("Endless loop trying to close polygon; there is probably something wrong with the data");
                    }
                    curPoint = curWay[idx];

                    polygon.Add(curPoint);
                    if (origIntersections.Contains(curPoint))
                    {
                        Way<Point> otherWay = pointsToIntersectingWays[curPoint];

                        if (touchingButNotTransectingFollowViewPort.Contains(curPoint))
                        {
                            followViewPort = true;
                        }
                        else if (touchingButNotTransectingDontFollowViewPort.Contains(curPoint))
                        {
                            followViewPort = false;
                        }
                        else
                        {
                            followViewPort = ShouldFollowViewport(viewPort, origViewPort, curWay, otherWay, polygon, intersections, curPoint);
                        }

                        if (PointTouchesButDoesntIntersectViewPort(otherWay, curPoint, origViewPort))
                        {
                            // Basically, if currently on viewPort and now told to not follow it, or vice versa, this point can
                            // be formed into another polygon aside from the one we are currently forming. So add it to the
                            // intersections List again
                            // Examples are Tiles (-14, 143) (54, -59) (55, -61) (66, -62) (72, -109) (60, -45)
                            Point next = otherWay.GetPointAtOffsetFromPoint(curPoint, 1);
                            Point previous = otherWay.GetPointAtOffsetFromPoint(curPoint, -1);
                            if (!PointOutsideViewPort(next, otherWay, origViewPort) && !PointOutsideViewPort(previous, otherWay, origViewPort))
                            {
                                if (((!curWay.Equals(viewPort) && followViewPort) || (curWay.Equals(viewPort) && !followViewPort))
                                    && !touchingButNotTransectingFollowViewPort.Contains(curPoint) && !touchingButNotTransectingDontFollowViewPort.Contains(curPoint))
                                {
                                    intersections.Add(curPoint);
                                    if (followViewPort)
                                    {
                                        touchingButNotTransectingDontFollowViewPort.Add(curPoint);
                                    }
                                    else
                                    {
                                        touchingButNotTransectingFollowViewPort.Add(curPoint);
                                    }
                                }
                            }
                        }

                        if (followViewPort)
                        {
                            curWay = viewPort;
                        }
                        else
                        {
                            curWay = otherWay;
                        }

                    }
                    else
                    {
                        // just follow the way until the next intersection
                        // exclude viewPort because won't affect speeed + we make viewPort smaller each time
                        // which affects this algorithm.
                        if (!curWay.Equals(viewPort))
                        {
                            int beginIDX = curWay.IndexOf(curPoint);
                            int endIntersectionIDX = (curWay.intersectionIDXs.IndexOf(beginIDX - 1) + 1) % curWay.intersectionIDXs.Count;
                            int endIDX = curWay.intersectionIDXs[endIntersectionIDX];
                            if (endIDX == 0 && curWay.IsClosedWay())
                            {
                                endIDX = curWay.Count - 1;
                            }
                            for (int i = beginIDX + 1; i != endIDX; i = (i + 1) % curWay.Count)
                            {
                                // avoid repeats on roll over
                                if (!(i == 0 && polygon[polygon.Count - 1].Equals(curWay[i])))
                                {
                                    curPoint = curWay[i];
                                    polygon.Add(curPoint);
                                }
                            }
                        }
                    }

                    idx = curWay.IndexOf(curPoint);

                    idx = (idx + 1) % curWay.Count;
                    if (intersections.Contains(curPoint))
                    {
                        intersections.Remove(curPoint);
                    }
                }
                polygons.Add(polygon);

                foreach (Point p in polygon)
                {
                    if (!(touchingButNotTransectingFollowViewPort.Contains(p) || touchingButNotTransectingDontFollowViewPort.Contains(p)))
                    {
                        viewPort.Remove(p);
                    }
                }

                if (intersections.Count > 0)
                {
                    // start at 0, or the first intersection, if there is one. Always starting at 0 can lead to cycles forming (endless loop)
                    Point p = null;
                    for (int i = 0; i < viewPort.Count; i++)
                    {
                        p = viewPort[i];
                        if (intersections.Contains(p))
                        {
                            idx = i;
                            break;
                        }
                    }
                    curWay = pointsToIntersectingWays[p];
                    idx = curWay.IndexOf(p);
                    polygon = new Way<Point>();
                    followViewPort = true;
                }
            }

            return true;
        }

        // returns true if a way is circular, and only touches the viewport at singular points (rather than cutting it).
        // not really needed anymore. TODO: consider deleting
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

        private static bool ShouldRemovePointFromViewPort(Point point, Way<Point> viewPort, Way<Point> unmodifiedViewPort)
        {
            return viewPort.Contains(point) && !unmodifiedViewPort.Contains(point);
        }

        // remove intersections where a way intersects with the viewPort at a single point from the outside
        // TODO: need to clean up some repetitive code in the below function
        private static void CleanOutsideSinglePointIntersections(Dictionary<Point, Way<Point>> pointsToIntersectingWays, Way<Point> viewPort, Way<Point> unmodifiedViewPort, List<Point> intersections)
        {
            for (int i = 0; i < viewPort.Count; i++)
            {
                Point curPoint = viewPort[i];
                if (intersections.Contains(curPoint))
                {
                    Way<Point> otherWay = pointsToIntersectingWays[curPoint];
                    Point next = otherWay.GetPointAtOffsetFromPoint(curPoint, 1);
                    Point prev = otherWay.GetPointAtOffsetFromPoint(curPoint, -1);

                    if (!otherWay.IsClosedWay())
                    {
                        if (curPoint.Equals(otherWay[0]) && PointOutsideViewPort(next, otherWay, viewPort))
                        {
                            intersections.Remove(curPoint);
                            if (ShouldRemovePointFromViewPort(curPoint, viewPort, unmodifiedViewPort))
                            {
                                viewPort.Remove(curPoint);
                                i--;
                            }
                            otherWay.RemoveUpdatingIntersectionIDXs(curPoint);
                        }
                        else if (curPoint.Equals(otherWay[otherWay.Count - 1]) && PointOutsideViewPort(prev, otherWay, viewPort))
                        {
                            intersections.Remove(curPoint);
                            if (ShouldRemovePointFromViewPort(curPoint, viewPort, unmodifiedViewPort))
                            {
                                viewPort.Remove(curPoint);
                                i--;
                            }
                            otherWay.RemoveUpdatingIntersectionIDXs(curPoint);
                        }
                    }
                    if (PointTouchesViewPortOutside(otherWay, curPoint, viewPort))
                    {
                        intersections.Remove(curPoint);
                        if (ShouldRemovePointFromViewPort(curPoint, viewPort, unmodifiedViewPort))
                        {
                            viewPort.Remove(curPoint);
                            i--;
                        }
                        otherWay.RemoveUpdatingIntersectionIDXs(curPoint);
                    }
                    else if (PointOutsideViewPort(prev, otherWay, viewPort))
                    {
                        while (PointOnViewPortSegment(viewPort, otherWay, curPoint) && !PointInViewport(next, viewPort))
                        {
                            Point nextCurPoint = otherWay.GetPointAtOffsetFromPoint(curPoint, 1);
                            intersections.Remove(curPoint);
                            if (ShouldRemovePointFromViewPort(curPoint, viewPort, unmodifiedViewPort))
                            {
                                viewPort.Remove(curPoint);
                                i--;
                            }
                            otherWay.RemoveUpdatingIntersectionIDXs(curPoint);
                            curPoint = nextCurPoint;
                            next = otherWay.GetPointAtOffsetFromPoint(curPoint, 1);
                        }
                    }
                    else if (PointOutsideViewPort(next, otherWay, viewPort))
                    {
                        while (PointOnViewPortSegment(viewPort, otherWay, curPoint) && !PointInViewport(prev, viewPort))
                        {
                            Point nextCurPoint = otherWay.GetPointAtOffsetFromPoint(curPoint, -1);
                            intersections.Remove(curPoint);
                            if (ShouldRemovePointFromViewPort(curPoint, viewPort, unmodifiedViewPort))
                            {
                                viewPort.Remove(curPoint);
                                i--;
                            }
                            otherWay.RemoveUpdatingIntersectionIDXs(curPoint);
                            curPoint = nextCurPoint;
                            prev = otherWay.GetPointAtOffsetFromPoint(curPoint, -1);
                        }
                    }
                    else
                    {
                        Point nnext = otherWay.GetPointAtOffsetFromPoint(next, 1);
                        while (PointOnViewPortSegment(viewPort, otherWay, next) && PointOnViewPortSegment(viewPort, otherWay, nnext))
                        {
                            intersections.Remove(next);
                            if (ShouldRemovePointFromViewPort(next, viewPort, unmodifiedViewPort))
                            {
                                viewPort.Remove(next);
                                i--;
                            }
                            otherWay.RemoveUpdatingIntersectionIDXs(next);
                            next = nnext;
                            nnext = otherWay.GetPointAtOffsetFromPoint(next, 1);
                        }
                    }
                }
            }
        }

        private static List<Way<Point>> CoastWaysToPolygon(List<Way<Point>> coastWays, Way<Point> viewPort, List<Way<Point>> islands, List<Way<Point>> inlandPolygons)
        {
            List<Way<Point>> polygons = new List<Way<Point>>();
            List<Point> allIntersections = new List<Point>();
            Dictionary<Point, Way<Point>> pointsToIntersectingWays = new Dictionary<Point, Way<Point>>();
            List<Way<Point>> waysShouldBeLandPolygons = new List<Way<Point>>();
            Way<Point> unmodifiedViewPort = new Way<Point>(viewPort);

            WaterMasking.totalPoints = 0;
            foreach (Way<Point> way in coastWays)
            {
                List<Point> intersections = way.IntersectsWith(viewPort, true, true);
                if (intersections == null)
                {
                    continue;
                }
                WaterMasking.totalPoints += way.Count;
                if (way.IsClosedWay())
                {
                    // if the way is circular and only touches the viewport at singular points (rather than cutting through it)
                    // treat these was as land polygons
                    // not really needed anymore, as ShouldFollowViewport is robust enough in creating water polygons now even in
                    // cases with these types of ways... TODO: consider removing this
                    if (!CircularWayOnlyTouchesViewPort(way, intersections, viewPort))
                    {
                        foreach (Point p in intersections)
                        {
                            allIntersections.Add(p);
                        }
                        PopulatePointToIntersectingWayDict(pointsToIntersectingWays, way, intersections);
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
                    PopulatePointToIntersectingWayDict(pointsToIntersectingWays, way, intersections);
                }
            }
            // add viewPort count to total possible points
            WaterMasking.totalPoints += viewPort.Count;
            // now, remove all those ways whose intersections only touch the viewport at a single point, but do not fully traverse it
            // as these should be treated as land polygons. Otherwise, breaks our sea polygon creation logic and get endless loops
            foreach (Way<Point> way in waysShouldBeLandPolygons)
            {
                coastWays.Remove(way);
                if (way.IsClosedWay())
                {
                    islands.Add(way);
                }
            }

            CleanOutsideSinglePointIntersections(pointsToIntersectingWays, viewPort, unmodifiedViewPort, allIntersections);
            bool keepTrying = true;
            int startingIdx = 0;
            Way<Point> startingWay = null;
            bool followViewPort = false;
            Way<Point> origViewPort = new Way<Point>(viewPort);
            // start at 0, or the first intersection, if there is one. Always starting at 0 means we need to backtrack, which
            // leads to unnecessarily complicated code
            for (int i = 0; i < viewPort.Count; i++)
            {
                Point p = viewPort[i];
                if (allIntersections.Contains(p))
                {
                    startingWay = pointsToIntersectingWays[p];
                    startingIdx = startingWay.IndexOf(p);
                    break;
                }
            }

            int numRetries = 0;
            while (keepTrying)
            {
                keepTrying = false;
                bool newPolys = false;
                try
                {
                    newPolys = TryToBuildPolygons(polygons, pointsToIntersectingWays, startingWay, viewPort, origViewPort, startingIdx, followViewPort, allIntersections);
                }
                catch (Exception e)
                {
                    throw e;
                }
                if (!newPolys)
                {
                    numRetries++;
                    keepTrying = true;
                }
            }
            // reset for next round
            keepTrying = true;
            startingWay = viewPort;
            startingIdx = 0;

            return polygons;
        }

        // TODO: the array of List<Way<Point>> is ugly. Find another way to represent the different layers representing alternating land and water.
        public static void CreatePolygons(List<Way<Point>> coastWaterPolygons, List<Way<Point>> islands, List<Way<Point>> inlandPolygons, string coastXML, string waterXML, Way<Point> viewPort)
        {
            OSMXMLParser waterParser = new OSMXMLParser(waterXML);
            List<Way<Point>> waterWays = waterParser.GetWays(true);
            // frees memory and calls GC.Collect and GC.WaitForPendingFinalizers
            waterParser.ForceFreeMemory();

            OSMXMLParser coastParser = new OSMXMLParser(coastXML);
            List<Way<Point>> coastWays = coastParser.GetWays(true);
            // frees memory and calls GC.Collect and GC.WaitForPendingFinalizers
            coastParser.ForceFreeMemory();

            foreach (Way<Point> way in waterWays)
            {
                // non coast water ways should always be closed polygons
                // TODO: is this always true? Lake erie, for example, is not closed for some reason.
                // Not sure if this is bug with your code trying to form water multipolygons, or just something
                // with the OSM data
                if (!way.IsClosedWay())
                {
                    way.Add(way[way.Count - 1]);
                }
                inlandPolygons.Add(way);
            }

            List<Way<Point>> mergedCoasts = MergeCoastLines(coastWays);

            GC.Collect();
            GC.WaitForPendingFinalizers();

            List<Way<Point>> mergedWaterPolys = CoastWaysToPolygon(mergedCoasts, viewPort, islands, inlandPolygons);
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
                    islands.Add(way);
                }
            }
        }

        private static List<Way<Point>> MergeCoastLines(List<Way<Point>> coastWays)
        {
            OSMXMLParser.MergeMultipolygonWays(coastWays);

            return coastWays;
        }

        public static void GetPolygons(List<Way<Point>> coastWaterPolygons, List<Way<Point>> islands, List<Way<Point>> inlandPolygons, DownloadArea d, Way<Point> viewPort, string saveLoc)
        {
            string coastXML = DownloadOsmCoastData(d, saveLoc + Path.DirectorySeparatorChar + "coast.osm");
            string waterXML = DownloadOsmWaterData(d, saveLoc + Path.DirectorySeparatorChar + "water.osm");

            CreatePolygons(coastWaterPolygons, islands, inlandPolygons, coastXML, waterXML, viewPort);
        }

        public static Point LatLongToPixel(Point latLong, decimal startLat, decimal startLong, decimal pixelsPerLongitude, decimal pixelsPerLatitude)
        {
            Point pixel = new Point(pixelsPerLongitude * (latLong.X - startLong), pixelsPerLatitude * (startLat - latLong.Y));

            return pixel;
        }

        public static Point CoordToPixel(decimal lat, decimal lon, decimal NWLat, decimal NWLon, decimal pixelsPerLongitude,
                                    decimal pixelsPerLatitude)
        {
            Point tempCoord = new Point(lon, lat);
            Point pixel = LatLongToPixel(tempCoord, NWLat, NWLon, pixelsPerLongitude, pixelsPerLatitude);
            pixel.X += xOffset;
            pixel.Y += yOffset;

            return pixel;
        }

        public static void DrawPolygon(Graphics g, SolidBrush b, decimal pixelsPerLon, decimal pixelsPerLat, AutomaticWaterMasking.Point NW, Way<AutomaticWaterMasking.Point> way)
        {
            if (way.Count == 0)
            {
                return;
            }

            List<PointF> l = new List<PointF>();
            for (int i = 0; i < way.Count - 1; i++) // FillPolygon polygons don't need last AutomaticWaterMasking.Point, hence - 1
            {
                AutomaticWaterMasking.Point p = way[i];
                AutomaticWaterMasking.Point pixel = WaterMasking.CoordToPixel(p.Y, p.X, NW.Y, NW.X, pixelsPerLon, pixelsPerLat);

                l.Add(new PointF((float)pixel.X, (float)pixel.Y));
            }
            PointF[] pf = l.ToArray();
            g.FillPolygon(b, pf);

        }

        public static void DrawPolygons(Bitmap bmp, Graphics g, SolidBrush b, decimal pixelsPerLon, decimal pixelsPerLat, AutomaticWaterMasking.Point NW, List<Way<AutomaticWaterMasking.Point>> polygons)
        {
            foreach (Way<AutomaticWaterMasking.Point> way in polygons)
            {
                DrawPolygon(g, b, pixelsPerLon, pixelsPerLat, NW, way);
            }
        }

        public static void DrawInlandPolys(List<Way<AutomaticWaterMasking.Point>> polys, Bitmap bmp, Graphics g, AutomaticWaterMasking.Point NW, decimal pixelsPerLon, decimal pixelsPerLat)
        {
            SolidBrush b = null;

            // sort from biggest to smallest
            polys.Sort(delegate (Way<AutomaticWaterMasking.Point> way1, Way<AutomaticWaterMasking.Point> way2)
            {
                // credit: https://stackoverflow.com/questions/2034540/calculating-area-of-irregular-polygon-in-c-sharp
                decimal area1 = way1.Area();
                decimal area2 = way2.Area();
                if (area1 - area2 > 0)
                {
                    return -1;
                }
                if (area1 - area2 < 0)
                {
                    return 1;
                }

                return 0;
            });

            foreach (Way<AutomaticWaterMasking.Point> way in polys)
            {
                if (way.relation == null || way.relation == "outer")
                {
                    b = new SolidBrush(Color.Black);
                    DrawPolygon(g, b, pixelsPerLon, pixelsPerLat, NW, way);
                }
                else if (way.relation == "inner")
                {
                    b = new SolidBrush(Color.White);
                    DrawPolygon(g, b, pixelsPerLon, pixelsPerLat, NW, way);
                }
                else
                {
                    throw new Exception("Unknown way relation");
                }
            }
        }

        private static void ClampPixelToImg(AutomaticWaterMasking.Point pixel, Bitmap bmp)
        {
            if (pixel.X < 0)
            {
                pixel.X = 0;
            }
            if (pixel.X >= bmp.Width)
            {
                pixel.X = bmp.Width - 1;
            }
            if (pixel.Y < 0)
            {
                pixel.Y = 0;
            }
            if (pixel.Y >= bmp.Height)
            {
                pixel.Y = bmp.Height - 1;
            }
        }

        private static bool TileAdjacentToWater(double[] tile, double[] checkTile, AutomaticWaterMasking.Point NW, decimal pixelsPerLon, decimal pixelsPerLat, Bitmap bmp)
        {
            // checkTile is to the north
            if (checkTile[0] > tile[0])
            {
                AutomaticWaterMasking.Point pixel = WaterMasking.CoordToPixel((decimal)checkTile[0], (decimal)checkTile[1], NW.Y, NW.X, pixelsPerLon, pixelsPerLat);
                pixel.Y--;
                ClampPixelToImg(pixel, bmp);
                return bmp.GetPixel((int)pixel.X, (int)pixel.Y).ToArgb() == Color.Black.ToArgb();
            }
            // checkTile is to the south
            if (checkTile[0] < tile[0])
            {
                AutomaticWaterMasking.Point pixel = WaterMasking.CoordToPixel((decimal)tile[0], (decimal)tile[1], NW.Y, NW.X, pixelsPerLon, pixelsPerLat);
                pixel.Y++;
                ClampPixelToImg(pixel, bmp);
                return bmp.GetPixel((int)pixel.X, (int)pixel.Y).ToArgb() == Color.Black.ToArgb();
            }
            // checkTile is to the east
            if (checkTile[1] > tile[1])
            {
                AutomaticWaterMasking.Point pixel = WaterMasking.CoordToPixel((decimal)checkTile[0], (decimal)checkTile[1], NW.Y, NW.X, pixelsPerLon, pixelsPerLat);
                pixel.X++;
                ClampPixelToImg(pixel, bmp);
                return bmp.GetPixel((int)pixel.X, (int)pixel.Y).ToArgb() == Color.Black.ToArgb();
            }
            // checkTile is to the west
            if (checkTile[1] < tile[1])
            {
                AutomaticWaterMasking.Point pixel = WaterMasking.CoordToPixel((decimal)tile[0], (decimal)tile[1], NW.Y, NW.X, pixelsPerLon, pixelsPerLat);
                pixel.X--;
                ClampPixelToImg(pixel, bmp);
                return bmp.GetPixel((int)pixel.X, (int)pixel.Y).ToArgb() == Color.Black.ToArgb();
            }

            return false;
        }

        // takes a tile which might be all land or all ocean water, and checks adjacent tiles to tell which is the truth
        private static bool AmbiguousTileHasSeaWater(double[] tile, Dictionary<double[], MaskingPolys> tilePolysMap, AutomaticWaterMasking.Point NW, decimal pixelsPerLon, decimal pixelsPerLat, Bitmap bmp)
        {
            MaskingPolys thisTileMaskingPolys = tilePolysMap[tile];
            List<double[]> allTiles = tilePolysMap.Keys.ToList();

            // has islands? the base of the tile should be water as a base(black)
            if (thisTileMaskingPolys.islands.Count > 0)
            {
                return true;
            }

            // by now, coastwater polys are 0. if we have some inland polys, then this tile should be land as a base (white)
            if (thisTileMaskingPolys.inlandPolygons.Count > 1)
            {
                return false;
            }

            // no coast water polys, no inland polys (think the middle of the dessert), so look at adjacent tiles
            allTiles.Sort(delegate (double[] tile1, double[] tile2)
            {
                if (tile1[0] > tile2[0])
                {
                    return 1;
                }
                if (tile1[0] < tile2[0])
                {
                    return -1;
                }
                // lat is equal so look at lon
                if (tile1[1] > tile2[1])
                {
                    return 1;
                }
                if (tile1[1] < tile2[1])
                {
                    return -1;
                }

                return 0;
            });

            for (int i = 0; i < allTiles.Count; i++)
            {
                double[] checkTile = allTiles[i];
                MaskingPolys mp = tilePolysMap[checkTile];
                // only use this tile to check for ambiguity if this tile has coast water polygons...
                if (mp.coastWaterPolygons.Count > 0 || mp.islands.Count > 0)
                {
                    if (TileAdjacentToWater(tile, checkTile, NW, pixelsPerLon, pixelsPerLat, bmp))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        public static List<Way<AutomaticWaterMasking.Point>> GetUniqueInlandWays(List<Way<AutomaticWaterMasking.Point>> allWays)
        {
            Dictionary<int, List<Way<AutomaticWaterMasking.Point>>> uniqueInlandPolysDict = new Dictionary<int, List<Way<AutomaticWaterMasking.Point>>>();
            if (allWays.Count > 0)
            {
                // pre-populate so below loop runs correctly
                List<Way<AutomaticWaterMasking.Point>> polysWithThisCount = new List<Way<AutomaticWaterMasking.Point>>();
                polysWithThisCount.Add(allWays[0]);
                uniqueInlandPolysDict.Add(allWays[0].Count, polysWithThisCount);
                foreach (Way<AutomaticWaterMasking.Point> inlandPoly in allWays)
                {
                    if (uniqueInlandPolysDict.ContainsKey(inlandPoly.Count))
                    {
                        List<Way<AutomaticWaterMasking.Point>> potentialMatches = uniqueInlandPolysDict[inlandPoly.Count];
                        foreach (Way<AutomaticWaterMasking.Point> alreadyThere in potentialMatches)
                        {
                            if (inlandPoly.DeepEquals(alreadyThere))
                            {
                                break;
                            }
                        }

                        // not there
                        potentialMatches.Add(inlandPoly);
                    }
                    else
                    {
                        polysWithThisCount = new List<Way<AutomaticWaterMasking.Point>>();
                        polysWithThisCount.Add(inlandPoly);
                        uniqueInlandPolysDict.Add(inlandPoly.Count, polysWithThisCount);
                    }

                }
            }
            List<Way<AutomaticWaterMasking.Point>> uniqueInlandPolysList = new List<Way<AutomaticWaterMasking.Point>>();
            foreach (KeyValuePair<int, List<Way<AutomaticWaterMasking.Point>>> kv in uniqueInlandPolysDict)
            {
                List<Way<AutomaticWaterMasking.Point>> polys = kv.Value;
                foreach (Way<AutomaticWaterMasking.Point> way in polys)
                {
                    uniqueInlandPolysList.Add(way);
                }
            }

            return uniqueInlandPolysList;
        }

        public static Bitmap GetMask(Dictionary<double[], MaskingPolys> allMaskingPolys, int pixelsX, int pixelsY, AutomaticWaterMasking.Point NW, decimal pixelsPerLon, decimal pixelsPerLat, Graphics g = null, Bitmap bmp = null)
        {
            bool createdNewGraphics = false;
            SolidBrush b = null;
            if (bmp == null)
            {
                bmp = new Bitmap(pixelsX, pixelsY, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
                g = Graphics.FromImage(bmp);
                g.FillRectangle(Brushes.White, 0, 0, bmp.Width, bmp.Height);
                createdNewGraphics = true;
            }
            Dictionary<double[], MaskingPolys> ambiguousTiles = new Dictionary<double[], MaskingPolys>();
            List<Way<AutomaticWaterMasking.Point>> allInlandPolys = new List<Way<AutomaticWaterMasking.Point>>();
            foreach (KeyValuePair<double[], MaskingPolys> kv in allMaskingPolys)
            {

                MaskingPolys polys = kv.Value;
                double[] tile = kv.Key;
                // first, draw the coast water polygons
                if (polys.coastWaterPolygons.Count > 0)
                {
                    b = new SolidBrush(Color.Black);
                    WaterMasking.DrawPolygons(bmp, g, b, pixelsPerLon, pixelsPerLat, NW, polys.coastWaterPolygons);
                }
                else
                {
                    // but not the ones in tiles with no coast water polygon
                    ambiguousTiles.Add(tile, polys);
                }

                // now draw the islands
                b = new SolidBrush(Color.White);
                WaterMasking.DrawPolygons(bmp, g, b, pixelsPerLon, pixelsPerLat, NW, polys.islands);
                foreach (Way<AutomaticWaterMasking.Point> way in polys.inlandPolygons)
                {
                    allInlandPolys.Add(way);
                }
            }

            List<Way<AutomaticWaterMasking.Point>> uniqueInlandPolys = GetUniqueInlandWays(allInlandPolys);
            // now, draw the layeredpolygons
            WaterMasking.DrawInlandPolys(uniqueInlandPolys, bmp, g, NW, pixelsPerLon, pixelsPerLat);

            // now handle supicious tiles which are potentially in middle of ocean without a coast intersecting viewport or are all land with no water
            foreach (KeyValuePair<double[], MaskingPolys> kv in ambiguousTiles)
            {
                List<Way<AutomaticWaterMasking.Point>> coastWaterPolygons = new List<Way<AutomaticWaterMasking.Point>>();
                double[] tile = kv.Key;
                MaskingPolys polys = kv.Value;
                if (AmbiguousTileHasSeaWater(tile, allMaskingPolys, NW, pixelsPerLon, pixelsPerLat, bmp))
                {
                    // if all water, draw the extent of this tile as all black. if not all water, the extent will already be all white...
                    Way<AutomaticWaterMasking.Point> tileExtent = new Way<AutomaticWaterMasking.Point>();
                    tileExtent.Add(new AutomaticWaterMasking.Point((decimal)tile[1], (decimal)(tile[0] + 1)));
                    tileExtent.Add(new AutomaticWaterMasking.Point((decimal)tile[1] + 1, (decimal)(tile[0] + 1)));
                    tileExtent.Add(new AutomaticWaterMasking.Point((decimal)(tile[1] + 1), (decimal)tile[0]));
                    tileExtent.Add(new AutomaticWaterMasking.Point((decimal)tile[1], (decimal)tile[0]));
                    tileExtent.Add(new AutomaticWaterMasking.Point((decimal)tile[1], (decimal)(tile[0] + 1)));
                    coastWaterPolygons.Add(tileExtent);
                    b = new SolidBrush(Color.Black);
                    WaterMasking.DrawPolygons(bmp, g, b, pixelsPerLon, pixelsPerLat, NW, coastWaterPolygons);
                    // redraw the islands
                    b = new SolidBrush(Color.White);
                    WaterMasking.DrawPolygons(bmp, g, b, pixelsPerLon, pixelsPerLat, NW, polys.islands);
                    // redraw the layered polygons for this tile
                    WaterMasking.DrawInlandPolys(uniqueInlandPolys, bmp, g, NW, pixelsPerLon, pixelsPerLat);
                }
            }

            if (createdNewGraphics)
            {
                g.Dispose();
            }

            return bmp;
        }
    }
}
