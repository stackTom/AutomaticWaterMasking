# AutomaticWaterMasking
Takes OSM XML from OpenStreetMap (local files or direct download from OpenStreetMap), and outputs .OSM files describing closed polygons denoting land/water.
These output .OSM files can also be generated into water mask bitmaps.
# To generate bitmaps
Call GetPolygons, then pass the resultant lists to GetMask.
