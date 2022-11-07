##This script converts a POI from poly. 
# Generate POIS using the poi_alongRoads call 
import argparse
import math
import xml.etree.ElementTree as ET

"""
Usage example 
python "C:\Program Files (x86)\Eclipse\Sumo\tools\shapes\poi_alongRoads.py" .\tenWest.net.xml [123106011,30321913,528363194,976016924,528320137] 100
python .\poi2poly.py .\pois.add.xml
Now copy contents of add.poly.xml to the poly file being used by sumo
"""

def setCircle(idx, x, y, r, c, prefix, type, color, fill, layer, output):
    angle = 2 * math.pi / c
    shape = ""
    for i in range(c):
        shape += "%.2f,%.2f " % (math.cos(i * angle) * r + x,
                                 math.sin(i * angle) * r + y)
        if i == 0:
            beginPoint = shape
    shape += beginPoint
    print('    <poly id="%s%s" type="%s" color="%s" fill="%i" layer="%s" shape="%s"/>' % (
        prefix, idx, type, color, fill, layer, shape[:-1]),
        file=output)

def convert_poi_poly(fname, outfile="add.poly.xml"):
    f = open(outfile, "w")
    tree = ET.parse(fname)
    root = tree.getroot()
    for i, child in enumerate(root):
        idx = str(i)
        prefix = "poi"
        type = "poi"
        color = child.attrib["color"]
        fill = 1
        layer = 5
        x = float(child.attrib["x"]) 
        y = float(child.attrib["y"])
        r = 1
        c = 100
        setCircle(idx + "a", x, y + 12, r, c, prefix, type, color, fill, layer, f)
        setCircle(idx + "b", x, y - 12, r, c, prefix, type, color, fill, layer, f)
    f.close()

def convert_poi_rsu(fname):
    # Converts poi positions to RSUs
    tree = ET.parse(fname)
    root = tree.getroot()
    for i, child in enumerate(root):
        x = float(child.attrib["x"]) 
        y = float(child.attrib["y"])
        print("{},{},15;".format(y+12, x))
        print("{},{},15;".format(y-12, x))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
                    prog = 'poi2poly',
                    description = 'Converts poi xml file from poi_along roads to polygon file ',
                    epilog = 'usage:\n python convert_poi_poly abs_file_path.xml')
    parser.add_argument('filename')
    args = parser.parse_args()
    print(args.filename)
    convert_poi_poly(args.filename)
    convert_poi_rsu(args.filename)