import xml.etree.ElementTree as ET


def createSDF():
    sdfHead = ET.Element("sdf")
    sdfHead.set("version", "1.4")
    return sdfHead