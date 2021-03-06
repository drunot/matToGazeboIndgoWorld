import xml.etree.ElementTree as ET


def createWorld(sdfHead):
    world = ET.SubElement(sdfHead, "world")
    world.set("name", "default")
    sun = ET.SubElement(world, "include")
    sunURI = ET.SubElement(sun, "uri")
    sunURI.text = "model://sun"
    ground = ET.SubElement(world, "include")
    groundURI = ET.SubElement(ground, "uri")
    groundURI.text = "model://ground_plane"
    return world