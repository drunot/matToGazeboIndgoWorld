import xml.etree.ElementTree as ET
import math


def createWall(
    startx: int,
    starty: int,
    endx: int,
    endy: int,
    model,
    name: str,
    offsetX: int = 0,
    offsetY: int = 0,
):
    # Make sure end is the largest,
    if startx > endx:
        startx, endx = endx, startx
    if starty > endy:
        starty, endy = endy, starty
    endx += 1
    endy += 1
    zRot = 0
    rotated = False
    # If x is smaller than y rortate 90
    if endx - startx > endy - starty:
        zRot = math.pi / 2
        rotated = True

    link = ET.SubElement(model, "link")
    link.set("name", name)
    collision = ET.SubElement(link, "collision")
    collision.set("name", name + "_Collision")
    cgeometry = ET.SubElement(collision, "geometry")
    cBox = ET.SubElement(cgeometry, "box")
    cSize = ET.SubElement(cBox, "size")
    if not rotated:
        X = startx - endx
        Y = starty - endy
    else:
        Y = startx - endx
        X = starty - endy
    cSize.text = f"{X} {Y} 2.5"
    cPose = ET.SubElement(collision, "pose")
    cPose.text = "0 0 1.25 0 -0 0"

    visual = ET.SubElement(link, "visual")
    visual.set("name", name + "_Visual")
    vPose = ET.SubElement(visual, "pose")
    vPose.text = "0 0 1.25 0 -0 0"
    vgeometry = ET.SubElement(visual, "geometry")
    vBox = ET.SubElement(vgeometry, "box")
    vSize = ET.SubElement(vBox, "size")
    vSize.text = f"{X} {Y} 2.5"
    # Material
    material = ET.SubElement(visual, "material")
    mScritp = ET.SubElement(material, "script")
    mUri = ET.SubElement(mScritp, "uri")
    mName = ET.SubElement(mScritp, "name")
    mUri.text = "file://media/materials/scripts/gazebo.material"
    mName.text = "Gazebo/White"

    velocity_decay = ET.SubElement(link, "velocity_decay")
    vdLinear = ET.SubElement(velocity_decay, "linear")
    vdLinear.text = "0"
    vdAngular = ET.SubElement(velocity_decay, "angular")
    vdAngular.text = "0"
    pose = ET.SubElement(link, "pose")
    pose.text = f"{startx + (endx-startx)/2 - offsetX} {starty + (endy-starty)/2 - offsetY} 0 0 -0 {zRot}"