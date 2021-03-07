import xml.etree.ElementTree as ET


def newModel(world, modelName: str):
    model = ET.SubElement(world, "model")
    model.set("name", modelName)
    static = ET.SubElement(model, "static")
    static.text = "1"
    return model