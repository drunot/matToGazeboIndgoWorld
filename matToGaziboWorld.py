import xml.etree.ElementTree as ET
import scipy.io
import math
import mat2rect
import sys


def createWall(
    startx: int,
    starty: int,
    endx: int,
    endy: int,
    model,
    name: str,
    initX: int = 0,
    initY: int = 0,
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
    pose.text = f"{startx + (endx-startx)/2 - initX} {starty + (endy-starty)/2 - initY} 0 0 -0 {zRot}"


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Takes 3 arguments: <mat-file-path> <init-pos-x,init-pos-y> <world-name>")
        exit(1)
    worldname = sys.argv[3]
    pos = sys.argv[2].split(",")
    try:
        initX = int(pos[0])
        initY = int(pos[1])
    except:
        print("Position must by in format: int,int")
        exit(1)

    # create the file structure
    sdfHead = ET.Element("sdf")
    sdfHead.set("version", "1.4")
    world = ET.SubElement(sdfHead, "world")
    world.set("name", "default")
    sun = ET.SubElement(world, "include")
    sunURI = ET.SubElement(sun, "uri")
    sunURI.text = "model://sun"
    ground = ET.SubElement(world, "include")
    groundURI = ET.SubElement(ground, "uri")
    groundURI.text = "model://ground_plane"
    model = ET.SubElement(world, "model")
    model.set("name", "test")
    static = ET.SubElement(model, "static")
    static.text = "1"
    try:
        mat = scipy.io.loadmat(sys.argv[1])
    except:
        print("Error in opening file")
        exit(1)

    rects = mat2rect.mat2rect(mat)

    for num, rect in enumerate(rects):
        createWall(
            rect.x_min,
            rect.y_min,
            rect.x_max,
            rect.y_max,
            model,
            f"Wall_{num}",
            initX,
            initY,
        )

    mydata = ET.tostring(sdfHead, encoding="utf-8", xml_declaration=True)
    myfile = open(f"turtlebot_mw_{worldname}.world", "wb")
    myfile.write(mydata)
    myfile.close()

    launchFile = """
    <launch>                                                                                                                                                                                                                                                                                                                                      
    <arg name="base"      value="$(optenv TURTLEBOT_BASE kobuki)"/> <!-- create, roomba -->                                                                                                                                                                                                                                                     
    <arg name="battery"   value="$(optenv TURTLEBOT_BATTERY /proc/acpi/battery/BAT0)"/>  <!-- /proc/acpi/battery/BAT0 -->                                                                                                                                                                                                                       
    <arg name="stacks"    value="$(optenv TURTLEBOT_STACKS hexagons)"/>  <!-- circles, hexagons -->                                                                                                                                                                                                                                             
    <arg name="3d_sensor" value="$(optenv TURTLEBOT_3D_SENSOR kinect)"/>  <!-- kinect, asus_xtion_pro -->                                                                                                                                                                                                                                       
                                                                                                                                                                                                                                                                                                                                                
    <include file="$(find gazebo_ros)/launch/empty_world.launch">                                                                                                                                                                                                                                                                               
        <arg name="use_sim_time" value="true"/>                                                                                                                                                                                                                                                                                                   
        <arg name="debug" value="false"/>                                                                                                                                                                                                                                                                                                         
        <arg name="world_name" value="$(find turtlebot_gazebo)/worlds/turtlebot_mw_{:}.world"/>                                                                                                                                                                                                                                                
    </include>                                                                                                                                                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                                                                                
    <include file="$(find turtlebot_gazebo)/launch/includes/$(arg base).launch.xml">                                                                                                                                                                                                                                                            
        <arg name="base" value="$(arg base)"/>                                                                                                                                                                                                                                                                                                    
        <arg name="stacks" value="$(arg stacks)"/>                                                                                                                                                                                                                                                                                                
        <arg name="3d_sensor" value="$(arg 3d_sensor)"/>                                                                                                                                                                                                                                                                                          
    </include>                                                                                                                                                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                                                                                
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">                                                                                                                                                                                                                                                
        <param name="publish_frequency" type="double" value="30.0" />                                                                                                                                                                                                                                                                             
    </node>                                                                                                                                                                                                                                                                                                                                     
                                                                                                                                                                                                                                                                                                                                                
    <!-- Fake laser -->                                                                                                                                                                                                                                                                                                                         
    <node pkg="nodelet" type="nodelet" name="laserscan_nodelet_manager" args="manager"/>
    <node pkg="nodelet" type="nodelet" name="depthimage_to_laserscan"
            args="load depthimage_to_laserscan/DepthImageToLaserScanNodelet laserscan_nodelet_manager">
        <param name="scan_height" value="10"/>
        <param name="output_frame_id" value="/camera_depth_frame"/>
        <param name="range_min" value="0.45"/>
        <remap from="image" to="/camera/depth/image_raw"/>
        <remap from="scan" to="/scan"/>
    </node>
    </launch>
    """.format(
        worldname
    )

    myfile = open(f"turtlebot_mw_{worldname}.launch", "w")
    myfile.write(launchFile)
    myfile.close()

    runScript = (
        "#!/bin/bash\nsource /opt/ros/indigo/setup.bash\nexport ROS_IP=$(ifconfig eth0 | grep \"inet addr\" | awk -F: '{print $2}' | awk '{print $1}')\n"
        + f"roslaunch turtlebot_gazebo turtlebot_mw_{worldname}.launch"
    )

    format(worldname)
    myfile = open(f"{worldname}_run.sh", "w")
    myfile.write(runScript)
    myfile.close()