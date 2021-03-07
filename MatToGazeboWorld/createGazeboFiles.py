import xml.etree.ElementTree as ET


def createGazeboFiles(sdfHead, worldName: str):
    worldData = ET.tostring(sdfHead, encoding="utf-8", xml_declaration=True)
    worldFile = open(f"turtlebot_mw_{worldName}.world", "wb")
    worldFile.write(worldData)
    worldFile.close()

    launchData = """
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
        worldName
    )

    launchFile = open(f"turtlebot_mw_{worldName}.launch", "w")
    launchFile.write(launchData)
    launchFile.close()

    runScript = (
        "#!/bin/bash \nsource /opt/ros/indigo/setup.bash\nexport ROS_IP=$(ifconfig eth0 | grep \"inet addr\" | awk -F: '{print $2}' | awk '{print $1}')\n"
        + f"roslaunch turtlebot_gazebo turtlebot_mw_{worldName}.launch"
    )

    format(worldName)
    runFile = open(f"{worldName}_run.sh", "w")
    runFile.write(runScript)
    runFile.close()