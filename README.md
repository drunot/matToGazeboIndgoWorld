# Mat To Gazebo world

This is a project that aims to make it easier to transfer a map made in matlab and into Gazebo Indigo. Please note that the only tested version of Gazebo used to test this with is the one linked to [here](https://se.mathworks.com/support/product/robotics/v3-installation-instructions.html) running in a virtual machine (VM) made by MatLab.

## How to use the script

### Prerequisites

- A python runtime is needed. We have used version 3.8 ealier versions not tested.

- Clone this respository to get all the files needed.

- Install required python liberaries. If you are using pip this can be done by writing:

```bash
python3 -m pip install -r requirements.txt
```

- Have a `*.mat` file in the correct format. Our script looks for a array in this mat file called `map`. Here will all wall tiles be equal to 1 and all floor tiles be equal to 0.

### Running the script

When running the script call:

```bash
ptyhon3 matToGazeboWorld.py <path-to-mat-file> <offsetX,offsetY> <world-name> (<scaling-divider>)
```

Here will the offset be the starting point for the turtlebot. It is important to notice tha the turtlebot will always start in 0. This offset moves the map so the walls is correct relatively to the turtlebot. The offset should be specified in one word with two whole numbers. An example with an offset of 4 in x and 2 in y would be: `4,2`.

World-name specifies what the name of the output files will be. The script outputs 3 files, with the names: `turtlebot_mw_<world-name>.launch`, `turtlebot_mw_<world-name>.world` and `<world-name>_run.sh`.

The scaling devider devides the whole map with the specified devision. So a 100x120 map with a devider of 5 will become a 20x24 map. Please note that the offset will not be devided.

### Using the output files

The three output files can be moved to the VM using secure copy. Once on the VM
the `*.launch`-file should be moved to the path `/opt/ros/indigo/share/turtlebot_gazebo/launch/`. The `*.world`-file whould be moved to `/opt/ros/indigo/share/turtlebot_gazebo/world/`. Now you can open gazebo with the given world by runnning the `*.sh`-file. Note that if you get an error running the script this might be because it was generated in Windows. To fix this simply run:

```bash
sed -i -e 's/\r$//' <script-name>.sh
```
