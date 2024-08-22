Firstly, the system needs to run on Unbuntu22.04

1 sudo apt update

2 sudo apt install ros - humble - desktop

3 sudo apt install ros - humble - turtlebot3 *

4 sudo apt install python3 - colcon - common - extensions

5 sudo apt install ros - humble - rosbridge - server

6 sudo apt install ros - humble - gzebo *

7 sudo apt install python3

8 sudo apt install openjdk -17 - jdk

Install Jason

Download the Jason platform from the official website: Jason Download.

Unzip the downloaded file to your preferred directory.

Ensure that jasn.jar is included in the class path:

1 export CLASSPATH = $CLASSPATH :/ path / to / jason / jason . jar

Set Up the TurtleBot3 Simulation:

1 export TURTLEBOT3_MODEL = burger

2 source / opt / ros / humble / setup . bash

3 ros2 launch turtlebot3_gazebo turtlebot3_world . launch . py

A.2 Case1–Turtlebot World

Launch ROSBridge Server:

1 ros2 launch rosbridge_server rosbridge_websocket_launch . xml

Set Up the TurtleBot3 Simulation:

1 export TURTLEBOT3_MODEL = burger

2 ros2 launch turtlebot3_gazebo turtlebot3_world . launch . py

A.3. CASE2–TURTLEBOT HOUSE 57

Jason

Run Jason code:

1 ./ gradlew

Python

Run python code:

1 python3 agent . py

A.3 Case2–Turtlebot House

Build the Workspace and source the setup file,then launch the Gazebo world with multiple Turtle-

bot. But you must make sure to execute the following command in the ’agentw_s path’

1 colcon build -- symlink - install

2 source ./ install / setup . bash

3 ros2 launch turtlebot3_multi_robot main . launch . py

Launch ROSBridge Server:

1 ros2 launch rosbridge_server rosbridge_websocket_launch . xml

Jason

Run jason code

1 ./ gradlew

Python

Run python code:

1 python3 launch . py
