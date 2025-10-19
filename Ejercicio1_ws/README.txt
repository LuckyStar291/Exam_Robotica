For the exercises in the first part of the exam (three sections), you must run the following commands in two separate terminals for each section.
Make sure not to run all exercises at the same time, as this can cause conflicts between nodes.

(a) Implement the visualization and inverse kinematics control for the index finger.
Terminal 1: 
- colcon build
- source install/setup.bash
- ros2 launch robot_description view_robot.launch.py
Terminal 2:
- colcon build
- source install/setup.bash
- ros2 run visual_pubsub inverse_k

(b) Implement the visualization and inverse kinematics control for the finger.
Terminal 1:
- colcon build
- source install/setup.bash
- ros2 launch robot_description view_robot.launch2.py
Terminal 2:
- colcon build
- source install/setup.bash
- ros2 run visual_pubsub inverse_k2

(c) Implement the visualization and inverse kinematics control for both the index finger and the thumb in a single URDF.
Terminal 1: 
- colcon build
- source install/setup.bash
- ros2 launch robot_description view_robot.launch3.py
Terminal 2: 
- colcon build
- source install/setup.bash
- ros2 run visual_pubsub inverse_k3
