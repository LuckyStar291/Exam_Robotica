# Exemen-Robotica-1
Examen de Robotica semestre 2-2025

Integrantes: 
Adrian Fuentes Castillo
Alejandro Miranda Saravia
Luz Maria Soria Carvajal 


EXERCISE 1


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

EXERCISE 2
This package contains a ROS2 Python implementation for a sensor data processing system, designed as part of a robotics assignment. It includes publisher nodes for three sensors, a filter node to compute the average, and a display node to show the results.

Overview

Sensors: Three nodes (sensor1, sensor2, sensor3) publish random float values (0.0 to 10.0) on topics /sensor_1, /sensor_2, and /sensor_3 respectively, at a 0.5-second interval.
Filter: A node (filter) subscribes to the sensor topics, calculates the average, and publishes the result on /filtered_sensor using a custom FilteredSensor message (with float64 sensor_value and string name).
Display: A node (display) subscribes to /filtered_sensor and logs the average value in real-time.

Install:
Put in the workspace.
Run: colcon build && source install/setup.bash.
Run:
ros2 run py_pubsub sensor1
ros2 run py_pubsub sensor2
ros2 run py_pubsub sensor3
ros2 run py_pubsub filter
ros2 run py_pubsub display

