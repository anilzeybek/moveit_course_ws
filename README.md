This project is code part of `Tech Support/ROS 2 MoveIt tutorial.md` in Obsidian.

Everything created in this order:
1. Create the URDF files of the robot under `src/my_robot_description/urdf/`
2. Create `src/my_robot_description/launch/display.launch.py` to easily visualize that URDF.
3. Use MoveIt setup assistant to generate `src/my_robot_moveit_config`.
4. Create `src/my_robot_bringup/launch/my_robot.launch.py` to be able to plan with moveit in RViz and make it ready to be commanded.
5. Create `src/my_robot_commander/src/test_moveit.cpp` to control robot with MoveIt through Python.
