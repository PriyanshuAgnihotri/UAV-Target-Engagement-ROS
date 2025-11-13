# UAV Target Engagement ROS Package

This ROS package demonstrates a **vision-guided UAV mission** that detects, tracks, and engages a target using simulated logic.

## Repository Structure
```
UAV_Target_Engagement_ROS/
├── launch/
│   └── uav_mission.launch
├── models/
│   └── tank_model/
│       └── model.sdf
├── src/
│   ├── uav_control_node.py
│   ├── computer_vision_node.py
│   └── mission_planner_node.py
└── README.md
```
See the assignment write-up for full explanation of the workflow, P-controller logic, and engagement behavior.
