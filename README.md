# Multi-Agent Voxblox
This repository contains a multi-agent version of [Voxblox](https://github.com/ethz-asl/voxblox). It is based on the original Voxblox implementation, with the additional capacity of fusing the sensory data from multiple robots.  
This repository is used in a wider framework for multi-robot path planning, available [here](https://github.com/VIS4ROB-lab/multi_robot_coordination).  

If you use this Voxblox version in your academic work, please cite:
        _"Multi-robot Coordination with Agent-Server Architecture for Autonomous Navigation in Partially Unknown Environments"_ by Luca Bartolomei, Marco Karrer and Margarita Chli, IROS 2020.

# Installation
To install the multi-agent version of Voxblox, follow [these instructions](https://voxblox.readthedocs.io/en/latest/pages/Installation.html).

## ROS Nodes
The multi-agent version of Voxblox allows to use both the standard implementation and the modified one. In particular:
* To run the standard Voxblox nodes (single agents), use the executables: `esdf_server` and `tsdf_server`  
* The executables that support the multi-robot pose-graph backend are: `esdf_pose_graph_server` and `tsdf_pose_graph_server`
