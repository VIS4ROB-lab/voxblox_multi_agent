# Multi-Agent Voxblox
This repository contains a multi-agent version of [Voxblox](https://github.com/ethz-asl/voxblox). It is based on the original Voxblox implementation, with the additional capacity of fusing the sensory data from multiple robots.  
This repository is used in a wider framework for multi-robot path planning, available [here](https://github.com/VIS4ROB-lab/multi_robot_coordination).  

If you use this code in your academic work, please cite ([PDF](https://www.research-collection.ethz.ch/handle/20.500.11850/441280)):

    @inproceedings{bartolomei2020multi,
      title={Multi-robot Coordination with Agent-Server Architecture for Autonomous Navigation in Partially Unknown Environments},
      author={Bartolomei, Luca and Karrer, Marco and Chli, Margarita},
      booktitle={2020 {IEEE/RSJ} International Conference on Intelligent Robots and Systems ({IROS})},
      year={2020}
    }

## Installation
To install the multi-agent version of Voxblox, follow [these instructions](https://voxblox.readthedocs.io/en/latest/pages/Installation.html). In addition, clone the following repository:
```
$ git clone git@github.com:VIS4ROB-lab/comm_msgs.git # with ssh
$ git clone https://github.com/VIS4ROB-lab/comm_msgs.git # with https
```

## ROS Nodes
The multi-agent version of Voxblox allows to use both the standard implementation and the modified one. In particular:
* To run the standard Voxblox nodes (single agents), use the executables: `esdf_server` and `tsdf_server`  
* The executables that support the multi-robot pose-graph backend are: `esdf_pose_graph_server` and `tsdf_pose_graph_server`
