# Voblox Multi Agent
To run the standard Voxblox nodes (single agents), use: `esdf_server` and `tsdf_server`  
The nodes that support the multi robot pose graph backend are: `esdf_pose_graph_server` and `tsdf_pose_graph_server`

## Remarks

This is the voxblox used for the radiation mapping. In this branch, the waypoints for the sampled peaks to revisit are creating using the first gradient for each step walking away from the identified peak location. This returns meaningful waypoints in a more stable manner (= waypoints that make sense/not shifted from the peak). 
