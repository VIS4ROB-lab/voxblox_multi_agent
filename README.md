# Voblox Multi Agent
To run the standard Voxblox nodes (single agents), use: `esdf_server` and `tsdf_server`  
The nodes that support the multi robot pose graph backend are: `esdf_pose_graph_server` and `tsdf_pose_graph_server`

## Remarks

This is the voxblox used for the radiation mapping. In this branch, the waypoints for the sampled peaks to revisit are creating using different gradients for each step walking away from the identified peak location.
Even though, this feature makes logical sense, it has sometimes (so far) unexplainable behaviour. The user is advised to use the `devel/radiation_mapping` branch.
