int32 mnId
float64 mTimeStamp

# Pose
geometry_msgs/Pose pose_cw

# Descriptor Vector
cloud_edge_slam/Descriptor[] descriptors
cloud_edge_slam/KeyPoint[] key_points

# MapPoint
int64[] mvp_map_points_index
