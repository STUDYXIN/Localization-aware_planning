# ErrorOutputWithTimestamp.msg

# 时间戳
std_msgs/Header header

# 数据
float64 stereo_error
float64 frame_track_error
std_msgs/Int32 feature_num
# 位置
geometry_msgs/PoseWithCovariance pose_truth
# 速度
geometry_msgs/TwistWithCovariance twist_truth
