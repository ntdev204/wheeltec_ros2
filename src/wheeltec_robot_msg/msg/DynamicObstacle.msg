std_msgs/Header header
int32 track_id
string class_name
geometry_msgs/Point position            # 3D position in map frame
geometry_msgs/Vector3 velocity          # m/s in map frame
geometry_msgs/Vector3 predicted_position  # predicted position in 1s
float32 radius                          # obstacle radius (m)
float32 confidence
