# Single-sensor target detection, published by each tracker node
std_msgs/Header header
string sensor_id
bool valid
geometry_msgs/Point position
common_msgs/Matrix2 covariance
