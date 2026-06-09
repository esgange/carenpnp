std_msgs/Header header
builtin_interfaces/Time image_stamp
uint32 image_width
uint32 image_height
int32[] ids
geometry_msgs/Pose[] poses
geometry_msgs/Point[] pixel_centers
geometry_msgs/Polygon[] pixel_corners
# Depth-derived 3D points in header.frame_id, one entry per marker.
geometry_msgs/Point[] camera_centers
geometry_msgs/Polygon[] camera_corners
