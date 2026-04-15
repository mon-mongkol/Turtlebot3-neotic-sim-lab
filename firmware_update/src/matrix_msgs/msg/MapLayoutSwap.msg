string layout_name
string map_name

bool amcl_estimate

uint8 func_poi = 1
uint8 func_car = 2 #x, y, z on map_frame
uint8 func_poi_invert = 3 

uint8 estimate_func
string poi_es
geometry_msgs/Pose pose_es