# waypoint detection message

# header field
Header header

# number of waypoints detected
uint16 num_waypoints_detected

# image - list of x coordinates of detected waypoints within image
int32[] x_coords

# image - list of y coordinates of detected waypoints within image
int32[] y_coords

# image - list of radius's of detected waypoints within image
int32[] radii
