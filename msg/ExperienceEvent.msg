Header header

#Captured image and BoF histogram
sensor_msgs/Image image_
float64[] histogram_

# Active Local View Cells
uint32[] lv_cell_id_
float64[] lv_cell_rate_

# Place Cell index (x,y,z) and rate
uint32[] pc_index_
float64 pc_rate_
PlaceCellActivity pc_activity_

# Distance traveled since last experience
geometry_msgs/Point traveled_distance_
geometry_msgs/Point ground_truth_


