# Field frame: origin (0,0) at one corner; x 0..1880 mm, y 0..1410 mm (match server/map).
# Default topic: /pinky_{ROS_DOMAIN_ID}/gps_pos. pinky_id: optional filter / legacy; bridge may set from domain.
std_msgs/Header header
uint8 pinky_id
float64 x_mm
float64 y_mm
float64 yaw_deg
uint8 confidence   # 0=not detected, 1..100=confidence percent
bool is_valid      # marker detected for this pinky_id
