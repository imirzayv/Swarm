uint8 drone_id
string class_name
float32 confidence
float32[4] bbox           # [x1, y1, x2, y2] in pixels
float64[3] world_position # [x, y, z] estimated ground position in local frame (meters)
builtin_interfaces/Time stamp
