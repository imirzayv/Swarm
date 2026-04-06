uint8 MODE_EXPLORE=0
uint8 MODE_EXPLOIT=1

uint8 mode                    # EXPLORE or EXPLOIT
uint8[] exploit_drone_ids     # drones assigned to exploitation
string event_class            # class that triggered exploit mode (e.g., "person", "vehicle", "fire")
float64[3] event_position     # [x, y, z] of the triggering detection in local frame
float32 event_confidence      # confidence of the triggering detection
builtin_interfaces/Time stamp
