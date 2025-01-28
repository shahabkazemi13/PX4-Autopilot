# uORB message definition for load cell data
# This message is used to store 6-axis load cell measurements

float32[6] axis_data  # Load cell measurements along 6 axes
uint64 timestamp      # Timestamp in microseconds (from hrt_absolute_time())
