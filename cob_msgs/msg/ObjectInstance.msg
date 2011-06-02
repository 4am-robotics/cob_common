# This message contains an object instance
#

std_msgs/Header header        

# definite object number
uint32 objectID

# definite class number which the object belongs to - redundant, but here for convenience
uint32 classID

# Similar to object ID, but in a more human understandable manner
string objectName

# object pose
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw

