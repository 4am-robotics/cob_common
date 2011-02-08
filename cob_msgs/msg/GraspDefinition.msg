#message for exchanging information about grasps from the grasp table

#information about the grasped object
# definite object number
uint32 objectID

# Similar to object ID, but in a more human understandable manner
string objectName

#information about the grasp
#grasp index in the grasp table
uint32 index

#grasp quality
float32 quality

#grasp poses and configurations
#AbsPos of grasp_pose
cob_msgs/Transformation grasp_pose

#AbsPos of pre_grasp_pose
cob_msgs/Transformation pre_grasp_pose

#Configuration of hand at pre_grasp_pose
sensor_msgs/JointState hand_pre_grasp_config

#Configuration of hand at grasp_pose
sensor_msgs/JointState hand_grasp_config

#Configuration of hand at grasp_pose (optimal)
sensor_msgs/JointState hand_optimal_grasp_config

