

# header better in PersonList.msg
#Header header				#  sequence ID: consecutively increasing ID 
					## uint32 seq
					#  Two-integer timestamp that is expressed as:
					#  * stamp.secs: seconds (stamp_secs) since epoch
					#  * stamp.nsecs: nanoseconds since stamp_secs
					#  time-handling sugar is provided by the client library
					## time stamp
					#  Frame this data is associated with
					#  0: no frame
					#  1: global frame
					## string frame_id


############## position###########################################


geometry_msgs/PoseWithCovariance position		#  Pose pose
					#  Row-major representation of the 6x6 covariance matrix
					#  The orientation parameters use a fixed-axis representation.
					#  In order, the parameters are:
					#  (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
					#  float64[36] covariance

geometry_msgs/TwistWithCovariance rotation 		#  This expresses velocity in free space with uncertianty.
					# Row-major representation of the 6x6 covariance matrix
					# The orientation parameters use a fixed-axis representation.
					# In order, the parameters are:
					# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)

float32 p_f				#  probabillity_false = 1-w, w= probability
					#  reliability of the detection


