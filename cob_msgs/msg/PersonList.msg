
Header header					#  sequence ID: consecutively increasing ID 
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
	
uint32 num_persons				#  number of detected Persons
PersonHypothesis[] list				#  comprised the detected persons
						#

uint32 num_associations				#  number of associations (number of persons currently in filter)
float32[] association_propability		#

uint32[] ID					#  this are the person IDs (unique identifier to associate persons with a data base - for image based recognition)

uint32[] filter_update				#  this table is for the filter update. if filter_update(i)==0 -> no filter update
