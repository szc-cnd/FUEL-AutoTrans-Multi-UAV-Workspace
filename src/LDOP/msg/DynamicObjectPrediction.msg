uint8 CLASS_UNKNOWN=0
uint8 CLASS_HUMAN=1
uint8 CLASS_VEHICLE=2
uint8 CLASS_UAV=3
uint8 CLASS_OTHER=4

uint8 MOTION_MODEL_CA2D=0
uint8 MOTION_MODEL_CA3D=1
uint8 MOTION_MODEL_CV3D=2
uint8 MOTION_MODEL_CTRA=3

uint32 id
geometry_msgs/Vector3 size
uint8 object_class

# Motion model used to interpret every points[].model_state and
# points[].model_covariance in this prediction.
uint8 motion_model_type

bool matched_in_current_frame

DynamicObjectPredictionBranch[] branches
