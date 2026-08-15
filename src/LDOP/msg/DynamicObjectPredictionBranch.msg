uint8 BEHAVIOR_LONGITUDINAL=1
uint8 BEHAVIOR_VERTICAL=2
uint8 BEHAVIOR_TURNING=3
uint8 BEHAVIOR_LATERAL=4

# Behavior family represented by this branch.
uint8 behavior_type

# Normalized signed behavior strength in [-1,1].
float64 behavior_value

# Branch probability.
float64 probability

DynamicObjectPredictionPoint[] points
