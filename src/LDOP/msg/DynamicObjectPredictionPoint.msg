# Relative time from the source frame stamp to this predicted sample.
duration time_from_start

# Future model-native state. Semantics match DynamicObject.model_state.
float64[] model_state

# Future model-native covariance, row-major, len(model_state)^2.
# Semantics match DynamicObject.model_covariance.
float64[] model_covariance

# Planning/RViz influence in [0,1]. This does not change model_covariance.
float64 influence_weight
