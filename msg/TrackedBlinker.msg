float64 x
float64 y
int32 id
uint32 track_id
builtin_interfaces/Time stamp

# Current image-plane uncertainty used by downstream estimators.
float64 covariance_00
float64 covariance_01
float64 covariance_10
float64 covariance_11

# Raw detector measurement covariance when a detection was associated.
float64 measurement_covariance_00
float64 measurement_covariance_01
float64 measurement_covariance_10
float64 measurement_covariance_11

float64 predicted_x
float64 predicted_y
float64 prediction_covariance_00
float64 prediction_covariance_01
float64 prediction_covariance_10
float64 prediction_covariance_11
float64 confidence_x
float64 confidence_y
builtin_interfaces/Time prediction_reference_time
float64[] x_coeff
float64[] y_coeff
bool poly_reg_computed
bool extended_search
bool virtual_point
bool associated_with_detection
