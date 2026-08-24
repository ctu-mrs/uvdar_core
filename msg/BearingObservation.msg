# Signal identifier decoded by the tracker. Negative values denote an
# unidentified track when publishing unidentified tracks is enabled.
int32 id

# Stable tracker-local identifier. This distinguishes separate image tracks
# even when their decoded signal identifiers are equal.
uint32 track_id

# Unit bearing expressed in BearingObservationArrayStamped.header.frame_id.
geometry_msgs/Vector3 bearing

# Row-major 3x3 covariance of the unit-bearing components. This is a
# tangent-plane covariance and is therefore normally rank two.
float64[9] covariance

# True when the bearing was generated from a tracker prediction because no
# detector measurement was associated in this frame.
bool predicted

