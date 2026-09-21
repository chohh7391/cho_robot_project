# Whether one object's pose is being published, and what every camera has to
# say about it. One entry per object in the table cho_object_pose was given.

string name
# A pose went out within the last aggregation window. This is the question a
# task actually asks; everything below explains the answer.
bool publishing
# The camera whose priority is currently suppressing the lower-priority ones
# for this object, or empty when no override is in force. While this is set,
# `min_cameras` is NOT applied to this object -- see the field's own note in
# cho_object_pose/config/cameras.yaml. An override is a claim that one view
# supersedes the others, and demanding that the others agree with it would be
# a contradiction, so the cross-camera check is traded away knowingly and said
# out loud here.
string override_camera
# The aggregate reason, in the same words the node logs: '4/5 samples in the
# last 0.50s', 'unstable: position spread 14.2 mm > 10.0', 'publishing, ...'.
string status
CameraVisibility[] cameras
