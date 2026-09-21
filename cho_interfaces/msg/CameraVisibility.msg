# What ONE camera contributed to ONE object's pose estimate, and why.
#
# cho_object_pose already knew this per camera -- it is what its periodic
# report logs -- but a log line is not something a behaviour tree can read.
# This is that same knowledge on a topic, because "the beaker is occluded from
# the OAK" is the trigger for a recovery sweep and nothing downstream could
# observe it before.
#
# `state` is the machine-readable half and `detail` the human one. Branch on
# the state; print the detail.

# Nothing has been heard from this camera about this object yet. Start-up, or a
# detector that was never launched for it.
uint8 STATE_UNKNOWN = 0
# The camera delivered a sample into the aggregation window.
uint8 STATE_OK = 1
# The camera is publishing detections, and this object's tag is not among
# them. THIS IS WHAT OCCLUSION LOOKS LIKE -- and also what a tag outside the
# field of view looks like, which is why the sweep is aimed by the task and not
# inferred here.
uint8 STATE_NOT_IN_FRAME = 2
# The tag decoded but failed the quality gate (hamming, decision margin, or
# apparent edge length). Too far, too oblique, or motion blur.
uint8 STATE_REJECTED = 3
# The tag decoded and passed, but TF could not put it in the robot's base
# frame at the image's own timestamp. A missing extrinsic, or a wrist camera
# with no robot running.
uint8 STATE_NO_TF = 4
# The camera's samples were DISCARDED in favour of a higher-priority camera's
# (see cameras.yaml `priority`). Not a fault: the near view superseded the far
# one for this object.
uint8 STATE_SUPPRESSED = 5
# This camera's last word about this object is older than the aggregation
# window, so whatever it said can no longer be in that window. A camera that
# stopped publishing looks exactly like a healthy one without this.
uint8 STATE_STALE = 6

string camera
uint8 state
# Free text: the rejection reason, the TF error, the camera that suppressed
# this one. Empty when the state says everything.
string detail
# Seconds since this camera last said anything about this object, measured
# when the message was built. Grows without bound for a camera that went away.
float64 age_sec
# The camera's configured priority, copied here so a consumer does not have to
# read cameras.yaml to know which view would win.
int32 priority

# HOW WELL the tag decoded, from the last detection this camera offered for
# this object -- accepted or rejected. -1 when there was no detection to score.
#
# The state alone says the detection passed the pipeline's own gate, which is
# set for "good enough to publish a pose from". A recovery sweep wants more
# than that: it exists to replace a marginal far view with a good near one, and
# "a pose exists" is satisfied by the marginal one. So the score comes with the
# state, and what counts as good enough is the consumer's threshold rather than
# a second gate here.
#
# `decision_margin` is apriltag's own confidence in the decode; higher is
# better and it falls off with range, obliquity and blur. `edge_px` is the
# shortest apparent edge of the tag in pixels, which is the range in units a
# camera can actually see -- a 39 mm tag at 0.45 m on a D435's 848-wide infra
# stream is about 39 px.
float64 decision_margin
float64 edge_px
