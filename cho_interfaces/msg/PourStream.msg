# Whether material is falling between the lip and the receiving vessel, right now.
#
# The scale cannot answer this. It reports what has LANDED, so "nothing has
# started yet" and "it started but has not landed yet" look identical to it for
# a whole transport delay, and during a drip the HS-AA held one value and
# flagged it stable for 5.4 s with material still arriving. A camera watching
# the fall column answers in one frame.
#
# Scope, deliberately narrow: this says material is crossing a band of image,
# not how much, and never whether the pour is finished. Mass stays the scale's
# job -- it resolves 0.01 g and no camera comes close. Camera says WHEN, scale
# says HOW MUCH.

std_msgs/Header header

# Material is crossing the detection band.
#
# Its edges carry the value. The rising edge is the pouring onset, seen roughly
# a transport delay before the scale can infer it. The falling edge says the lip
# has stopped delivering, which is what makes the mass still in the air a known
# quantity rather than an estimate.
#
# A gap between drops reads as false here, and that is correct: this reports the
# instant, not the verdict. Deciding that a pour is over -- rather than between
# avalanches -- belongs to whatever is counting the mass.
bool flowing

# Fraction of the band's rows the stream crosses, 0..1. A stream in free fall
# crosses all of them; a shadow, a splash or an insect does not. Useful as a
# crude thickness signal, and as the number to set thresholds from.
float64 coverage

# The detector has a background model and this frame was usable. False while it
# is still learning the background, and whenever so much of the band changed at
# once that it cannot be a stream -- the gripper swinging through, a light being
# switched, someone's hand. A consumer must treat false as "no information",
# never as "not flowing".
bool valid

# Why valid is false, empty when it is true. For commissioning; not for logic.
string status
