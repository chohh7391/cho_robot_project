"""What a remembered detection looks like as it ages. No ROS here.

``node.py`` draws each object for ``marker_lifetime_sec`` after its last
detection and then lets rviz delete it, which is right for commissioning: a
marker that outlives its evidence is a stale claim about where a vessel is.
For showing a run it is the wrong way round. The recovery sweep finds a vessel,
the arm leaves the viewpoint, the pose ages out of the aggregation window within
half a second -- and the vessel the tree just latched vanishes from rviz at
exactly the moment the audience should be looking at it.

``marker_memory.py`` keeps it instead, and this module decides how it looks
while it is kept: at full strength for ``hold_sec``, then fading linearly over
``fade_sec`` down to ``floor_fraction`` of its own alpha, where it stays. It
dims rather than disappears on purpose -- the position is still the last thing
any camera said, and a faint body says both "here" and "not seen lately".

The label says the second half out loud once the pose node itself would have
given up on the object (``stale_after_sec``, its ``marker_lifetime_sec``), so a
full-strength body with an age on it reads as "just lost", not "still seen".
"""

# The CameraVisibility.state a camera has to report for its line of sight to be
# drawn. Mirrors cho_interfaces/msg/CameraVisibility.msg; kept as a plain int so
# this module stays importable without the message package.
STATE_OK = 1


def fade_alpha(age_sec, base_alpha, hold_sec, fade_sec, floor_fraction):
    """Alpha for a marker last refreshed ``age_sec`` ago.

    ``base_alpha`` is the alpha the pose node drew it with, so each object keeps
    its own translucency and the fade is relative to it. The result never goes
    below ``base_alpha * floor_fraction``: a remembered object dims, it does not
    vanish.
    """
    base_alpha = float(base_alpha)
    floor = base_alpha * min(max(float(floor_fraction), 0.0), 1.0)
    if age_sec <= hold_sec:
        return base_alpha
    if fade_sec <= 0.0 or age_sec >= hold_sec + fade_sec:
        return floor
    progress = (age_sec - hold_sec) / fade_sec
    return base_alpha + (floor - base_alpha) * progress


def aged_label(text, age_sec, stale_after_sec):
    """Return the label as drawn: the object's name while live, with its age once stale."""
    if age_sec <= stale_after_sec:
        return text
    return '%s (%ds ago)' % (text, int(age_sec))


def is_forgotten(age_sec, forget_sec):
    """Whether a remembered object should stop being drawn. 0 means never."""
    return forget_sec > 0.0 and age_sec > forget_sec


def sight_lines(visibility, targets):
    """``[(camera, object)]``: every camera that is seeing an object we can draw.

    ``visibility`` is ``[(object, [(camera, state), ...]), ...]`` -- the shape of
    an ObjectVisibilityArray with the ROS types taken out. ``targets`` maps an
    object name to a position; an object the display has never been given a
    marker for has nothing to draw a line to, and is skipped rather than drawn
    to the origin.

    Only STATE_OK counts. A SUPPRESSED camera is also seeing the tag, but its
    samples are not in the estimate while a higher-priority camera overrides
    it, and drawing its line would show a fusion that is not happening.
    """
    pairs = []
    for name, cameras in visibility:
        if name not in targets:
            continue
        for camera, state in cameras:
            if state == STATE_OK:
                pairs.append((camera, name))
    return pairs


def pair_labels(markers):
    """``{name: body_key}`` from the pose node's (key, type_is_text, text) triples.

    ``node.py`` numbers each object's markers as a pair -- the body at ``2*i``
    and its text label at ``2*i + 1`` -- and only the label carries the name.
    This rebuilds that pairing so a line of sight can end at the BODY, which is
    where the vessel is, rather than at the label floating above it. ``markers``
    is ``[((ns, id), is_text, text), ...]``.
    """
    bodies = {key for key, is_text, _ in markers if not is_text}
    names = {}
    for (ns, marker_id), is_text, text in markers:
        if is_text and (ns, marker_id - 1) in bodies:
            names[text] = (ns, marker_id - 1)
    return names
