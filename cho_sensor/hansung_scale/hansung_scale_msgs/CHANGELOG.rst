^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hansung_scale_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-07)
------------------
* Initial release, split out of ``hansung_scale_driver`` (then named
  ``hansung_scale``) the way
  ``realsense2_camera_msgs`` is split out of ``realsense2_camera`` (an
  ``ament_python`` package cannot generate ROS interfaces).
* ``msg/WeightStamped``: stamped weight with unit, grams conversion,
  stability flag and the raw status field.
* ``srv/DeviceInfo``: which port is actually open, the adapter's serial
  number and USB id, line settings, and frame statistics.
* ``srv/SendCommand``: send raw bytes and optionally capture the reply, for
  reverse-engineering the undocumented indicator commands.
