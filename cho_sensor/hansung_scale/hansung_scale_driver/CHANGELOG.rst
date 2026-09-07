^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hansung_scale_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-07)
------------------
Aligned the package with the realsense2_camera wrapper's interface, keeping
only the features that mean something for a serial indicator.

* New ``hansung_scale_msgs`` package: ``WeightStamped``, ``DeviceInfo``, ``SendCommand``.
* ``~/weight_stamped`` publishes one stamped, self-describing sample (weight,
  unit, grams, stability, raw status) instead of leaving consumers to
  time-correlate ``~/weight`` / ``~/stable`` / ``~/unit``.
* Device selection by ``serial_no`` / ``usb_port_id`` / ``device_type``
  instead of a ``ttyUSB`` index that renumbers on reboot. ``port`` now also
  accepts a ``/dev/serial/by-id`` symlink, and is ignored when one of the
  three selectors is set.
* ``wait_for_device_timeout`` waits for the adapter during configure;
  ``reconnect_timeout`` re-attaches after the link drops instead of leaving a
  dead node behind.
* ``/diagnostics``: link state, measured frame rate, staleness, parse errors,
  dropped bytes. Published from a plain publisher so an inactive-but-
  configured node still reports.
* ``enable_*`` per-topic flags and ``weight_qos`` / ``raw_qos`` presets.
* New services: ``~/device_info``, ``~/hw_reset``, ``~/send_command`` (with
  optional response capture, for finding the undocumented tare/zero bytes).
* Runtime-settable parameters are honoured; the rest are rejected with an
  explanation once the node is configured, rather than accepted and ignored.
* ``scale_sniffer`` gained ``list_ports:=true`` (port table plus copy-paste
  selectors) and ``decode:=true`` (shows how the parser reads each line).
* ``scale.launch.py`` rebuilt around a ``configurable_parameters`` table.
  Unlike ``rs_launch.py``, arguments left at their default are dropped so
  ``params_file`` actually decides those values.
* Frame parsing moved to ``hansung_scale_driver/protocol.py`` and port handling to
  ``hansung_scale_driver/device.py``, both free of ROS and pyserial-hardware dependencies;
  131 tests now run with no scale attached.
* ``parity`` accepts the spelled-out ``NONE``/``EVEN``/``ODD``; the
  single-letter form is still accepted but ``N`` is a YAML 1.1 boolean.
* Fixed: ``self._services`` shadowed ``rclpy.Node``'s own service list, which
  garbage-collected the parameter services off the graph — ``ros2 param``
  timed out with nothing logged.
* Fixed: pulsing DTR on a device without modem control lines raises bare
  ``OSError(ENOTTY)``, not ``serial.SerialException``; escaping a service
  callback, it took the whole node down.
* Fixed: unterminated input is now bounded instead of buffered without limit.
* Corrected ``expected_frame_rate`` from a guessed 10.0 to the measured 5.0:
  the real HS-AA emits a frame every 192-209 ms.
* The indicator's RS232 port is unidirectional (output only). Confirmed by
  the manufacturer, and independently on the bench: 43 candidates across three
  axes -- command bytes, command framing, and RTS/DTR line state -- produced no
  reaction on any of four signals (reading, frame rate, stream continuity,
  non-standard lines) and no change on the indicator's own display.
* Consequently ``enable_commands`` (default false) gates the whole write-side
  interface: ``~/cmd``, ``~/tare``, ``~/zero`` and ``~/send_command`` are not
  advertised at all, rather than advertised and always failing. A service in
  ``ros2 service list`` is a promise, and on this unit it is one the hardware
  cannot keep. The code stays for an indicator that does accept input.
* README rewritten as a read-only usage guide: quick start, per-topic and
  per-parameter reference, task recipes (including a settled-reading consumer
  verified against the real unit), a troubleshooting table, and the record of
  the unidirectional investigation so nobody repeats it.
* Poll mode reassembles a reply split across reads, clears the assembler each
  cycle so a leftover fragment cannot be glued onto the next reply, and
  recovers from a device that goes silent (three empty polls) rather than
  warning forever while only streaming mode reconnected.

0.1.0
-----
* Initial lifecycle driver for the HS-AA: 2400 8N1 continuous ASCII frames
  confirmed by sniffing the real unit, ``~/weight`` / ``~/stable`` /
  ``~/unit`` / ``~/raw``, ``~/cmd``, ``~/tare``, ``~/zero``, and the
  ``scale_sniffer`` raw dump tool.
