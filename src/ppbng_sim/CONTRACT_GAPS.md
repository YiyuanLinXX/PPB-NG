# Simulation contract gaps

This package intentionally adapts current mock APIs without changing their owning packages.
The following behaviors must not be mistaken for production-driver guarantees:

- `ppbng_rgb::IRgbBackend::next_frame()` and
  `ppbng_thermal::IThermalBackend::next_frame()` do not accept a trigger event. The simulator
  records the timing-controller event immediately before requesting a mock frame and owns that
  event-to-frame association. `ppbng_core::FrameTriggerAssociator` now provides the production
  sequence-anchor contract using trigger ID, camera frame ID, and camera/host timestamps, but the
  production ROS camera processes now keep an append-only association audit and consume the
  resolved `/acquisition/trigger` stream. Exact `MATCHED` status remains disabled until each
  camera's missed-trigger/counter semantics are verified on hardware.
- `MockThermalBackend` has no explicit disconnect injection. The simulator represents a dropout
  by retaining trigger events while suppressing thermal frame requests, then calls the existing
  `recover()`/`configure()`/`arm()` sequence. A production mock should expose a deterministic
  disconnect/timeout fault point.
- `ControllerModel` validates and emits configured channels but does not autonomously synthesize
  events at configured rates. The simulator advances its hardware tick and requests FX10e at
  120 Hz, SWIR at 80 Hz, and RGB/Thermal at 2 Hz.
- The orchestrator currently exposes aggregate health rather than a GNSS device model. The
  simulator calls `mark_degraded()` and stores the RTK-fixed flag beside every trigger observation.
- `SegmentIndex` rejects empty segments. Integration tests ensure a segment has at least one
  accepted sample before recovery; production recorders preserve explicit association and fault
  sidecars for interruptions, including starts that produce no accepted frame.

The simulation performs no device, network, ROS graph, or dataset filesystem I/O.
