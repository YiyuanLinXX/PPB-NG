# Time authority contract

`ppbng_core/time_authority_node` is a conservative correlation authority. Production defaults
consume `/timing/pps_anchor`, `/timing/trigger_event`, and `/gnss/observation`, and publish
`/acquisition/trigger`. These four topic names are explicit parameters so an installation may
remap them without changing the correlation contract.

Only a validated `UNIHEADINGA` observation with `time_status == FINE`, a valid absolute
receiver UTC value, and exactly zero fractional nanoseconds is eligible to label a PPS.
The node never rounds a fractional receiver epoch into a PPS second. The GNSS host-time
estimate is:

`host_receive_monotonic_ns - receiver_output_delay_ms * 1e6`

It is compared with:

`pps.host_receive_monotonic_ns + expected_system_offset_ns`

The nearest unique candidate must fall within `maximum_host_skew_ns`. Both raw host
timestamps, PPS sequence, receiver delay, sequence monotonicity, and UTC monotonicity are
therefore evidence; they are not silently replaced by ROS wall-clock arrival time.

`pps_gnss_evidence_confirmed` defaults to `false`. In that state, a correlated UTC estimate
is published only as `HOLDOVER` with an explicit `UNVERIFIED` detail and uncertainty of at
least `maximum_host_skew_ns`. `LOCKED` is possible only after the installation-specific PPS
wiring/latency evidence has been measured and this parameter has deliberately been set true,
and only while the controller's anchor itself reports locked.

Trigger events wait for an anchor for at most `trigger_wait_timeout_ns`. Timeout, queue
overflow, controller reboot, invalid timing fields, or unavailable evidence publishes the
original trigger sequence as `UNSYNCED` rather than dropping it; this retains the key needed
for later offline repair. All queues are bounded. Defaults are deliberately conservative and
must be validated on the final wiring with an oscilloscope before enabling confirmed evidence.

PPS absence is a supported task mode, not a fatal error. Immediate-start controller events retain
their channel sequence, hardware tick and tick frequency with PPS sequence zero and `UNSYNCED`
quality. The raw GNSS stream continues independently. `FrameContext` may attach a best-effort GNSS
position using host-monotonic receive-time bracketing, but labels that association `DEGRADED` and
does not synthesize canonical frame UTC.
