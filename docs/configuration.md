# Configuration policy

`ppbng_bringup/config/defaults.yaml` contains versioned, non-secret task defaults. A local
machine overlay supplies COM ports, camera identities, NIC-bound addresses, calibration
paths, output root, and verified exposure/gain values.

The shipped configuration deliberately has `hardware_enabled: false`. A future production
launch must require an explicit command-line opt-in before constructing hardware backends;
simulation and validation remain available without that opt-in. Enabling hardware does not
grant permission to change NIC, driver, firmware, persistent device configuration, or
calibration state.

Empty strings, `REQUIRED` placeholders, `null` exposure/gain, and an unverified thermal
frame-sync mode are preflight errors for the affected hardware backend. They are not filled
from legacy projects or guessed from a previous camera session.

Production launch accepts only machine-config schema 1 and requires device reset, network
reconfiguration, and firmware update permissions to be explicitly false. The UM982 path must be
explicitly receive-only, and RSM400 launch readiness must remain observe-only. These restrictions
are launch invariants, not optional defaults.

Configuration is loaded and validated before creating a dataset. Requested values, actual
device readback, and a deterministic configuration hash are written into the session
manifest. Task-critical parameters are rejected after task configuration is frozen, including
while a PPS-required task is waiting for PPS, while recording, or while stopping. A reconnect
repeats all readback checks and opens a new segment.

`timing.pps_required` is a mandatory Boolean selected before each task. `true` waits for the next
accepted PPS edge before trigger generation. `false` starts the already-frozen schedule from the
controller hardware timer without waiting for PPS. The timing controller is still required because
it generates the camera triggers; only its PPS input is optional. In no-PPS mode, trigger sequence,
controller tick, camera timestamps, raw GNSS and RSM data remain recorded, but canonical UTC is
`UNSYNCED`. GNSS attached to frames uses a clearly marked `DEGRADED` host-receive-time bracket.

The launch process passes the exact UTF-8 configuration text used to construct node parameters
to the production manager. The manager refuses startup if the on-disk file changes during that
handoff and later persists the bound text as `machine_config.snapshot.yaml`; it never re-reads a
possibly changed file when the operator starts a dataset.

Credentials are not part of this schema. In particular, the Windows acquisition PC only
receives UM982 output and never stores NTRIP settings.
