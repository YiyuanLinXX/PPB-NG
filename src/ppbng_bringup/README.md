# PPBNG bringup safety

`simulation.launch.py` is the default development path and does not start device processes.

`production.launch.py` is deliberately fail-closed. It requires all of the following before it
will create the production acquisition manager and independent RGB, Thermal, FX10e, SWIR,
GNSS, RSM400, and timing ROS processes (RGB and Thermal are always separate processes):

- the explicit launch argument `hardware_enabled:=true`;
- a valid YAML `machine_config` with both `safety.configured: true` and
  `safety.hardware_enabled: true`;
- a nonempty `machine_id` that exactly matches the YAML value;
- nonempty output path, COM ports, camera identities, and no `REQUIRED` placeholders.
- explicit Stage 5 confirmation of the installed RSM400 STAB motion-status digit and bounded,
  mandatory receive-only readiness telemetry.

Passing the launch gate only constructs inert processes. No device opens during node construction.
The user must then call `acquisition/start`; the manager exclusively creates the dataset session and
manifest and dispatches asynchronous `PrepareDevice`, arm, and start batches. Dark capture requires
`acquisition/confirm_dark_ready`, and sample capture requires `acquisition/confirm_sample_ready`.
When `timing.pps_required` is true, the first observed PPS-gated trigger changes the state to
recording. When false, successful immediate hardware-timer arming enters recording without waiting
for PPS. Service callbacks never wait for device futures.

Global `FaultEvent` records disarm timing outputs before stream stop. A non-global device fault or
degraded status is recorded in the manifest/status while other streams continue. A batch timeout
treats unanswered start calls as possibly-open and attempts the same disarm-first safe stop.

Production launch must never be exercised with `hardware_enabled:=true` until every identity,
frequency, geometry, serial port and RSM feature flag in the copied machine file has been verified.
The example intentionally remains disabled and contains `REQUIRED` placeholders.

Copy `config/machine.example.yaml` to a local configuration outside source control, verify every
physical identity, replace all placeholders, and only then set `configured: true`. Never store
NTRIP credentials in this PC's configuration.

## Operator terminal sequence

Use a second ROS 2 terminal to keep the transient-local workflow prompt visible:

```powershell
ros2 topic echo /acquisition/status ppbng_interfaces/msg/AcquisitionStatus
```

After the approved hardware validation stages and local machine configuration are complete, the
bringup command is:

```powershell
ros2 launch ppbng_bringup production.launch.py hardware_enabled:=true machine_config:="C:/path/to/machine.yaml" machine_id:="PPBNG-IPC"
```

In the status terminal, follow the prompts. Use a new `request_id` for every new operator command:

```powershell
ros2 service call /acquisition/start ppbng_interfaces/srv/StartAcquisition "{request_id: 'start-001', dataset_name: 'DATASET_NAME', force_degraded: false}"
ros2 service call /acquisition/confirm_dark_ready ppbng_interfaces/srv/ConfirmDarkReady "{request_id: 'dark-001'}"
ros2 service call /acquisition/confirm_sample_ready ppbng_interfaces/srv/ConfirmSampleReady "{request_id: 'sample-001'}"
ros2 service call /acquisition/stop ppbng_interfaces/srv/StopAcquisition "{request_id: 'stop-001', reason: 'operator completed task'}"
```

The launch terminal also prints every workflow transition. Degraded transitions are warnings and
fault transitions are errors. Do not issue the dark or sample confirmation until the corresponding
prompt appears.
