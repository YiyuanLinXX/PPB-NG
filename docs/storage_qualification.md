# Windows durable-write qualification

`ppbng_storage_durable_write_qualification` is an operator-invoked qualification tool. Production
launch does not run it, and constructing any ROS node does not run it. `--help`, missing arguments,
unknown arguments, or any invocation without the exact
`--confirm-durable-write-qualification` switch performs no write and exits refused.

The tool is intentionally bounded:

- `--duration-seconds` is limited to 1–60 seconds;
- `--block-mib` is limited to 1–64 MiB;
- `--maximum-test-mib` is limited to 1–16384 MiB and must hold at least one block;
- the output root must already exist and have the maximum test allocation plus a fixed 100 GiB
  reserve available before the probe begins.

After explicit authorization it canonicalizes the output root, identifies the containing Windows
volume, and creates one UUID-named probe directly below that root with `CREATE_NEW`, exclusive
sharing, `FILE_FLAG_WRITE_THROUGH`, and sequential access. It never opens, truncates, appends to,
or replaces an existing probe. It writes complete bounded blocks, calls `FlushFileBuffers` (the
Win32 durable equivalent of a `Flush(true)` request), then marks the process-owned open handle for
delete-on-close. This avoids a close-then-delete path race and deletes only the object whose
exclusive creation succeeded in the current process. Failure cleanup is likewise ownership-gated;
it never scans or deletes by wildcard.

On success, the tool writes JSON to a second UUID-named `CREATE_NEW` temporary file, calls
`FlushFileBuffers`, closes it, and atomically renames it in the same directory using
`MOVEFILE_WRITE_THROUGH` without replacement. The evidence records the canonical output root,
volume root/serial/filesystem, UTC interval, bytes, durable duration/rate, block size, maximum test
bytes, reserve, and flush/write-through method. An existing final or temporary name causes a
fail-closed error.

The tool never edits machine YAML. After reviewing the JSON, the operator places only its absolute
path in `session.throughput_evidence_path`. Production launch reads the evidence without modifying
it and fails closed unless the canonical output root, current Windows volume serial/filesystem,
write-through and `FlushFileBuffers` flags, byte count, duration, and internally recomputed rate all
agree. The old manually asserted `throughput_qualified` and rate fields are not accepted as proof.

Example syntax (documentation only; do not run without approving the target volume and write):

```powershell
ppbng_storage_durable_write_qualification.exe `
  --confirm-durable-write-qualification `
  --output-root D:\PPBNG_DATA `
  --duration-seconds 20 `
  --block-mib 16 `
  --maximum-test-mib 8192
```

For the checked-in unified configuration, first run the read-only planner. It validates the
configured directory, calculates the exact rate and capacity gates used by production, and prints
the bounded qualification command; it does not create, write, rename, or delete any file:

```powershell
.\tools\plan_storage_qualification.cmd
```

As of the current `ppbng_config.yaml` (FX10e 50 Hz, SWIR 20.59 Hz, RGB and thermal 2 Hz), the
all-stream estimate is 76,318,659 B/s and the 25%-headroom durable-write threshold is
95,398,324 B/s. The two-hour start-capacity requirement, including the 100 GiB reserve, is
794,242,113,400 bytes. These numbers are derived from configuration and must be recalculated after
any geometry or rate change. The planner currently resolves the configured target to
`C:\Users\cairlab\Desktop\Projects\PPB_NG_2026\ppbng_ros2_ws\data` on `C:`.

Only after reviewing that read-only output and explicitly approving the bounded write should the
operator run the printed command. After success, set `session.throughput_evidence_path` to the
new JSON's absolute path, restart production launch, and retain the JSON with the configuration
snapshot. Do not copy the rate into YAML; production recomputes it and verifies the evidence.

This is only a storage qualification. It does not authorize cameras, serial ports, trigger outputs,
RSM400 motion, network changes, or a production acquisition. A two-hour full-rate acceptance run
remains required by `hardware_validation_plan.md`.
