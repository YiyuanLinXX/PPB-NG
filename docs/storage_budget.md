# Default two-hour storage budget

Production launch derives the payload rate from the frozen machine configuration; it does not
accept a manually typed aggregate rate. The following is the conservative 120 Hz example profile
in `machine.example.yaml`, not the currently checked-in unified machine configuration:

| Stream | Geometry | Rate | Payload rate |
|---|---:|---:|---:|
| FX10e | 1024 spatial × 448 bands × 16 bit | 120 Hz | 110,100,480 B/s |
| SWIR | 384 spatial × 288 bands × 16 bit | 120 Hz | 26,542,080 B/s |
| RGB | 12,288,000-byte raw Bayer frame | 2 Hz | 24,576,000 B/s |
| A6701 | full 640 × 513 × 16-bit transport payload | 2 Hz | 1,313,280 B/s |

The total is **162,531,840 B/s** (about 155.00 MiB/s). For 7,200 seconds this is
1,170,229,248,000 bytes (about 1.064 TiB) of raw payload. The configured 25% capacity headroom
raises the stream allowance to 1,462,786,560,000 bytes. Adding the fixed 100 GiB finalization and
OS reserve gives a minimum start-time free-space requirement of **1,570,160,742,400 bytes**
(about 1.428 TiB).

An empty nominal 2 TB drive would have about 400 GiB remaining after this budget, but nominal
capacity is not admission evidence. Before every task, the production manager queries the actual
free space on the configured output volume and refuses to start below the calculated requirement.
It repeats the check during acquisition against the remaining task duration and always retains the
fixed reserve.

The 25% allowance covers framing, sidecars, dark references, recovery segments, filesystem
overhead, and modest rate uncertainty. It is not proof of sustained write performance. The same
volume must separately pass the explicitly approved durable-write qualification at no less than
203,164,800 B/s (payload rate plus 25%). The two-hour full-rate Stage 7 run remains the final
thermal, throughput, and stability acceptance test.

The current `ppbng_config.yaml` uses measured/configured HSI line rates of 50 Hz and 20.59 Hz. Its
production estimate is 76,318,659 B/s, its 25%-headroom qualification threshold is 95,398,324 B/s,
and its two-hour capacity gate plus 100 GiB reserve is 794,242,113,400 bytes. Run
`.\tools\plan_storage_qualification.cmd` to recalculate these values read-only from the active
configuration before qualification; never reuse either set of numbers after a rate change.
