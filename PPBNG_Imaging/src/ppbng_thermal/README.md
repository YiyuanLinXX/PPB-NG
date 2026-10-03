# A6701 thermal

Spinnaker-backed raw acquisition, calibration metadata and NUC state reporting. The production build wrapper enables the hardware backend. Current Mono16 transport is 640 x 513: row 0 is auxiliary and the other 512 rows are image counts. Counts and false color are not Celsius. The Pi must consume motion permission with a timeout watchdog; do not bypass it. See the [Imaging README](../../README.md) for operation.
