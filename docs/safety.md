# PPBNG development safety policy

Full filesystem or administrator access is a technical capability, not operator consent to
control hardware or alter the computer. Every developer and automated agent must follow
this policy.

## Allowed without additional approval

- Read project source, vendor documentation, and non-secret device logs.
- Search official vendor documentation online.
- Edit files only inside `ppbng_ros2_ws`.
- Configure, build, and run unit tests that do not open device interfaces.
- Run simulators that bind only to loopback and write only disposable test data inside the
  workspace or the operating-system temporary directory.
- Perform read-only checks of installed SDK/compiler/ROS versions and filesystem capacity.

## Explicit operator approval required

Approval must identify the device or system action and applies only to the described test.

- Enumerate, open, configure, arm, trigger, acquire from, reset, or update any camera.
- Open a physical serial/USB device, send an RSM400 command, or emit a timing-controller
  output.
- Start illumination, a scan stage, robot motion, or any other actuator.
- Change NIC address, subnet, MTU, jumbo-frame, routing, firewall, driver, power-management,
  registry, service, firmware, or system settings.
- Install, upgrade, uninstall, repair, or overwrite vendor software or SDKs.
- Write to legacy projects, vendor installations, calibration packs, or existing datasets.
- Delete or relocate material files, or perform destructive source-control operations.

## Hardware-test procedure

Before requesting approval, state:

1. the exact device and interface;
2. whether the action is read-only or sends commands;
3. expected physical behavior;
4. files and settings that may change;
5. stop/rollback procedure;
6. maximum test duration.

After approval, begin with one device, conservative settings, bounded timeouts, and an
operator-visible stop path. Never combine the first connection test with a long acquisition
or multi-device trigger test.

## Data and legacy protection

The sibling legacy directories are read-only references. Production code and generated
development artifacts belong only under `ppbng_ros2_ws`. Tests must create unique output
directories and must refuse to overwrite an existing dataset.

Credentials, NTRIP secrets, calibration data, camera firmware, and vendor installation
contents must not be copied into source control. Logs must avoid serial-port payloads or
environment dumps that could expose secrets unless the specific fields have been reviewed.

## Stop conditions

Stop the current operation and report immediately if a command targets an unexpected path
or device, a device shows ambiguous identity/control ownership, a test would require an
undisclosed system change, or observed physical behavior differs from the stated expectation.

