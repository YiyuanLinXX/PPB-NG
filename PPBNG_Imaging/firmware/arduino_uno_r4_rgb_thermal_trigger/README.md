# UNO R4 synchronized RGB + thermal trigger

Upload `arduino_uno_r4_rgb_thermal_trigger.ino` to the UNO R4 WiFi.

Connections:

- D11 to Blackfly S GPIO cable black wire, pin 2 `OPTOIN` / Line 0.
- Arduino GND to Blackfly S GPIO cable blue wire, pin 5 `Opto GND`.
- D12 to A6701 `SYNC IN` BNC center.
- Arduino GND to the A6701 `SYNC IN` BNC shield.

D11 and D12 are P411 and P410 on the same RA4M1 port. The firmware changes both bits with one masked `R_IOPORT_PortWrite`, rather than two sequential `digitalWrite` calls. Both outputs power up low, require an exact `START` command, and return low if host keepalives stop for three seconds.

The ROS 2 synchronized launch owns the serial port and sends commands. Do not keep Arduino Serial Monitor open while running ROS 2.
