# Checking on a real robot

The emulator tests the library against its own understanding of the protocol. These checks need
a robot. Keep Wireshark recording on the PC (filter `host 172.26.1.1`) and save the capture: it is
the reference for fixing any mismatch.

Put the robot on a stand with the wheels in the air for steps 2–4.

1. **Connection and readings**: `print_sensors 172.26.1.1`.
   - Info text appears (`iod: 695`, `firmware: 670`).
   - Battery voltage matches the robot display; touching the bumper changes `bumper`; a hand in
     front of a distance sensor raises its voltage; sensor `i` is the one at `i * 40` deg.
   - Readings keep coming for minutes; no reconnects.
2. **Motors**: `check_motors 172.26.1.1` (or `examples/python/check_motors.py`) runs each motor
   at ±30 rad/s and prints PASS or FAIL for the checks below; watch which wheel turns.
   - The right wheel turns, the measured velocity has the same sign and is close to 30 rad/s.
   - The motor position grows in the same direction as the velocity (the protocol sends positions
     with the opposite sign; the library assumes OpenRobotinoAPI got this right).
   - `reset_motor_positions()` brings the positions to zero.
   - `set_brake(i, true)` stops the wheel.
3. **Watchdog and disconnect**: set a velocity, then
   - kill the program: the robot stops at once (the OS closes the TCP connection and the robot
     sees it; neither timeout is involved);
   - freeze the program (Ctrl+Z or `kill -STOP` on Linux, "Suspend process" in Resource Monitor
     on Windows): the connection stays open but IOControl stops; the robot stops almost at once
     (robot side motor watchdog, well under 0.5 s). Resume it after 3 s (`fg`): the program
     continues without errors, since the robot keeps the connection open;
   - with `command_timeout = 0.5 s`, stop calling `set_velocity` while the program keeps running:
     the motors stop after 0.5 s;
   - unplug the network for 5 s: the robot stops almost at once; the client reports errors and
     reconnects.
4. **Second client**: connect a second program while the first one runs: it gets
   `ConnectionBusy` or `passive_mode == true`. Note which one.
5. **Driving** (robot on the floor): `drive_square 172.26.1.1`. The robot drives a square of about
   0.5 m without turning; compare the odometry with a tape measure. If the sides are too short or
   long, check `DriveLayout::base_radius` (125 mm in OpenRobotinoAPI, 132 mm in the robot's
   `robotino.xml`).
6. **IO**: digital outputs and relays (a multimeter or LEDs on the IO connector), analog inputs
   (a voltage source, 0–10 V), power output (`set_power_output(0.5)`) and the encoder input.
7. **Camera**: `camera_dump 172.26.1.1`. If the PC firewall allows UDP port 8080, ten images are
   saved. Save the capture with camera traffic: it is the first one, so check the image header
   layout in [protocol.md](protocol.md) against it.
8. **IOControl trailer**: in a capture of OpenRobotinoAPI with `Odometry::set()` or
   `NorthStar::setCeilingCal()` calls, check the layout of the last 20 bytes of IOControl.
