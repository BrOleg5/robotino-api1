# OpenRobotinoAPI 1 network protocol

How a client talks to a Festo Robotino of the generation before Robotino 3, the one served by
OpenRobotinoAPI 1.x. Checked against a robot with `iod` 695 and IO board firmware 670.

Sources:
- the OpenRobotinoAPI sources published by REC GmbH under LGPL 2.1:
  [openrobotino code, revision 23](https://svn.code.sf.net/p/openrobotino/code/trunk/?p=23),
  directories `lib/rec/xcom`, `lib/rec/sercom`, `lib/rec/robotinocom`;
- the headers of OpenRobotinoAPI 1.8.54 (`rec/iocontrol/remotestate/SetState.h`, `SensorState.h`,
  `rec/iocontrol/sercom/qdsa_protocol.h`);
- a Wireshark capture of OpenRobotinoAPI 1.8.54 talking to a robot; the messages in
  [tests/data](../tests/data) come from it.

Facts marked *unverified* are not confirmed by a capture or a robot yet.

## Transport

- TCP, robot port 80 (the robot's factory address is 172.26.1.1). RobotinoSim listens on 8080+.
- All multi-byte values are little-endian.
- Every message is `[id: u8][length: u16][payload: length bytes]`, `length <= 32000`.

| id | Message | Direction | Payload length |
|---|---|---|---|
| 0 | IOControl | client → robot | 76 |
| 1 | IOStatus | robot → client | 141 |
| 2 | CameraControl | client → robot | 5 |
| 3 | KeepAlive | both | 0 |
| 4 | RawData | | |
| 5 | ConnectionBusy | robot → client | 0 |
| 6 | Info | robot → client | 2001 |

## Session

From the capture:

1. The client connects and sends IOControl.
2. The robot sends Info (versions as text) once.
3. The client sends CameraControl (OpenRobotinoAPI sends all zeros: `02 05 00 00 00 00 00 00`).
4. The robot answers every IOControl with one IOStatus within 1.5–4 ms. OpenRobotinoAPI sends
   IOControl every 30 ms by default (`Com::setMinimumUpdateCycleTime`, at most 100 ms).

Without IOControl the robot stops the motors almost at once but keeps the connection open: a
client frozen for 3 s continues without errors. A second client
gets ConnectionBusy (or the robot serves it in passive mode, see `passive_mode`).

## IOControl (client → robot)

| Offset | Size | Field |
|---|---|---|
| 0 | u8 | image request: 1 = stream camera images over UDP |
| 1 | u8 | shutdown |
| 2 | u16 | UDP port for images (OpenRobotinoAPI: 8080) |
| 4 | i8 | NorthStar room id |
| 5 | u8 | NorthStar calibration flag |
| 6 | u8 | NorthStar calibration state |
| 7 | u16 | NorthStar calibration distance |
| 9 | 47 | P2Q frame, see below |
| 56 | 20 | see below |

The last 20 bytes are zeros in the r23 sources, but OpenRobotinoAPI 1.8.54 sends
`00×13 00 00 80 3f 00 00 00`, i.e. `1.0f` at offset 69. *Unverified* layout consistent with
that and with the fields of `SetState`: set odometry u8, odometry x, y, phi f32, NorthStar ceiling
calibration f32 (default 1.0), 3 bytes padding.

## IOStatus (robot → client)

| Offset | Size | Field |
|---|---|---|
| 0 | u16 | update rate of the link to the IO board, Hz (54–55) |
| 2 | 101 | Q2P frame, see below |
| 103 | u32 | NorthStar sequence number |
| 107 | u8 | NorthStar room id |
| 108 | u8 | NorthStar spots visible |
| 109 | i16 | NorthStar x |
| 111 | i16 | NorthStar y |
| 113 | f32 | NorthStar theta |
| 117 | u16 | NorthStar magnitude spot 0 |
| 119 | u16 | NorthStar magnitude spot 1 |
| 121 | u8 | passive mode |
| 122 | 19 | zeros in the capture |

## Info (robot → client)

2000 bytes of NUL-padded text and a passive mode byte. Example:

```
camd: 695
iod: 695
lcdd: 695
com: 695
firmware: 670
```

## CameraControl (client → robot)

resolution u8 (0 QVGA, 1 VGA), compression u8 (0 high, 1 low, 2 none), brightness u8,
contrast u8, auto white balance u8.

## P2Q and Q2P frames

The robot forwards these frames between the client and the IO board. A frame starts with `REC`,
ends with `rec` and holds a master block and four slave blocks. Slaves 0–2 drive motors 0–2;
slave 3 drives the power output and reads the encoder input.

### P2Q (47 bytes)

| Offset | Size | Field |
|---|---|---|
| 0 | 3 | `REC` |
| 3 | 1 | master: bit 1 shutdown, bit 3 always set ("new software": without it the LPC board ignores kp/ki/kd) |
| 4 + 10·s | 10 | slave s, see below |
| 44 | 3 | `rec` |

Slave block:

| Offset | Field |
|---|---|
| 0 | digital out: bit 0 brake off, bits 1–4 DO0–3 (slave 0) or DO4–7 (slave 1), bit 5 relay 0/1 |
| 1 | misc out: bit 0 position mode, bit 1 positive direction, bit 3 reset position |
| 2 | velocity magnitude, encoder ticks per control period (1/900 s); power output duty 0–255 for slave 3 |
| 3–6 | position setpoint, i32 (unused) |
| 7, 8, 9 | kp, ki, kd; 255 = board default (OpenRobotinoAPI sends 0 for slave 3) |

One velocity unit is `900 · 60 / 2000 = 27` rpm at the motor shaft.

### Q2P (101 bytes)

| Offset | Size | Field |
|---|---|---|
| 0 | 3 | `REC` |
| 3 | 11 | master: 8 ADC high bytes, 2 bytes of ADC low bits, time |
| 14 + 21·s | 21 | slave s, see below |
| 98 | 3 | `rec` |

ADC channels are 10-bit: the high 8 bits in bytes 0–7 of the block, the low 2 bits of channels
0–3 in byte 8 (bits 0–1 channel 0, 2–3 channel 1, ...) and of channels 4–7 in byte 9.

Slave block:

| Offset | Field |
|---|---|
| 0–9 | ADC |
| 10 | misc in: bit 0 velocity positive, bits 4–7 firmware version bits 3..0 (slave 0 only) |
| 11 | velocity magnitude, ticks per control period |
| 12–15 | position, i32, with the opposite sign to the velocity |
| 16 | digital in: bits 0–3 DI0–3 (slave 0) or DI4–7 (slave 1), bit 4 bumper (slave 0) |
| 17–20 | time, u32 |

ADC assignment:

| Signal | Channel | Scale |
|---|---|---|
| battery current | master 0 | / 61.44 A |
| battery 1 voltage | master 1 | / 71.02 V (not used by OpenRobotinoAPI) |
| battery voltage | master 2 | / 36.10 V |
| motor (power output) current | slave 0–3 channel 0 | / 156 A |
| distance sensors 0–8 | s2 ch2, s2 ch1, s1 ch7, s1 ch2, s1 ch1, s0 ch1, s0 ch2, s0 ch7, s2 ch7 | · 2.55/1024 V |
| analog inputs 0–7 | s0 ch3–6, s1 ch3–6 | · 10/1024 V |

Firmware version bits map to 0 → 411, 1 → 670, 2 → 1000 (EA09 board).

## Camera images

With the image request set, the robot sends images over UDP to the client's address and the port
from IOControl. Each image is split into datagrams; the first starts with a 26-byte header, the
last ends with `StoppStopp`:

| Offset | Size | Field |
|---|---|---|
| 0 | 10 | `StartStart` |
| 10 | u32 | image data size |
| 14 | u8 | type: 0 JPEG, 1 JPEG 2000, 2 raw, 3 BMP, 4 PNG, 5 TIFF |
| 15 | u8 | resolution: 0 QVGA, 1 VGA, 2 custom |
| 16 | u32 | width |
| 20 | u32 | height |
| 24 | u8 | channels |
| 25 | u8 | bits per channel |

*Unverified*: the capture has no camera traffic; this follows the r23 sources.
