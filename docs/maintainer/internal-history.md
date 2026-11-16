# Internal history

Development record. The kit manual does not include these notes.
Public changelogs keep only what a kit owner needs after an upgrade.

## Unreleased 2.0.1

**2.0.1** was an internal convergence target and was never tagged or shipped.
Official packaging began at **2.0.2** ("Horizon"). Identify integer `20001`
was reserved for 2.0.1 and does not appear on a released board.

BMS, ASB, and GS 2.0.1 entries were skeletons only (source not shipped).
The MCB 2.0.1 work below landed inside the 2.0.2 tree.

### MCB 2.0.1 work (shipped later as 2.0.2)

Wire layouts were unchanged. Two fixes change reported values:

- Inbound radio packets were decoded with a raw struct read
  (`radio.read(&pkt, sizeof(pkt))`) that did not match the big-endian wire
  format. Transmitter id, destination id, and packet type were wrong, which
  broke mesh tracking and uplink routing. Decoding moved to
  `TelemetryRadio::deserializePacket()`.
- ASB altitude decode did not remove the +100 m bias the ASB adds on the
  wire, so altitude read about 100 m high and was rebroadcast. The parser
  now subtracts 100 m.
- ASB temperature T1 was parsed into a `uint8_t`, truncating the
  centi-degree value. It is parsed as `int`.
- A stray `pinMode(26, OUTPUT)` drove ADC0 while it was also read with
  `analogRead(26)`. Removed.
- `TELEM_HEADER_SIZE` corrected from 8 to 12. `buildPacket()` clamped to
  the 20-byte on-wire payload.
- `strncpy` sites null-terminate. The per-packet `[RXP]` dump was removed.
- Dead `DEFAULT_SERIAL_NUMBER 0x23` removed. At that time the constructor
  default was `0x00000021` (`SystemConfig`).
- Command handlers gained range checks. Version strings come from
  `version.h`.
- CSV log fields are sanitized.
- `_accessLevel` is stored and left at 0. It is not enforced. The kit
  manual states this as: there is no password on USB or radio commands.

`version.h`, `TELEM_MAX_WIRE_DATA_SIZE` (20), and
`LOG_MAX_CONSECUTIVE_ERRORS` (10, `LOG_VERBOSE` default off) were added
in that pass.

## Serial numbers during development

Older notes mentioned MCB serials `0x21` and `0x30`, and a removed define
`0x23`. The 2.1.0 constructor default is `0x00000043`. Identify prints the
low 16 bits, so a board with no EEPROM override shows `0043`. EEPROM magic
`0xCB26` loads a stored serial on the next boot.

GS identify serial `0x0016` and the `serial` command string `8601-0042`
are different fields on purpose. Hosts that talk on the radio use `0016`.

## Ground station baseline and GTerm

2.0.2 GS packaging started from baseline `8601-FW-v1.0.1-re1125`.
Identify `FW_VERSION` on that baseline was `1`. 2.0.2 changed it to
`20002`. 2.1.0 is `20100`.

GTerm is the internal host used to exercise GS uplink. It is not a product
name. The kit command is `tx` on the ground-station USB port.

The 8701 ground station is a separate product and is not in this suite.

## ASB sketch names

2.0.2 replaced separate `ASB.ino` and `ASB_SHT4x.ino` sketches with one
`ASB/ASB.ino` that prefers SHT4x and falls back to SHT31.
`ASB_SHT4x.ino` is not in this tree.

`examples/ASB_Framework` was imported from that older layout. It still
emits `$ASB,3` and `$ASB,4`, encodes altitude without +100, and scales
particulate mass by 100 and number concentration by 1/10. Stock
`ASB/ASB.ino` emits `$ASB,0` and `$ASB,1`, adds 100 m on the wire, and
scales particulate mass and number concentration by 10. The flight
computer accepts ids 3 and 4 as the same packets as 0 and 1 and decodes
them with the stock scales. The kit packet specification documents the
stock sentences. The framework quick start tells a custom build to match
that specification before it is flown with the stock flight computer.

`ASB/ASB_PACKET_SPECIFICATION.md` revision 1.0 (2025-09-27) described
ASB3/ASB4, a 1 Hz rate, altitude with no offset, and PM mass ×100.
That revision was wrong for the 2.1.0 sketch and was rewritten to match
`txMainPkt()` / `txPM2Pkt()`.

## Arduino sketch folders

Putting the version in the folder or `.ino` name made the IDE offer to
nest `MCB/MCB/`, which dropped the companion `.cpp` files. That dialog
was a recurring upload failure. Public docs tell the user to cancel it.
Folders stay `MCB/`, `BMS/`, `ASB/`, `GS/`.

## Release manifest note replaced in 2.1.0 docs

`release.json` previously said: "2.1.0 (Orbit) enables bidirectional
ground-station link for GTerm: GS validates #1/#R frames, accepts host
'tx' commands, MCB adds mesh messaging and radio control commands."
The published note now describes the same behavior without the tool name.
`"released"` was still null and the ASB artifact hash was null at the
time the kit manual was separated from this file.
