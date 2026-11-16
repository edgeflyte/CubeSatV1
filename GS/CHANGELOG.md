# Changelog — Ground station 8601

Notable changes for people running this board.
Format based on [Keep a Changelog](https://keepachangelog.com).
This module uses [semantic versioning](https://semver.org).

## [2.1.0] - 2026-10-12 (Orbit)

### Added
- `tx` followed by 64 hex characters sends one 32-byte radio frame.
  A broadcast (destination all zeros) does not wait for a reply.
  Any other destination does. The console prints `TX OK` or `TX ERR`.
- `channel` and a number from 0 to 125 sets this station's channel.
- A stuck or corrupt receive buffer is cleared. Three faults within 5 seconds restart the radio.

### Changed
- A frame is accepted when it starts with `#1` or `#R` and the sender id is not zero.
- `device` followed by a hex id keeps that sender. `device 0` keeps every sender.
- Version reads `2.1.0`. The identify field is `20100`.
- `serial` prints `8601-0042`. Identify still prints `0016`.

### Removed
- Frames that start with `*` or `%`, and the old checksum on those frames.
- The commands `json`, `csv`, and `compact`. The computer still receives `[HEX]` and 64 hex characters.

## [2.0.2] - 2026-08-21 (Horizon)

First packaged suite release. The computer still receives `[HEX]` lines.

### Changed
- Version reads `2.0.2`. The identify field is `20002`.
  Software that only accepted identify version `1` needs to accept `20002`, and `20100` for this suite.

Development notes for this line are in
[docs/maintainer/internal-history.md](../docs/maintainer/internal-history.md).
