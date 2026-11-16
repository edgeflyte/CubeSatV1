# Changelog — Flight computer (MCB)

Notable changes for people running this board.
Format based on [Keep a Changelog](https://keepachangelog.com).
This module uses [semantic versioning](https://semver.org).

## [2.1.0] - 2026-10-12 (Orbit)

### Added
- Short text messages between satellites and ground stations.
- Radio commands to set the channel, transmit power, and fast telemetry interval, and to save those three.
- Confirmation for reboot, restart, and emergency power-off. On USB the word is `confirm`. On the radio the byte is `A5`.

### Changed
- Version reads `2.1.0`. The identify field is `20100`.
- Atmospheric sentences `0` and `1` are accepted. Older sentences `3` and `4` still use the same fields.
- Relayed radio traffic skips command messages.

## [2.0.2] - 2026-08-21 (Horizon)

First packaged suite release. Radio byte layouts match 1.x boards.

### Changed
- Version reads `2.0.2`. The identify field is `20002`.
- Altitude from the atmospheric board is about 100 m lower than on 1.x. That lower number is the corrected altitude.
- Messages received from other radios show the sender, destination, and message type that were transmitted.
- Temperature 1 from the atmospheric board is the full reading.
- There is no password on USB or radio commands. Treat both links as trusted.

Development notes for this line are in
[docs/maintainer/internal-history.md](../docs/maintainer/internal-history.md).
