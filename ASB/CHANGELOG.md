# Changelog — Atmospheric board (ASB)

Notable changes for people running this board.
Format based on [Keep a Changelog](https://keepachangelog.com).
This module uses [semantic versioning](https://semver.org).

## [2.1.0] - 2026-10-12 (Orbit)

### Changed
- Version reads `2.1.0`. The identify field is `20100`.
- Sentences are still `$ASB,0` (environment) and `$ASB,1` (particulate matter).

## [2.0.2] - 2026-08-21 (Horizon)

First packaged suite release. One sketch covers both humidity sensors.

### Added
- Identify response, model `3001`.
- At startup the board uses an SHT4x when it answers, and otherwise an SHT31.

### Changed
- Version reads `2.0.2`. The identify field is `20002`.
- Altitude on the wire is meters plus 100. The flight computer subtracts 100.
- A missing sensor is skipped. The rest of the board keeps running.

Development notes for this line are in
[docs/maintainer/internal-history.md](../docs/maintainer/internal-history.md).
