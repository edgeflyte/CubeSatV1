# Changelog — Battery board (BMS)

Notable changes for people running this board.
Format based on [Keep a Changelog](https://keepachangelog.com).
This module uses [semantic versioning](https://semver.org).

## [2.1.0] - 2026-10-12 (Orbit)

### Changed
- Version reads `2.1.0`. The identify field is `20100`.
- The `$BMS` sentence is unchanged.

## [2.0.2] - 2026-08-21 (Horizon)

First packaged suite release. The `$BMS` sentence matches 1.x boards.

### Added
- Identify response, model `2001`.

### Changed
- Version reads `2.0.2`. The identify field is `20002`.
- Solar inputs are measured as inputs.
- Pack and solar voltages use a 3.3 V reference.
- Pack percent runs from 0 to 100.

After an upgrade from 1.x, compare the reported pack voltage with a meter.
The sentence layout did not change, and the numbers can shift slightly.

Development notes for this line are in
[docs/maintainer/internal-history.md](../docs/maintainer/internal-history.md).
