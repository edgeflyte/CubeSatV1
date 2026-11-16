# Release checklist — Suite 2.1.0

Use this before setting `release.json` `"released"` and tagging `v2.1.0`.
The host tool used during bring-up is GTerm.

## Identity

- [ ] MCB: `version`, `#G` version, `#?EF?#`, and SD boot log report `2.1.0` / `20100`
- [ ] BMS: `#?EF?#` reports model `2001` and `FW_VERSION=20100`
- [ ] ASB: `#?EF?#` reports model `3001` and `FW_VERSION=20100`
- [ ] GS: `version` and `#?EF?#` report `2.1.0` / `20100`

## GS link (GTerm)

- [ ] GS prints `[HEX]` for MCB `#1` frames and counts them as valid
- [ ] Host `tx <64 hex>` of NOP (`0x8000`) returns `TX OK` and MCB acknowledges
- [ ] Short message `0x0900` from GTerm appears on MCB USB as `[MSG]`
- [ ] Second GS / radio hears the same message (optional)

## MCB

- [ ] ASB `$ASB,0` / `$ASB,1` decode into environmental / PM telemetry
- [ ] `reboot` without `confirm` is rejected; `reboot confirm` reboots
- [ ] Radio `0x8200` / `0x8201` / `0x8204` change channel / PA / telem interval
- [ ] Mesh relay does not rebroadcast command packets

## Release

- [ ] `release.json` SHA-256 fields populated after local builds
  (`tools/record-release-hashes.ps1`)
- [ ] Tag `v2.1.0` (maintainer)

There is no CI build and no hardware-in-the-loop test. This list is the gate.
