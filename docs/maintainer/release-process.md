# Release process

Manual. There is no CI yet. Kit-facing version rules live in
[`../../VERSIONING.md`](../../VERSIONING.md).

## Artifacts for each coordinated release

1. `release.json` — suite version, per-module versions, codename, build date,
   and SHA-256 of each built artifact.
2. Per-module `CHANGELOG.md` — user-visible changes only.
   Development notes that should not ship in the kit manual go in
   [internal-history.md](internal-history.md).
3. Module `README.md` version banners.
4. Git tag `v<suite-version>` (created by the maintainer; not automated).

## Steps

1. Land all code and doc changes for the release on a branch.
2. Bump `version.h` in each affected module. Do not hardcode the version
   string anywhere else.
3. Update each `CHANGELOG.md`.
4. Build each sketch and record the SHA-256 of the artifact in `release.json`.

   ```powershell
   Get-FileHash -Algorithm SHA256 <artifact>
   ```

   Board ids used by `tools/record-release-hashes.ps1`:

   - MCB: `rp2040:rp2040:rpipico`
   - BMS and ASB: `MiniCore:avr:328:clock=8MHz_internal,variant=modelPB`
   - GS: `adafruit:samd:adafruit_feather_m0`

5. Update `release.json` `suite_version`, `released` date, and module blocks.
6. Update the top-level `README.md` version banner.
7. Run [release-checklist.md](release-checklist.md).
8. Commit, tag `v2.1.0`, and push.

## Still deferred

CI, reproducible builds, and recorded fuse bytes.
