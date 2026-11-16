# Build the four suite sketches and, when arduino-cli succeeds, write
# each artifact path and SHA-256 into release.json.
# Requires: arduino-cli on PATH, and the board cores and libraries already installed.
# Usage (from the suite root, or from anywhere):
#   powershell -File tools/record-release-hashes.ps1

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path (Join-Path $root "release.json"))) {
  $root = $PSScriptRoot
  if (-not (Test-Path (Join-Path $root "release.json"))) {
    throw "Run from Firmware v2.1 (release.json not found)"
  }
}

$outDir = Join-Path $root "build"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

$modules = @(
  @{ key = "mcb"; path = "MCB"; fqbn = "rp2040:rp2040:rpipico"; ext = "uf2" },
  @{ key = "bms"; path = "BMS"; fqbn = "MiniCore:avr:328:clock=8MHz_internal,variant=modelPB"; ext = "hex" },
  @{ key = "asb"; path = "ASB"; fqbn = "MiniCore:avr:328:clock=8MHz_internal,variant=modelPB"; ext = "hex" },
  @{ key = "gs";  path = "GS";  fqbn = "adafruit:samd:adafruit_feather_m0"; ext = "hex" }
)

function Show-CompileCommands {
  Write-Host "Example compile commands:"
  foreach ($m in $modules) {
    $sketchDir = Join-Path $root $m.path
    $dest = Join-Path $outDir $m.key
    Write-Host ("arduino-cli compile -b {0} --output-dir `"{1}`" `"{2}`"" -f $m.fqbn, $dest, $sketchDir)
  }
}

$cli = Get-Command arduino-cli -ErrorAction SilentlyContinue
if (-not $cli) {
  Write-Host "arduino-cli was not found on PATH."
  Write-Host "Install it, install the board cores, then re-run this script."
  Write-Host "Hashes in release.json are left unchanged."
  Write-Host ""
  Show-CompileCommands
  exit 2
}

$releasePath = Join-Path $root "release.json"
$raw = Get-Content -Raw -Path $releasePath
$failed = $false

foreach ($m in $modules) {
  $sketchDir = Join-Path $root $m.path
  $dest = Join-Path $outDir $m.key
  $stage = Join-Path $outDir (".staging-" + $m.key)
  if (Test-Path $stage) {
    Remove-Item -Recurse -Force $stage
  }
  New-Item -ItemType Directory -Force -Path $stage | Out-Null

  Write-Host ""
  Write-Host ("Compiling {0} ({1})" -f $m.key, $m.fqbn)
  & arduino-cli compile -b $m.fqbn --output-dir $stage $sketchDir
  if ($LASTEXITCODE -ne 0) {
    Write-Host ("FAILED {0}" -f $m.key)
    $failed = $true
    continue
  }

  $artifact = Get-ChildItem -Path $stage -Filter ("*." + $m.ext) -File | Select-Object -First 1
  if (-not $artifact) {
    Write-Host ("FAILED {0}: no .{1} in {2}" -f $m.key, $m.ext, $stage)
    $failed = $true
    continue
  }

  if (Test-Path $dest) {
    Remove-Item -Recurse -Force $dest
  }
  New-Item -ItemType Directory -Force -Path $dest | Out-Null
  Copy-Item -Path (Join-Path $stage $artifact.Name) -Destination (Join-Path $dest $artifact.Name)
  Remove-Item -Recurse -Force $stage
  $artifact = Get-Item (Join-Path $dest $artifact.Name)

  $hash = (Get-FileHash -Algorithm SHA256 -Path $artifact.FullName).Hash.ToLowerInvariant()
  $relative = ("build/{0}/{1}" -f $m.key, $artifact.Name) -replace '\\', '/'
  Write-Host ("{0}  {1}" -f $hash, $relative)

  $pattern = '(?s)("' + [regex]::Escape($m.key) + '"\s*:\s*\{.*?"artifact"\s*:\s*)(?:"[^"]*"|null)(.*?"sha256"\s*:\s*)("[^"]*"|null)'
  $match = [regex]::Match($raw, $pattern)
  if (-not $match.Success) {
    Write-Host ("FAILED {0}: could not update release.json" -f $m.key)
    $failed = $true
    continue
  }
  $raw = $raw.Substring(0, $match.Groups[1].Index) +
    $match.Groups[1].Value + '"' + $relative + '"' +
    $match.Groups[2].Value + '"' + $hash + '"' +
    $raw.Substring($match.Groups[3].Index + $match.Groups[3].Length)
}

$utf8 = New-Object System.Text.UTF8Encoding $false
[System.IO.File]::WriteAllText($releasePath, $raw, $utf8)
if ($failed) {
  Write-Host ""
  Write-Host "One or more sketches failed. Successful hashes were written; failed modules stay null."
  exit 1
}

Write-Host ""
Write-Host "release.json updated."
exit 0
