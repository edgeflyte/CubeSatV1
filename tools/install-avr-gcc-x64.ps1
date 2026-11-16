# Point every installed MiniCore at a 64-bit avr-gcc 7.3.0.
#
# Arduino's Windows avr-gcc 7.3.0-atmel3.6.1-arduino7 is a 32-bit compiler.
# On this host its preprocessor segfaults intermittently at avr/power.h
# (#include <stdint.h>). ASB pulls in enough translation units that the
# build almost always dies with "internal compiler error: Segmentation fault"
# or "when writing output to nul".
#
# Zak Kemble's 64-bit GCC 7.3.0 does not crash, but its avr-libc has no
# ATmega328PB startup file or device library. This script keeps that
# compiler and overlays the 328PB headers and libs from the Arduino toolchain.
#
# Usage (MiniCore and the Arduino AVR toolchain already installed):
#   powershell -File tools/install-avr-gcc-x64.ps1
# Restart the Arduino IDE if a compile still launches the 32-bit avr-g++.

$ErrorActionPreference = "Stop"

$packages = Join-Path $env:LOCALAPPDATA "Arduino15\packages"
$arduinoGcc = Join-Path $packages "arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7"
$x64Gcc = Join-Path $packages "arduino\tools\avr-gcc\7.3.0-x64-mingw"
$zipUrl = "https://github.com/ZakKemble/avr-gcc-build/releases/download/v7.3.0-1/avr-gcc-7.3.0-x64-mingw.zip"

if (-not (Test-Path (Join-Path $arduinoGcc "avr\include\avr\iom328pb.h"))) {
  throw "Arduino avr-gcc 7.3.0-atmel3.6.1-arduino7 was not found. Install MiniCore first so that toolchain is downloaded."
}

if (-not (Test-Path (Join-Path $x64Gcc "bin\avr-g++.exe"))) {
  $zip = Join-Path $env:TEMP "avr-gcc-7.3.0-x64-mingw.zip"
  $stage = Join-Path $env:TEMP "avr-gcc-7.3.0-x64-mingw"
  Write-Host "Downloading 64-bit avr-gcc 7.3.0..."
  Invoke-WebRequest -Uri $zipUrl -OutFile $zip
  if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
  Expand-Archive -Path $zip -DestinationPath $stage -Force
  $extracted = Join-Path $stage "avr-gcc-7.3.0-x64-mingw"
  if (-not (Test-Path (Join-Path $extracted "bin\avr-g++.exe"))) {
    throw "The toolchain zip did not contain bin\avr-g++.exe"
  }
  if (Test-Path $x64Gcc) { Remove-Item $x64Gcc -Recurse -Force }
  New-Item -ItemType Directory -Path (Split-Path $x64Gcc) -Force | Out-Null
  & robocopy $extracted $x64Gcc /E /NFL /NDL /NJH /NJS /nc /ns /np | Out-Null
  if ($LastExitCode -ge 8) { throw "robocopy failed with exit $LastExitCode" }
}

Write-Host "Overlaying ATmega328PB headers and libraries from the Arduino toolchain..."
& robocopy (Join-Path $arduinoGcc "avr\include") (Join-Path $x64Gcc "avr\include") /E /NFL /NDL /NJH /NJS /nc /ns /np | Out-Null
if ($LastExitCode -ge 8) { throw "robocopy of avr/include failed with exit $LastExitCode" }
Copy-Item (Join-Path $arduinoGcc "avr\lib\avr5\crtatmega328pb.o") (Join-Path $x64Gcc "avr\lib\avr5\crtatmega328pb.o") -Force
Copy-Item (Join-Path $arduinoGcc "avr\lib\avr5\libatmega328pb.a") (Join-Path $x64Gcc "avr\lib\avr5\libatmega328pb.a") -Force

$compilerPath = ($x64Gcc -replace '\\','/') + "/bin/"
$localTxt = @"
# Written by Firmware v2.1 tools/install-avr-gcc-x64.ps1.
# 64-bit avr-gcc 7.3.0. The stock 32-bit toolchain segfaults while preprocessing.
compiler.path=$compilerPath
"@

$cores = Get-ChildItem (Join-Path $packages "MiniCore\hardware\avr") -Directory -ErrorAction SilentlyContinue
if (-not $cores) { throw "No MiniCore hardware versions were found under $packages" }
foreach ($core in $cores) {
  $dest = Join-Path $core.FullName "platform.local.txt"
  Set-Content -Path $dest -Value $localTxt -Encoding ascii
  Write-Host "Updated $dest"
}

Write-Host "MiniCore will now compile with $compilerPath"
Write-Host "Restart the Arduino IDE if it is open, then rebuild ASB."
