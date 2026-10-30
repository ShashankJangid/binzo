<#
.SYNOPSIS
    BINZO Firmware Compilation & Upload Helper

.PARAMETER Mecanum
    Build/Flash the 4-motor Mecanum main controller firmware.

.PARAMETER Cam
    Build/Flash the ESP32-CAM firmware.

.PARAMETER Port
    Serial port (e.g. COM3, /dev/ttyUSB0).

.PARAMETER CompileOnly
    Only compile without flashing.
#>
param (
    [switch]$Mecanum,
    [switch]$Cam,
    [string]$Port = "COM3",
    [switch]$CompileOnly
)

$ErrorActionPreference = "Stop"

if (-not $Mecanum -and -not $Cam) {
    Write-Host "Please specify either -Mecanum or -Cam" -ForegroundColor Yellow
    exit 1
}

if ($Mecanum) {
    $Sketch = "$PSScriptRoot\Binzo\firmware\Binzo_Mecanum\Binzo_Mecanum.ino"
    $Fqbn = "esp32:esp32:esp32"
    Write-Host "[BINZO] Target: Main Mecanum Controller ($Sketch)" -ForegroundColor Cyan
}

if ($Cam) {
    $Sketch = "$PSScriptRoot\Binzo\firmware\Binzo_Cam\Binzo_Cam.ino"
    $Fqbn = "esp32:esp32:esp32cam:PSRAM=enabled,PartitionScheme=huge_app"
    Write-Host "[BINZO] Target: ESP32-CAM ($Sketch)" -ForegroundColor Cyan
}

if ($CompileOnly) {
    Write-Host "[BINZO] Compiling sketch..." -ForegroundColor Green
    arduino-cli compile --fqbn $Fqbn $Sketch
} else {
    Write-Host "[BINZO] Compiling and uploading on $Port..." -ForegroundColor Green
    arduino-cli compile --fqbn $Fqbn $Sketch
    arduino-cli upload -p $Port --fqbn $Fqbn $Sketch
}

Write-Host "[BINZO] Done!" -ForegroundColor Green
