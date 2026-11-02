<#
.SYNOPSIS
    BINZO Firmware Compilation & Upload Helper

.PARAMETER Mecanum
    Build/Flash the 4-motor Mecanum main controller firmware (L298N).

.PARAMETER Cam
    Build/Flash the ESP32-CAM firmware.

.PARAMETER M5Core2
    Build/Flash the M5Stack Core2 4-Channel Encoder Motor swerve drive firmware.

.PARAMETER Port
    Serial port (e.g. COM7, COM3, /dev/ttyUSB0).

.PARAMETER CompileOnly
    Only compile without flashing.
#>
param (
    [switch]$Mecanum,
    [switch]$Cam,
    [switch]$M5Core2,
    [string]$Port = "COM7",
    [switch]$CompileOnly
)

$ErrorActionPreference = "Stop"

if (-not $Mecanum -and -not $Cam -and -not $M5Core2) {
    Write-Host "Please specify either -M5Core2, -Mecanum, or -Cam" -ForegroundColor Yellow
    exit 1
}

if ($M5Core2) {
    $FirmwareDir = "$PSScriptRoot\m5core2_firmware"
    Write-Host "[BINZO] Target: M5Stack Core2 4EncoderMotor Controller ($FirmwareDir)" -ForegroundColor Cyan
    if ($CompileOnly) {
        Write-Host "[BINZO] Compiling M5Core2 PlatformIO firmware..." -ForegroundColor Green
        pio run -d $FirmwareDir
    } else {
        Write-Host "[BINZO] Compiling and uploading M5Core2 firmware on $Port..." -ForegroundColor Green
        pio run -d $FirmwareDir --target upload --upload-port $Port
    }
} elseif ($Mecanum) {
    $Sketch = "$PSScriptRoot\Binzo\firmware\Binzo_Mecanum\Binzo_Mecanum.ino"
    $Fqbn = "esp32:esp32:esp32"
    Write-Host "[BINZO] Target: Main Mecanum Controller ($Sketch)" -ForegroundColor Cyan
    if ($CompileOnly) {
        Write-Host "[BINZO] Compiling sketch..." -ForegroundColor Green
        arduino-cli compile --fqbn $Fqbn $Sketch
    } else {
        Write-Host "[BINZO] Compiling and uploading on $Port..." -ForegroundColor Green
        arduino-cli compile --fqbn $Fqbn $Sketch
        arduino-cli upload -p $Port --fqbn $Fqbn $Sketch
    }
} elseif ($Cam) {
    $Sketch = "$PSScriptRoot\Binzo\firmware\Binzo_Cam\Binzo_Cam.ino"
    $Fqbn = "esp32:esp32:esp32cam:PSRAM=enabled,PartitionScheme=huge_app"
    Write-Host "[BINZO] Target: ESP32-CAM ($Sketch)" -ForegroundColor Cyan
    if ($CompileOnly) {
        Write-Host "[BINZO] Compiling sketch..." -ForegroundColor Green
        arduino-cli compile --fqbn $Fqbn $Sketch
    } else {
        Write-Host "[BINZO] Compiling and uploading on $Port..." -ForegroundColor Green
        arduino-cli compile --fqbn $Fqbn $Sketch
        arduino-cli upload -p $Port --fqbn $Fqbn $Sketch
    }
}

Write-Host "[BINZO] Done!" -ForegroundColor Green
