# flash_all.ps1 -- Flash all binaries to ESP32-P4
# Usage: .\flash_all.ps1
# Uses $env:IDF_PATH and $env:IDF_PYTHON_ENV_PATH if set; auto-detects otherwise.
# Port defaults to COM30; override with $env:ESP_PORT.

$ErrorActionPreference = "Continue"

$ROOT = $PSScriptRoot
. (Join-Path $ROOT 'tools\Resolve-IdfEnv.ps1')
Initialize-IdfEnv

$BINS = Join-Path $ROOT 'firmware'
$PORT = if ($env:ESP_PORT) { $env:ESP_PORT } else { "COM30" }

# Kill any python processes holding the port
Stop-Process -Name python -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 2

# -- Partition table layout (must match partitions_ota.csv) -----
# Name       Offset     Bin file
$flash_map = @(
    @{ Offset = "0x2000";   File = "bootloader.bin";      Desc = "Bootloader" },
    @{ Offset = "0x8000";   File = "partition-table.bin";  Desc = "Partition Table" },
    @{ Offset = "0xD000";   File = "ota_data_initial.bin"; Desc = "OTA Data (boot factory)" },
    @{ Offset = "0x10000";  File = "launcher.bin";         Desc = "Launcher (factory)" },
    @{ Offset = "0x140000"; File = "nes_app.bin";          Desc = "NES (ota_0)" },
    @{ Offset = "0x240000"; File = "gb_app.bin";           Desc = "GB/GBC (ota_1)" },
    @{ Offset = "0x3C0000"; File = "snes_app.bin";         Desc = "SNES (ota_10)" },
    @{ Offset = "0x510000"; File = "genesis_app.bin";      Desc = "Genesis (ota_11)" },
    @{ Offset = "0x6D0000"; File = "neogeo_app.bin";       Desc = "Neo Geo (ota_12)" }
)

# Build the esptool command arguments
$args_list = @(
    "--chip", "esp32p4",
    "-p", $PORT,
    "-b", "460800",
    "--before", "default_reset",
    "--after", "hard_reset",
    "write_flash",
    "--flash_mode", "dio",
    "--flash_size", "16MB",
    "--flash_freq", "80m"
)

$missing = @()
foreach ($entry in $flash_map) {
    $path = "$BINS\$($entry.File)"
    if (Test-Path $path) {
        $size_kb = [math]::Round((Get-Item $path).Length / 1024)
        Write-Host "  $($entry.Desc): $($entry.File) (${size_kb}KB) @ $($entry.Offset)" -ForegroundColor Cyan
        $args_list += $entry.Offset
        $args_list += $path
    } else {
        Write-Host "  $($entry.Desc): MISSING $($entry.File) - skipping" -ForegroundColor Yellow
        $missing += $entry.Desc
    }
}

if ($missing.Count -gt 0) {
    Write-Host "`nWARNING: Skipping $($missing.Count) missing binaries: $($missing -join ', ')" -ForegroundColor Yellow
}

Write-Host "`n=== Flashing to $PORT ===" -ForegroundColor Green
python -m esptool $args_list

if ($LASTEXITCODE -eq 0) {
    Write-Host "=== Flash complete! ===" -ForegroundColor Green
} else {
    Write-Host "=== Flash FAILED ===" -ForegroundColor Red
    exit 1
}
